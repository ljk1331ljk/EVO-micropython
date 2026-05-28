from micropython import const
from I2CDevice import *
import time

_HUSKY_I2C_ADDR = const(0x32)
_PROTO_ADDR     = const(0x11)

_HEADER_1 = const(0x55)
_HEADER_2 = const(0xAA)

# Request / return commands
_CMD_REQUEST                 = const(0x20)
_CMD_REQUEST_BLOCKS          = const(0x21)
_CMD_REQUEST_ARROWS          = const(0x22)
_CMD_REQUEST_LEARNED         = const(0x23)
_CMD_REQUEST_BLOCKS_LEARNED  = const(0x24)
_CMD_REQUEST_ARROWS_LEARNED  = const(0x25)
_CMD_REQUEST_BY_ID           = const(0x26)
_CMD_REQUEST_BLOCKS_BY_ID    = const(0x27)
_CMD_REQUEST_ARROWS_BY_ID    = const(0x28)
_CMD_RETURN_INFO             = const(0x29)
_CMD_RETURN_BLOCK            = const(0x2A)
_CMD_RETURN_ARROW            = const(0x2B)
_CMD_REQUEST_KNOCK           = const(0x2C)
_CMD_REQUEST_ALGORITHM       = const(0x2D)
_CMD_RETURN_OK               = const(0x2E)
_CMD_REQUEST_FORGET          = const(0x37)
_CMD_REQUEST_IS_PRO          = const(0x3B)
_CMD_RETURN_IS_PRO           = const(0x3B)
_CMD_RETURN_BUSY             = const(0x3D)
_CMD_RETURN_NEED_PRO         = const(0x3E)


# Algorithms
ALGORITHM_FACE_RECOGNITION   = const(0)
ALGORITHM_OBJECT_TRACKING    = const(1)
ALGORITHM_OBJECT_RECOGNITION = const(2)
ALGORITHM_LINE_TRACKING      = const(3)
ALGORITHM_COLOR_RECOGNITION  = const(4)
ALGORITHM_TAG_RECOGNITION    = const(5)
ALGORITHM_OBJECT_CLASSIFY    = const(6)


class EvoHuskyLens:
    def __init__(self, channel, i2c=I2CA, address=0x32, timeout_ms=200,
                 keep_last_on_fail=True, auto_reconnect=True):
        self.i2c = i2c
        self.channel = channel
        self.address = address
        self.timeout_ms = timeout_ms

        self.keep_last_on_fail = keep_last_on_fail
        self.auto_reconnect = auto_reconnect

        self._results = []
        self._count = 0
        self._learned_count = 0
        self._frame_number = 0

        self._connected = False
        self._last_error = None
        self._last_ok_ms = 0

    # -----------------------------
    # Mux channel enforcement
    # -----------------------------
    def _select(self):
        self.i2c.switch_channel(self.channel)

    # -----------------------------
    # Status helpers
    # -----------------------------
    def connected(self):
        return self._connected

    def lastError(self):
        return self._last_error

    def lastSuccessTime(self):
        return self._last_ok_ms

    def clearResults(self):
        self._results = []
        self._count = 0
        self._learned_count = 0
        self._frame_number = 0

    def _mark_ok(self):
        self._connected = True
        self._last_error = None
        self._last_ok_ms = time.ticks_ms()

    def _mark_fail(self, err):
        self._connected = False
        self._last_error = err

    # -----------------------------
    # Low-level I2C helpers
    # -----------------------------
    def _checksum(self, buf):
        s = 0
        for b in buf:
            s = (s + b) & 0xFF
        return s

    def _build_frame(self, command, data=None):
        if data is None:
            data = b""

        length = len(data)
        frame = bytearray(5 + length + 1)
        frame[0] = _HEADER_1
        frame[1] = _HEADER_2
        frame[2] = _PROTO_ADDR
        frame[3] = length
        frame[4] = command

        for i in range(length):
            frame[5 + i] = data[i]

        frame[-1] = self._checksum(frame[:-1])
        return frame

    def _write_frame(self, command, data=None):
        self._select()
        frame = self._build_frame(command, data)
        self.i2c.writeto(self.channel, self.address, frame)

    def _read_frame(self):
        start = time.ticks_ms()

        while True:
            try:
                self._select()
                hdr = self.i2c.readfrom(self.channel, self.address, 5)
                if len(hdr) == 5 and hdr[0] == _HEADER_1 and hdr[1] == _HEADER_2:
                    break
            except OSError:
                pass

            if time.ticks_diff(time.ticks_ms(), start) > self.timeout_ms:
                raise OSError("HuskyLens read timeout")

            time.sleep_ms(5)

        data_len = hdr[3]

        self._select()
        rest = self.i2c.readfrom(self.channel, self.address, data_len + 1)

        frame = bytearray(5 + data_len + 1)
        frame[0:5] = hdr
        frame[5:] = rest

        if self._checksum(frame[:-1]) != frame[-1]:
            raise ValueError("HuskyLens checksum error")

        return frame

    def _u16(self, lo, hi):
        return lo | (hi << 8)

    def _parse_info(self, data):
        self._count = self._u16(data[0], data[1])
        self._learned_count = self._u16(data[2], data[3])
        self._frame_number = self._u16(data[4], data[5])

    def _parse_result(self, cmd, data):
        if cmd == _CMD_RETURN_BLOCK:
            return {
                "type": "block",
                "xCenter": self._u16(data[0], data[1]),
                "yCenter": self._u16(data[2], data[3]),
                "width": self._u16(data[4], data[5]),
                "height": self._u16(data[6], data[7]),
                "ID": self._u16(data[8], data[9]),
            }

        if cmd == _CMD_RETURN_ARROW:
            return {
                "type": "arrow",
                "xOrigin": self._u16(data[0], data[1]),
                "yOrigin": self._u16(data[2], data[3]),
                "xTarget": self._u16(data[4], data[5]),
                "yTarget": self._u16(data[6], data[7]),
                "ID": self._u16(data[8], data[9]),
            }

        return None

    def _try_reconnect(self):
        try:
            return self.knock()
        except Exception as e:
            self._mark_fail(e)
            return False

    def _handle_request_failure(self, err):
        self._mark_fail(err)

        if self.auto_reconnect:
            self._try_reconnect()

        if not self.keep_last_on_fail:
            self.clearResults()

        return False

    def _request_and_read(self, command, data=None):
        # Keep previous results until a full new read succeeds.
        old_results = self._results[:]
        old_count = self._count
        old_learned_count = self._learned_count
        old_frame_number = self._frame_number

        try:
            temp_results = []
            temp_count = 0
            temp_learned_count = 0
            temp_frame_number = 0

            self._write_frame(command, data)

            first = self._read_frame()
            cmd = first[4]
            data_len = first[3]
            payload = first[5:5 + data_len]

            if cmd == _CMD_RETURN_BUSY:
                raise OSError("HuskyLens busy")

            if cmd == _CMD_RETURN_NEED_PRO:
                raise OSError("HuskyLens feature requires pro model")

            if cmd == _CMD_RETURN_OK:
                self._results = []
                self._count = 0
                self._learned_count = 0
                self._frame_number = 0
                self._mark_ok()
                return True

            if cmd != _CMD_RETURN_INFO:
                raise ValueError("Unexpected HuskyLens response: %d" % cmd)

            temp_count = self._u16(payload[0], payload[1])
            temp_learned_count = self._u16(payload[2], payload[3])
            temp_frame_number = self._u16(payload[4], payload[5])

            for _ in range(temp_count):
                frm = self._read_frame()
                rcmd = frm[4]
                rlen = frm[3]
                rdat = frm[5:5 + rlen]
                result = self._parse_result(rcmd, rdat)
                if result is not None:
                    temp_results.append(result)

            # Commit only after full successful transaction
            self._results = temp_results
            self._count = temp_count
            self._learned_count = temp_learned_count
            self._frame_number = temp_frame_number
            self._mark_ok()
            return True

        except Exception as e:
            # Restore previous good data
            self._results = old_results
            self._count = old_count
            self._learned_count = old_learned_count
            self._frame_number = old_frame_number
            return self._handle_request_failure(e)

    # -----------------------------
    # Basic API
    # -----------------------------
    def begin(self):
        return self.knock()

    def knock(self):
        try:
            self._write_frame(_CMD_REQUEST_KNOCK)
            frame = self._read_frame()
            ok = (frame[4] == _CMD_RETURN_OK)
            if ok:
                self._mark_ok()
            else:
                self._mark_fail(OSError("HuskyLens knock failed"))
            return ok
        except Exception as e:
            self._mark_fail(e)
            return False

    def setTimeout(self, timeout_ms):
        self.timeout_ms = int(timeout_ms)

    def setAlgorithm(self, algorithm):
        data = bytearray(2)
        data[0] = algorithm & 0xFF
        data[1] = (algorithm >> 8) & 0xFF

        try:
            self._write_frame(_CMD_REQUEST_ALGORITHM, data)
            frame = self._read_frame()

            if frame[4] == _CMD_RETURN_OK:
                self._mark_ok()
                return True
            if frame[4] == _CMD_RETURN_BUSY:
                raise OSError("HuskyLens busy")

            raise ValueError("Failed to set algorithm")
        except Exception as e:
            return self._handle_request_failure(e)

    def forget(self):
        try:
            self._write_frame(_CMD_REQUEST_FORGET)
            frame = self._read_frame()
            ok = (frame[4] == _CMD_RETURN_OK)
            if ok:
                self._mark_ok()
            else:
                self._mark_fail(OSError("HuskyLens forget failed"))
            return ok
        except Exception as e:
            return self._handle_request_failure(e)

    def isPro(self):
        try:
            self._write_frame(_CMD_REQUEST_IS_PRO)
            frame = self._read_frame()

            if frame[4] != _CMD_RETURN_IS_PRO:
                self._mark_fail(OSError("Unexpected isPro response"))
                return False

            data_len = frame[3]
            data = frame[5:5 + data_len]
            self._mark_ok()
            return self._u16(data[0], data[1]) == 1
        except Exception as e:
            self._handle_request_failure(e)
            return False

    # -----------------------------
    # Request functions
    # -----------------------------
    def request(self):
        return self._request_and_read(_CMD_REQUEST)

    def requestBlocks(self):
        return self._request_and_read(_CMD_REQUEST_BLOCKS)

    def requestArrows(self):
        return self._request_and_read(_CMD_REQUEST_ARROWS)

    def requestLearned(self):
        return self._request_and_read(_CMD_REQUEST_LEARNED)

    def requestBlocksLearned(self):
        return self._request_and_read(_CMD_REQUEST_BLOCKS_LEARNED)

    def requestArrowsLearned(self):
        return self._request_and_read(_CMD_REQUEST_ARROWS_LEARNED)

    def requestByID(self, id_):
        data = bytearray(2)
        data[0] = id_ & 0xFF
        data[1] = (id_ >> 8) & 0xFF
        return self._request_and_read(_CMD_REQUEST_BY_ID, data)

    def requestBlocksByID(self, id_):
        data = bytearray(2)
        data[0] = id_ & 0xFF
        data[1] = (id_ >> 8) & 0xFF
        return self._request_and_read(_CMD_REQUEST_BLOCKS_BY_ID, data)

    def requestArrowsByID(self, id_):
        data = bytearray(2)
        data[0] = id_ & 0xFF
        data[1] = (id_ >> 8) & 0xFF
        return self._request_and_read(_CMD_REQUEST_ARROWS_BY_ID, data)

    # -----------------------------
    # Result access
    # -----------------------------
    def available(self):
        return len(self._results)

    def read(self):
        if not self._results:
            return None
        return self._results.pop(0)

    def readAll(self):
        out = self._results[:]
        self._results = []
        return out

    def results(self):
        return self._results

    def count(self):
        return self._count

    def countLearnedIDs(self):
        return self._learned_count

    def frameNumber(self):
        return self._frame_number

    def isLearned(self):
        return self._learned_count > 0

    # -----------------------------
    # Handy helpers
    # -----------------------------
    def getBlocks(self):
        return [r for r in self._results if r["type"] == "block"]

    def getArrows(self):
        return [r for r in self._results if r["type"] == "arrow"]

    def getFirstBlock(self):
        for r in self._results:
            if r["type"] == "block":
                return r
        return None

    def getFirstArrow(self):
        for r in self._results:
            if r["type"] == "arrow":
                return r
        return None

    def getBlocksByID(self, id_):
        return [r for r in self._results if r["type"] == "block" and r["ID"] == id_]

    def getArrowsByID(self, id_):
        return [r for r in self._results if r["type"] == "arrow" and r["ID"] == id_]


# -----------------------------
# Simple test
# -----------------------------
if __name__ == "__main__":
    husky = EvoHuskyLens(I2C1)

    print("Connected:", husky.begin())

    husky.setAlgorithm(ALGORITHM_OBJECT_TRACKING)

    while True:
        ok = husky.requestBlocks()

        print(
            "ok:", ok,
            "connected:", husky.connected(),
            "count:", husky.count(),
            "learned:", husky.countLearnedIDs(),
            "frame:", husky.frameNumber(),
            "err:", husky.lastError()
        )

        block = husky.getFirstBlock()
        if block:
            print(block)
            print(block["xCenter"], block["yCenter"], block["width"], block["height"], block["ID"])
        else:
            print("No block")

        time.sleep_ms(100)