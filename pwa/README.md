# Firmware Release Assets for PWA

The firmware release workflow publishes these PWA-facing JSON assets on each
GitHub Release:

- `firmware-manifest.json`: stable machine-readable manifest for the release.
- `latest.json`: copy of the same manifest, intended for simple PWA lookup.

For the newest stable release, the PWA can fetch a version-independent manifest:

```text
https://github.com/ljk1331ljk/EVO-micropython/releases/download/latest/latest.json
```

For a pinned release, replace `latest` with the firmware version tag:

```text
https://github.com/ljk1331ljk/EVO-micropython/releases/download/v1.0.0/latest.json
```

Each device entry includes a `flash_sequence` array with the files and offsets
needed by ESP32 flashing tools:

- `bootloader` at `0x1000`
- `partition_table` at `0x8000`
- `application` (`micropython.bin`) at `0x10000`

The manifest also includes SHA-256 hashes and file sizes for validation before
flashing.
