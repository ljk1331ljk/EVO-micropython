# Firmware Release Assets for PWA

The firmware release workflow publishes firmware to GitHub Releases and mirrors
the PWA-facing files to GitHub Pages.

Stable tags use normal semantic versions, for example `v1.2.0`. Beta tags must
include `beta`, for example `v1.3.0-beta.1`.

For the newest stable release, the PWA can fetch a version-independent manifest:

```text
https://ljk1331ljk.github.io/EVO-micropython/firmware/latest/latest.json
```

For the newest beta release, the PWA can fetch:

```text
https://ljk1331ljk.github.io/EVO-micropython/firmware/beta/latest.json
```

For the full firmware picker, including all pinned stable versions and the
latest beta, the PWA can fetch:

```text
https://ljk1331ljk.github.io/EVO-micropython/firmware/index.json
```

Pinned release artifacts are still available from GitHub Releases. Stable
firmware is mirrored under `firmware/<tag>/` and `firmware/latest/`; beta
firmware is mirrored under `firmware/<tag>/` and `firmware/beta/`.

Each manifest includes:

- `firmware-manifest.json`: machine-readable manifest for the release.
- `latest.json`: copy of the same manifest for that release directory.
- `channel`: either `stable` or `beta`.
- `prerelease`: `true` for beta firmware.

Each device entry includes a `flash_sequence` array with the files and addresses
needed by ESP32 flashing tools:

- `bootloader.bin` at `0x0`
- `partition-table.bin` at `0x8000`
- `micropython.bin` at `0x10000`

Each device asset also includes a direct `url` field that points at the matching
GitHub Pages firmware file.

Each device entry can include `capabilities.multiple_programs`. The release
workflow writes this block for EVO targets:

- `supported`: `true` when the firmware/PWA pairing can store multiple user
  programs.
- `root`: folder where uploaded programs should be created, currently
  `/programs`.
- `main_file`: entry file the launcher should run inside the selected program
  folder, currently `main.py`.

When `capabilities.multiple_programs.supported` is `true`, the PWA may enable a
multi-program upload mode. In that mode, each uploaded program should be written
to its own folder under `root`, and the selected program should be launched from
that folder's `main_file`. If the field is missing or `supported` is `false`,
the PWA should keep the existing single-program upload behavior.

Connected devices also expose this setting at runtime. Over USB, read
`evo.get_multiple_program_filesystem()` or
`evo.get_config()["multiple_program_filesystem"]`. Over BLE, use `HELLO`,
`INFO`, or `EvoDownloadManager.status()` and read
`capabilities.multiple_programs.supported`. When enabled, BLE uploads may target
`/programs/<program-name>/main.py` instead of replacing `/main.py`; `/main.py`
remains the launcher selected by the PWA.

The BLE `LIST` command accepts a `path` and returns that directory's immediate
children. Use `/` for the root or a nested path such as `/programs/LineFollower`
to browse program folders.

The manifest also includes SHA-256 hashes and file sizes for validation before
flashing.
