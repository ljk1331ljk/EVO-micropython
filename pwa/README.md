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

Each device entry includes a `flash_sequence` array with the files and offsets
needed by ESP32 flashing tools. Each entry also includes a direct `url` field
that points at the matching GitHub Pages firmware file:

- `bootloader` at `0x1000`
- `partition_table` at `0x8000`
- `application` (`micropython.bin`) at `0x10000`

The manifest also includes SHA-256 hashes and file sizes for validation before
flashing.
