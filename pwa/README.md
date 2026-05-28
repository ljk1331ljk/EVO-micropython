# Firmware Release Assets for PWA

The firmware release workflow publishes firmware to GitHub Releases and mirrors
the PWA-facing files to GitHub Pages:

- `firmware-manifest.json`: stable machine-readable manifest for the release.
- `latest.json`: copy of the same manifest, intended for simple PWA lookup.

For the newest stable release, the PWA can fetch a version-independent manifest:

```text
https://ljk1331ljk.github.io/EVO-micropython/firmware/latest/latest.json
```

Pinned release artifacts are still available from GitHub Releases. The Pages
endpoint is intended for the PWA's newest stable firmware lookup.

Each device entry includes a `flash_sequence` array with the files and offsets
needed by ESP32 flashing tools. Each entry also includes a direct `url` field
that points at the matching GitHub Pages firmware file:

- `bootloader` at `0x1000`
- `partition_table` at `0x8000`
- `application` (`micropython.bin`) at `0x10000`

The manifest also includes SHA-256 hashes and file sizes for validation before
flashing.
