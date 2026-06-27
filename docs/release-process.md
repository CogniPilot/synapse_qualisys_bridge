# Release process

GitHub Actions publishes binaries when a `v*` tag is pushed.

The release workflow:

1. Checks out this repository and `CogniPilot/qualisys_rust_sdk` side-by-side
   so the local `../qualisys_rust_sdk` dependency resolves in CI.
2. Builds `synapse-qualisys-bridge` for Linux, Windows, and macOS.
   The Linux build targets `x86_64-unknown-linux-musl`, the Windows build uses
   static CRT linking, and the macOS build is a universal binary for Intel and
   Apple Silicon Macs.
3. Packages the binaries with `README.md`, `LICENSE`, and
   `SYNAPSE_FBS_COMPATIBILITY.txt`. The Windows archive also includes the
   default `bridge.toml`.
4. Builds a Windows Inno Setup installer that installs the bridge, Start Menu
   shortcuts, a preserved per-user `bridge.toml`, and an optional login startup
   shortcut. The installed bridge runs Zenoh in embedded router mode by default,
   listens on UDP and TCP port 7447 by default, and does not need to bundle
   `zenohd`.
5. Publishes `.sha256` checksums for the archives and installer.
6. Creates or updates the GitHub release for the tag with the GitHub CLI and
   uploads the archives.
7. Updates the release notes with the locked `synapse_fbs` version from
   `Cargo.lock`.

Before publishing a release, make sure `Cargo.lock` contains the intended
`synapse_fbs` version. The release notes and packaged compatibility file are
the compatibility record for that release.
