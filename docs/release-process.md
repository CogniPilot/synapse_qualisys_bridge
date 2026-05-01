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
   `SYNAPSE_FBS_COMPATIBILITY.txt`.
4. Publishes `.sha256` checksums for the archives.
5. Creates or updates the GitHub release for the tag with the GitHub CLI and
   uploads the archives.
6. Updates the release notes with the locked `synapse_fbs` version from
   `Cargo.lock`.

Before publishing a release, make sure `Cargo.lock` contains the intended
`synapse_fbs` version. The release notes and packaged compatibility file are
the compatibility record for that release.
