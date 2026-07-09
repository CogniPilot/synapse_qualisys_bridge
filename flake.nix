{
  description = "Development and CI environment for synapse_qualisys_bridge";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { nixpkgs, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems =
        f:
        nixpkgs.lib.genAttrs systems (
          system:
          f (
            import nixpkgs {
              inherit system;
            }
          )
        );
    in
    {
      devShells = forAllSystems (pkgs: {
        default = pkgs.mkShell {
          packages = with pkgs; [
            cargo
            clippy
            git
            nodejs_24
            openssl
            pkg-config
            playwright-test
            rustc
            rustfmt
            zenoh
          ];

          nativeBuildInputs = [ pkgs.pkg-config ];
          buildInputs = [ pkgs.openssl ];

          shellHook = ''
            echo "synapse_qualisys_bridge CI toolchain loaded"
          '';
        };
      });

      apps = forAllSystems (
        pkgs:
        let
          ci = pkgs.writeShellApplication {
            name = "synapse-qualisys-bridge-ci";
            runtimeInputs = with pkgs; [
              bash
              cargo
              clippy
              git
              nodejs_24
              openssl
              pkg-config
              playwright-test
              rustc
              rustfmt
              zenoh
            ];
            text = ''
              exec bash "$PWD/scripts/ci.sh"
            '';
          };
        in
        {
          default = {
            type = "app";
            program = "${ci}/bin/synapse-qualisys-bridge-ci";
          };

          ci = {
            type = "app";
            program = "${ci}/bin/synapse-qualisys-bridge-ci";
          };
        }
      );

      formatter = forAllSystems (pkgs: pkgs.nixfmt);
    };
}
