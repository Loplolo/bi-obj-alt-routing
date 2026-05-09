{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };
  outputs = { self, nixpkgs }:
    let
      supportedSystems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
      pkgsFor = system: import nixpkgs { inherit system; };
    in {
      devShells = forAllSystems (system:
        let pkgs = pkgsFor system;
        in {
          default = pkgs.mkShell {
            packages = with pkgs; [
              cmake
              gnumake
              gcc
              zlib
              libosmium
              protozero
              boost
              gdb
              valgrind
              clang-tools
            ];
            shellHook = ''
              export CPATH="${pkgs.zlib.dev}/include:$CPATH"
              cat > .clangd << EOF
CompileFlags:
  CompilationDatabase: ./build
  Add:
    - "-I${pkgs.zlib.dev}/include"
EOF
            '';
          };
        });
    };
}
