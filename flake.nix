{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let pkgs = import nixpkgs { inherit system; };
      in with pkgs; {
        devShells.default = mkShell {
          buildInputs = with pkgs; [
            llvmPackages_16.stdenv
            wget
            python311
            doxygen
            graphviz
            ghostscript
            cmake
            assimp
            boost
            ceres-solver
            clp
            eigen
            expat
            flann
            geogram
            lemon-graph
            lz4
            nanoflann
            openexr
            openimageio
            zlib
            llvmPackages_16.openmp
            alembic
            cctag
            opencv
            onnxruntime
            pkg-config
          ];
        };
      });
}
