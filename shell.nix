{ pkgs ? import <nixpkgs> {} }:

pkgs.pkgsCross.sheevaplug.callPackage (

  { stdenv
  , lib
  , mkShell
  , buildPackages
  , linux_latest
  }:

  mkShell {
    buildInputs = linux_latest.buildInputs ++ [
    ];
    nativeBuildInputs = linux_latest.nativeBuildInputs ++ [
    ];

    CROSS_COMPILE = stdenv.cc.targetPrefix;

    makeFlags = [
      "O=$(buildRoot)"
      "CC=${stdenv.cc}/bin/${stdenv.cc.targetPrefix}cc"
      "HOSTCC=${buildPackages.stdenv.cc}/bin/${buildPackages.stdenv.cc.targetPrefix}cc"
      "HOSTLD=${buildPackages.stdenv.cc.bintools}/bin/${buildPackages.stdenv.cc.targetPrefix}ld"
      "ARCH=${stdenv.hostPlatform.linuxArch}"
    ] ++ lib.optionals (stdenv.hostPlatform != stdenv.buildPlatform) [
    "CROSS_COMPILE=${stdenv.cc.targetPrefix}"
    ];

    shellHook = ''
      cat <<EOF
      TIP:
      Use \`make \$makeFlags\` for make invocations.
      EOF
    '';
  }

) { }
