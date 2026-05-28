# Supported Architectures

This document lists all supported target architectures that can be specified by setting `TARGET_ARCHITECTURE` on the CMake CLI. Only use this explicitly if you know what you are doing!

## x86/x86_64

| Vendor | Codename / CPU Microarchitecture | Family | Name | CMake Flag |
|:--|:--|:--|:--|:--|
| Intel | Core | x86 / x86_64 | core | `TARGET_ARCHITECTURE=core` |
| Intel | Core 2 | x86 / x86_64 | core2 | `TARGET_ARCHITECTURE=core2` |
| Intel | Merom (65nm Core2) | x86 / x86_64 | merom | `TARGET_ARCHITECTURE=merom` |
| Intel | Penryn (45nm Core2) | x86 / x86_64 | penryn | `TARGET_ARCHITECTURE=penryn` |
| Intel | Nehalem | x86 / x86_64 | nehalem | `TARGET_ARCHITECTURE=nehalem` |
| Intel | Westmere | x86 / x86_64 | westmere | `TARGET_ARCHITECTURE=westmere` |
| Intel | Sandy Bridge | x86 / x86_64 | sandybridge | `TARGET_ARCHITECTURE=sandybridge` |
| Intel | Ivy Bridge | x86 / x86_64 | ivybridge | `TARGET_ARCHITECTURE=ivybridge` |
| Intel | Haswell | x86 / x86_64 | haswell | `TARGET_ARCHITECTURE=haswell` |
| Intel | Broadwell | x86 / x86_64 | broadwell | `TARGET_ARCHITECTURE=broadwell` |
| Intel | Skylake | x86 / x86_64 | skylake | `TARGET_ARCHITECTURE=skylake` |
| Intel | Skylake-X (Xeon) | x86 / x86_64 | skylake-xeon | `TARGET_ARCHITECTURE=skylake-xeon` |
| Intel | Kaby Lake | x86 / x86_64 | kabylake | `TARGET_ARCHITECTURE=kabylake` |
| Intel | Cannon Lake | x86 / x86_64 | cannonlake | `TARGET_ARCHITECTURE=cannonlake` |
| Intel | Cascade Lake | x86 / x86_64 | cascadelake | `TARGET_ARCHITECTURE=cascadelake` |
| Intel | Cooper Lake | x86 / x86_64 | cooperlake | `TARGET_ARCHITECTURE=cooperlake` |
| Intel | Ice Lake | x86 / x86_64 | icelake | `TARGET_ARCHITECTURE=icelake` |
| Intel | Ice Lake Xeon | x86 / x86_64 | icelake-xeon | `TARGET_ARCHITECTURE=icelake-xeon` |
| Intel | Tiger Lake | x86 / x86_64 | tigerlake | `TARGET_ARCHITECTURE=tigerlake` |
| Intel | Alder Lake | x86 / x86_64 | alderlake | `TARGET_ARCHITECTURE=alderlake` |
| Intel | Sapphire Rapids | x86 / x86_64 | sapphirerapids | `TARGET_ARCHITECTURE=sapphirerapids` |
| Intel | Rocket Lake | x86 / x86_64 | rocketlake | `TARGET_ARCHITECTURE=rocketlake` |
| Intel | Raptor Lake | x86 / x86_64 | raptorlake | `TARGET_ARCHITECTURE=raptorlake` |
| Intel | Bonnell | x86 / x86_64 | bonnell | `TARGET_ARCHITECTURE=bonnell` |
| Intel | Silvermont | x86 / x86_64 | silvermont | `TARGET_ARCHITECTURE=silvermont` |
| Intel | Goldmont | x86 / x86_64 | goldmont | `TARGET_ARCHITECTURE=goldmont` |
| Intel | Goldmont Plus | x86 / x86_64 | goldmont-plus | `TARGET_ARCHITECTURE=goldmont-plus` |
| Intel | Tremont | x86 / x86_64 | tremont | `TARGET_ARCHITECTURE=tremont` |
| Intel | Knights Landing | x86 / x86_64 | knl | `TARGET_ARCHITECTURE=knl` |
| Intel | Knights Mill | x86 / x86_64 | knm | `TARGET_ARCHITECTURE=knm` |
| Intel | Atom (generic) | x86 / x86_64 | atom | `TARGET_ARCHITECTURE=atom` |
| AMD | K8 | x86 / x86_64 | k8 | `TARGET_ARCHITECTURE=k8` |
| AMD | K8 SSE3 | x86 / x86_64 | k8-sse3 | `TARGET_ARCHITECTURE=k8-sse3` |
| AMD | Barcelona | x86 / x86_64 | barcelona | `TARGET_ARCHITECTURE=barcelona` |
| AMD | Istanbul | x86 / x86_64 | istanbul | `TARGET_ARCHITECTURE=istanbul` |
| AMD | Magny-Cours | x86 / x86_64 | magny-cours | `TARGET_ARCHITECTURE=magny-cours` |
| AMD | Bulldozer | x86 / x86_64 | bulldozer | `TARGET_ARCHITECTURE=bulldozer` |
| AMD | Interlagos | x86 / x86_64 | interlagos | `TARGET_ARCHITECTURE=interlagos` |
| AMD | Piledriver | x86 / x86_64 | piledriver | `TARGET_ARCHITECTURE=piledriver` |
| AMD | Steamroller | x86 / x86_64 | steamroller | `TARGET_ARCHITECTURE=steamroller` |
| AMD | Excavator | x86 / x86_64 | excavator | `TARGET_ARCHITECTURE=excavator` |
| AMD | Family 14h | x86 / x86_64 | amd14h | `TARGET_ARCHITECTURE=amd14h` |
| AMD | Family 16h | x86 / x86_64 | amd16h | `TARGET_ARCHITECTURE=amd16h` |
| AMD | Zen | x86 / x86_64 | zen | `TARGET_ARCHITECTURE=zen` |
| AMD | Zen 2 | x86 / x86_64 | zen2 | `TARGET_ARCHITECTURE=zen2` |
| AMD | Zen 3 | x86 / x86_64 | zen3 | `TARGET_ARCHITECTURE=zen3` |
| AMD | Zen 4 | x86 / x86_64 | zen4 | `TARGET_ARCHITECTURE=zen4` |
| Generic | Generic | x86 / x86_64 | generic | `TARGET_ARCHITECTURE=generic` |
| Generic | None (no optimization) | x86 / x86_64 | none | `TARGET_ARCHITECTURE=none` |
| Generic | Auto-detect host CPU | x86 / x86_64 | auto | `TARGET_ARCHITECTURE=auto` |
| Generic | Compiler “native” | x86 / x86_64 | native | `TARGET_ARCHITECTURE=native` |

## ARM/ARM64

| Vendor | Codename / CPU Microarchitecture | Family | Name | CMake Flag |
|:--|:--|:--|:--|:--|
| Fujitsu | A64FX | arm64 | a64fx | `TARGET_ARCHITECTURE=a64fx` |
| Apple | A6 | arm64 | apple-a6 | `TARGET_ARCHITECTURE=apple-a6` |
| Apple | A7 | arm64 | apple-a7 | `TARGET_ARCHITECTURE=apple-a7` |
| Apple | A8 | arm64 | apple-a8 | `TARGET_ARCHITECTURE=apple-a8` |
| Apple | A9 | arm64 | apple-a9 | `TARGET_ARCHITECTURE=apple-a9` |
| Apple | A10 | arm64 | apple-a10 | `TARGET_ARCHITECTURE=apple-a10` |
| Apple | A11 | arm64 | apple-a11 | `TARGET_ARCHITECTURE=apple-a11` |
| Apple | A12 | arm64 | apple-a12 | `TARGET_ARCHITECTURE=apple-a12` |
| Apple | A13 | arm64 | apple-a13 | `TARGET_ARCHITECTURE=apple-a13` |
| Apple | A14 | arm64 | apple-a14 | `TARGET_ARCHITECTURE=apple-a14` |
| Apple | A15 | arm64 | apple-a15 | `TARGET_ARCHITECTURE=apple-a15` |
| Apple | A16 | arm64 | apple-a16 | `TARGET_ARCHITECTURE=apple-a16` |
| Apple | M1 | arm64 | apple-m1 | `TARGET_ARCHITECTURE=apple-m1` |
| Apple | M2 | arm64 | apple-m2 | `TARGET_ARCHITECTURE=apple-m2` |
| Apple | M3 | arm64 | apple-m3 | `TARGET_ARCHITECTURE=apple-m3` |
| Apple | M4 | arm64 | apple-m4 | `TARGET_ARCHITECTURE=apple-m4` |
| ARM | Cortex-A5 | arm / arm64 | cortex-a5 | `TARGET_ARCHITECTURE=cortex-a5` |
| ARM | Cortex-A7 | arm / arm64 | cortex-a7 | `TARGET_ARCHITECTURE=cortex-a7` |
| ARM | Cortex-A8 | arm / arm64 | cortex-a8 | `TARGET_ARCHITECTURE=cortex-a8` |
| ARM | Cortex-A9 | arm / arm64 | cortex-a9 | `TARGET_ARCHITECTURE=cortex-a9` |
| ARM | Cortex-A15 | arm / arm64 | cortex-a15 | `TARGET_ARCHITECTURE=cortex-a15` |
| ARM | Cortex-A17 | arm / arm64 | cortex-a17 | `TARGET_ARCHITECTURE=cortex-a17` |
| ARM | Cortex-A32 | arm / arm64 | cortex-a32 | `TARGET_ARCHITECTURE=cortex-a32` |
| ARM | Cortex-A35 | arm / arm64 | cortex-a35 | `TARGET_ARCHITECTURE=cortex-a35` |
| ARM | Cortex-A53 | arm / arm64 | cortex-a53 | `TARGET_ARCHITECTURE=cortex-a53` |
| ARM | Cortex-A55 | arm / arm64 | cortex-a55 | `TARGET_ARCHITECTURE=cortex-a55` |
| ARM | Cortex-A57 | arm / arm64 | cortex-a57 | `TARGET_ARCHITECTURE=cortex-a57` |
| ARM | Cortex-A72 | arm / arm64 | cortex-a72 | `TARGET_ARCHITECTURE=cortex-a72` |
| ARM | Cortex-A73 | arm / arm64 | cortex-a73 | `TARGET_ARCHITECTURE=cortex-a73` |
| ARM | Cortex-A75 | arm / arm64 | cortex-a75 | `TARGET_ARCHITECTURE=cortex-a75` |
| ARM | Cortex-A76 | arm / arm64 | cortex-a76 | `TARGET_ARCHITECTURE=cortex-a76` |
| ARM | Cortex-A76AE | arm / arm64 | cortex-a76ae | `TARGET_ARCHITECTURE=cortex-a76ae` |
| ARM | Cortex-A77 | arm / arm64 | cortex-a77 | `TARGET_ARCHITECTURE=cortex-a77` |
| ARM | Cortex-A78 | arm / arm64 | cortex-a78 | `TARGET_ARCHITECTURE=cortex-a78` |
| ARM | Cortex-A78AE | arm / arm64 | cortex-a78ae | `TARGET_ARCHITECTURE=cortex-a78ae` |
| ARM | Cortex-A510 | arm / arm64 | cortex-a510 | `TARGET_ARCHITECTURE=cortex-a510` |
| ARM | Cortex-A710 | arm / arm64 | cortex-a710 | `TARGET_ARCHITECTURE=cortex-a710` |
| ARM | Cortex-X1 | arm / arm64 | cortex-x1 | `TARGET_ARCHITECTURE=cortex-x1` |
| ARM | Cortex-X2 | arm / arm64 | cortex-x2 | `TARGET_ARCHITECTURE=cortex-x2` |
| ARM | Neoverse E1 | arm64 | neoverse-e1 | `TARGET_ARCHITECTURE=neoverse-e1` |
| ARM | Neoverse N1 | arm64 | neoverse-n1 | `TARGET_ARCHITECTURE=neoverse-n1` |
| ARM | Neoverse N2 | arm64 | neoverse-n2 | `TARGET_ARCHITECTURE=neoverse-n2` |
| ARM | Neoverse V1 | arm64 | neoverse-v1 | `TARGET_ARCHITECTURE=neoverse-v1` |
| Qualcomm | Krait | arm / arm64 | krait | `TARGET_ARCHITECTURE=krait` |
| Qualcomm | Kryo | arm64 | kryo | `TARGET_ARCHITECTURE=kryo` |
| Qualcomm | Kryo 2 | arm64 | kryo2 | `TARGET_ARCHITECTURE=kryo2` |
| Cavium | ThunderX | arm64 | thunderx | `TARGET_ARCHITECTURE=thunderx` |
| Cavium | ThunderX2 | arm64 | thunderx2 | `TARGET_ARCHITECTURE=thunderx2` |
| Cavium | ThunderX2T99 | arm64 | thunderx2t99 | `TARGET_ARCHITECTURE=thunderx2t99` |
| Cavium | ThunderXT81 | arm64 | thunderxt81 | `TARGET_ARCHITECTURE=thunderxt81` |
| Cavium | ThunderXT83 | arm64 | thunderxt83 | `TARGET_ARCHITECTURE=thunderxt83` |
| Cavium | ThunderXT88 | arm64 | thunderxt88 | `TARGET_ARCHITECTURE=thunderxt88` |
| Marvell | PJ4 | arm / arm64 | marvell-pj4 | `TARGET_ARCHITECTURE=marvell-pj4` |
| Marvell | F | arm / arm64 | marvell-f | `TARGET_ARCHITECTURE=marvell-f` |
| Marvell | XScale | arm / arm64 | xscale | `TARGET_ARCHITECTURE=xscale` |
| Broadcom | Brahma B15 | arm / arm64 | brahma-b15 | `TARGET_ARCHITECTURE=brahma-b15` |
| Broadcom | Brahma B53 | arm / arm64 | brahma-b53 | `TARGET_ARCHITECTURE=brahma-b53` |
| Applied Micro | X-Gene 1 | arm64 | xgene1 | `TARGET_ARCHITECTURE=xgene1` |
| Generic | Generic | arm / arm64 | generic | `TARGET_ARCHITECTURE=generic` |
| Generic | None (no optimization) | arm / arm64 | none | `TARGET_ARCHITECTURE=none` |
| Generic | Auto-detect host CPU | arm / arm64 | auto | `TARGET_ARCHITECTURE=auto` |
| Generic | Compiler “native” | arm / arm64 | native | `TARGET_ARCHITECTURE=native` |

## PPC

| Vendor | Codename / CPU Microarchitecture | Family | Name | CMake Flag |
|:--|:--|:--|:--|:--|
| IBM | POWER8 | PPC | power8 | `TARGET_ARCHITECTURE=power8` |
| IBM | POWER9 | PPC | power9 | `TARGET_ARCHITECTURE=power9` |
| IBM | POWER10 | PPC | power10 | `TARGET_ARCHITECTURE=power10` |
| Generic | Generic | PPC | generic | `TARGET_ARCHITECTURE=generic` |
| Generic | None (no optimization) | PPC | none | `TARGET_ARCHITECTURE=none` |
| Generic | Auto-detect host CPU | PPC | auto | `TARGET_ARCHITECTURE=auto` |
| Generic | Compiler “native” | PPC | native | `TARGET_ARCHITECTURE=native` |
