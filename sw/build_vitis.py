#!/usr/bin/env python3
"""Reproducible Vitis 2024.2 Unified IDE CLI build for ZUBoard 1CG firmware.

Builds:
  1. Platform component (`zub1cg_<project>_plat`) from hw/build/zub1cg_<project>.xsa
     - Cortex-A53 domain (`standalone` + `lwip220`)
     - Cortex-R5F domain (`standalone`)
  2. Application `a53_net_bridge` (lwIP UDP <-> AXI-Lite & AXI-DMA bridge)
  3. Application `r5f_realtime`   (1 kHz deterministic CVG supervisor)

Usage:
  vitis -s sw/build_vitis.py [stage1_mems|stage0]
"""

import os
from pathlib import Path
import shutil
import sys
import vitis


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    project = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PROJECT", "stage1_mems")
    proj_name = f"zub1cg_{project}"
    xsa_path = repo_root / "hw" / "build" / f"{proj_name}.xsa"
    if not xsa_path.is_file():
        raise FileNotFoundError(f"Missing XSA: {xsa_path}. Run 'make bit PROJECT={project}' first.")

    ws_dir = repo_root / "sw" / "build" / f"ws_{project}"
    if ws_dir.exists():
        shutil.rmtree(ws_dir)
    ws_dir.mkdir(parents=True, exist_ok=True)

    plat_name = f"{proj_name}_plat"
    a53_dom_name = "standalone_psu_cortexa53_0"
    r5f_dom_name = "standalone_psu_cortexr5_0"

    print(f"=== Vitis 2024.2 Build: {proj_name} ===")
    print(f"  XSA       : {xsa_path}")
    print(f"  Workspace : {ws_dir}")

    client = vitis.create_client()
    try:
        client.set_workspace(path=str(ws_dir))

        platform = client.create_platform_component(
            name=plat_name,
            hw_design=str(xsa_path),
            os="standalone",
            cpu="psu_cortexa53_0",
            domain_name=a53_dom_name,
        )

        # Enable lwIP 2.2.0 on Cortex-A53 for GbE UDP telemetry bridge
        a53_domain = platform.get_domain(name=a53_dom_name)
        a53_domain.set_lib(lib_name="lwip220")

        # Add Cortex-R5F standalone domain for deterministic 1 kHz supervisor
        platform.add_domain(
            cpu="psu_cortexr5_0",
            os="standalone",
            name=r5f_dom_name,
        )

        print("Building platform (BSPs + FSBL + PMUFW)...")
        platform.build()

        xpfm_path = ws_dir / plat_name / "export" / plat_name / f"{plat_name}.xpfm"

        # Cortex-A53 UDP-DMA Bridge
        print("Building a53_net_bridge...")
        app_a53 = client.create_app_component(
            name="a53_net_bridge",
            platform=str(xpfm_path),
            domain=a53_dom_name,
        )
        app_a53.import_files(
            from_loc=str(repo_root / "sw" / "a53_net_bridge" / "src"),
            files=["main.c"],
            dest_dir_in_cmp="src",
        )
        app_a53.build()

        # Cortex-R5F 1 kHz Real-Time Supervisor
        print("Building r5f_realtime...")
        app_r5f = client.create_app_component(
            name="r5f_realtime",
            platform=str(xpfm_path),
            domain=r5f_dom_name,
        )
        app_r5f.import_files(
            from_loc=str(repo_root / "sw" / "r5f_realtime" / "src"),
            files=["main.c"],
            dest_dir_in_cmp="src",
        )
        app_r5f.build()

        a53_elf = ws_dir / "a53_net_bridge" / "build" / "a53_net_bridge.elf"
        r5f_elf = ws_dir / "r5f_realtime" / "build" / "r5f_realtime.elf"
        if not a53_elf.is_file() or not r5f_elf.is_file():
            raise RuntimeError(f"Missing ELF output! A53={a53_elf.is_file()}, R5F={r5f_elf.is_file()}")
        print(f"SUCCESS:\n  A53 ELF: {a53_elf}\n  R5F ELF: {r5f_elf}")
    finally:
        vitis.dispose()


if __name__ == "__main__":
    main()
