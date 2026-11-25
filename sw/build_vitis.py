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

        # Patch lwip220 xemacpsif_physpeed.c for Avnet ZUBoard 1CG Microchip KSZ9131RNX PHY (ID 0x0022)
        physpeed_c = (
            ws_dir / plat_name / "psu_cortexa53_0" / a53_dom_name / "bsp" / "libsrc" /
            "lwip220" / "src" / "lwip-2.2.0" / "contrib" / "ports" / "xilinx" / "netif" /
            "xemacpsif_physpeed.c"
        )
        if physpeed_c.is_file():
            src = physpeed_c.read_text()
            if "PHY_MICREL_IDENTIFIER" not in src:
                print("Patching lwip220 for ZUBoard 1CG Microchip KSZ9131 PHY (0x0022)...")
                src = src.replace(
                    "#define PHY_REALTEK_IDENTIFIER\t\t\t\t0x001c",
                    "#include \"sleep.h\"\n#define PHY_REALTEK_IDENTIFIER\t\t\t\t0x001c\n#define PHY_MICREL_IDENTIFIER\t\t\t\t0x0022",
                )
                src = src.replace(
                    "(phy_reg != PHY_ADI_IDENTIFIER)) {",
                    "(phy_reg != PHY_ADI_IDENTIFIER) &&\n\t\t    (phy_reg != PHY_MICREL_IDENTIFIER)) {",
                )
                micrel_fn = """static u16_t ksz9131_mmd_read(XEmacPs *xemacpsp, u32_t phy_addr, u16_t dev, u16_t reg)
{
\tu16_t val = 0;
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, 0x0D, dev);
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, 0x0E, reg);
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, 0x0D, (u16_t)(0x4000U | dev));
\tXEmacPs_PhyRead(xemacpsp, phy_addr, 0x0E, &val);
\treturn val;
}

static void ksz9131_mmd_write(XEmacPs *xemacpsp, u32_t phy_addr, u16_t dev, u16_t reg, u16_t val)
{
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, 0x0D, dev);
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, 0x0E, reg);
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, 0x0D, (u16_t)(0x4000U | dev));
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, 0x0E, val);
}

static u32_t get_Micrel_phy_speed(XEmacPs *xemacpsp, u32_t phy_addr)
{
\tu16_t ctrl = 0, status = 0, adv = 0, lpa = 0, gctrl = 0, gstat = 0;
\tu16_t rxc_dll = 0, txc_dll = 0;
\tu32_t timeout = 0;

\t/* Enable RGMII-ID 2 ns DLL delays on both RXC (MMD2 0x4C) and TXC (MMD2 0x4D) */
\trxc_dll = ksz9131_mmd_read(xemacpsp, phy_addr, 2, 0x4C);
\ttxc_dll = ksz9131_mmd_read(xemacpsp, phy_addr, 2, 0x4D);
\tksz9131_mmd_write(xemacpsp, phy_addr, 2, 0x4C, (u16_t)(rxc_dll & ~0x1000U));
\tksz9131_mmd_write(xemacpsp, phy_addr, 2, 0x4D, (u16_t)(txc_dll & ~0x1000U));

\txil_printf("KSZ9131 PHY @ addr %u: RGMII-ID DLL enabled, starting autonegotiation...\\r\\n", phy_addr);
\tXEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, &adv);
\tadv |= 0x01E0; /* 10/100 FD/HD */
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, adv);

\tXEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_1000_ADVERTISE_REG_OFFSET, &gctrl);
\tgctrl |= 0x0300; /* 1000BASE-T FD/HD */
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_1000_ADVERTISE_REG_OFFSET, gctrl);

\tXEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &ctrl);
\tctrl |= (IEEE_CTRL_AUTONEGOTIATE_ENABLE | IEEE_STAT_AUTONEGOTIATE_RESTART);
\tXEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, ctrl);

\twhile (timeout < 200) {
\t\tXEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);
\t\tXEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);
\t\tif ((status & IEEE_STAT_AUTONEGOTIATE_COMPLETE) && (status & 0x0004)) {
\t\t\tbreak;
\t\t}
\t\tusleep(50000);
\t\ttimeout++;
\t}
\tif (!(status & 0x0004)) {
\t\txil_printf("KSZ9131 link timeout (BMSR=0x%04X)\\r\\n", status);
\t\treturn XST_FAILURE;
\t}

\tXEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_1000_ADVERTISE_REG_OFFSET, &gctrl);
\tXEmacPs_PhyRead(xemacpsp, phy_addr, 0x0A, &gstat);
\tif (((gctrl & 0x0200) && (gstat & 0x0800)) || ((gctrl & 0x0100) && (gstat & 0x0400))) {
\t\txil_printf("KSZ9131 link speed: 1000 Mbps\\r\\n");
\t\treturn 1000;
\t}
\tXEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, &adv);
\tXEmacPs_PhyRead(xemacpsp, phy_addr, 0x05, &lpa);
\tif ((adv & lpa) & 0x0180) {
\t\txil_printf("KSZ9131 link speed: 100 Mbps\\r\\n");
\t\treturn 100;
\t}
\txil_printf("KSZ9131 link speed: 10 Mbps\\r\\n");
\treturn 10;
}

static u32_t get_IEEE_phy_speed(XEmacPs *xemacpsp, u32_t phy_addr)
{"""
                src = src.replace(
                    "static u32_t get_IEEE_phy_speed(XEmacPs *xemacpsp, u32_t phy_addr)\n{",
                    micrel_fn,
                    1,
                )
                src = src.replace(
                    "\t} else if (phy_identity == PHY_ADI_IDENTIFIER) {\n\t\tRetStatus = get_Adi_phy_speed(xemacpsp, phy_addr);",
                    "\t} else if (phy_identity == PHY_ADI_IDENTIFIER) {\n\t\tRetStatus = get_Adi_phy_speed(xemacpsp, phy_addr);\n\t} else if (phy_identity == PHY_MICREL_IDENTIFIER) {\n\t\tRetStatus = get_Micrel_phy_speed(xemacpsp, phy_addr);",
                    1,
                )
                if (
                    src.count("static u32_t get_Micrel_phy_speed(") != 1
                    or "RetStatus = get_Micrel_phy_speed(xemacpsp, phy_addr);" not in src
                ):
                    raise RuntimeError("Failed to patch xemacpsif_physpeed.c for KSZ9131 PHY")
                physpeed_c.write_text(src)
                print("Rebuilding platform BSP with KSZ9131 PHY support...")
                platform.build()

        xpfm_path = ws_dir / plat_name / "export" / plat_name / f"{plat_name}.xpfm"
        if not xpfm_path.is_file():
            raise RuntimeError(f"Platform build failed: missing {xpfm_path}")

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
        # Partition 1 GB LPDDR4: A53 owns 0x0000_0000..0x2FFF_FFFF (768 MB)
        a53_ld = ws_dir / "a53_net_bridge" / "src" / "lscript.ld"
        if a53_ld.is_file():
            txt = a53_ld.read_text()
            txt = txt.replace(
                "psu_ddr_0 : ORIGIN = 0x0, LENGTH = 0x3ff00000",
                "psu_ddr_0 : ORIGIN = 0x0, LENGTH = 0x30000000",
            )
            a53_ld.write_text(txt)
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
        # Partition 1 GB LPDDR4: R5F owns 0x3000_0000..0x3FEF_FFFF (255 MB, zero overlap with A53!)
        r5f_ld = ws_dir / "r5f_realtime" / "src" / "lscript.ld"
        if r5f_ld.is_file():
            txt = r5f_ld.read_text()
            txt = txt.replace(
                "psu_ddr_0 : ORIGIN = 0x100000, LENGTH = 0x3fefffff",
                "psu_ddr_0 : ORIGIN = 0x30000000, LENGTH = 0x0ff00000",
            )
            r5f_ld.write_text(txt)
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
