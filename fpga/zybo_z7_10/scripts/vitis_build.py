# SenseEdge on the Zybo Z7-10: scripted Vitis (unified, 2023.2+) build of the bare-metal program.
#   cd fpga/zybo_z7_10 && vitis -s scripts/vitis_build.py [build/senseedge_zybo.xsa]
# Creates build/vitis_ws/ with a standalone platform for ps7_cortexa9_0 from the .xsa and the
# application component senseedge_app from sw/senseedge_zybo.c, then copies out
#   build/senseedge_zybo.elf   the program          build/ps7_init.tcl   PS initialisation (from the .xsa)
# Run them with:  xsdb scripts/program.tcl build/senseedge_zybo.bit build/ps7_init.tcl build/senseedge_zybo.elf
import os
import shutil
import sys
import zipfile

import vitis

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(here, ".."))
build = os.path.join(root, "build")
xsa = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(build, "senseedge_zybo.xsa")
if not os.path.isfile(xsa):
    sys.exit(f"missing {xsa}; run scripts/build.tcl in Vivado first")
# The workspace must sit at a SHORT path: the BSP's CMake object paths exceed Windows' 250-character
# limit when the workspace is under a deep folder (e.g. OneDrive\Desktop\...), and the build fails
# with "cannot open dependency file".  Override with SE_VITIS_WS.
ws = os.environ.get("SE_VITIS_WS") or (os.path.join(os.path.expanduser("~"), "se_ws") if os.name == "nt" else os.path.join(build, "vitis_ws"))
pf_name, app_name, domain = "senseedge_pf", "senseedge_app", "standalone_ps7_cortexa9_0"

# Stage the sources so the program's include of ../../../firmware/nn_weights.h becomes local.
stage = os.path.join(build, "vitis_src")
os.makedirs(stage, exist_ok=True)
src = open(os.path.join(root, "sw", "senseedge_zybo.c"), encoding="utf-8").read()
src = src.replace('#include "../../../firmware/nn_weights.h"', '#include "nn_weights.h"')
open(os.path.join(stage, "senseedge_zybo.c"), "w", encoding="utf-8").write(src)
shutil.copy(os.path.join(root, "..", "..", "firmware", "nn_weights.h"), os.path.join(stage, "nn_weights.h"))

# ps7_init.tcl straight out of the .xsa (it is a zip)
with zipfile.ZipFile(xsa) as z:
    names = [n for n in z.namelist() if n.endswith("ps7_init.tcl")]
    if not names:
        sys.exit("no ps7_init.tcl inside the .xsa")
    open(os.path.join(build, "ps7_init.tcl"), "wb").write(z.read(names[0]))
print(f"ps7_init.tcl -> {os.path.join(build, 'ps7_init.tcl')}")

def _rm_readonly(func, path, _exc):
    """The exported platform contains read-only files; clear the bit and retry."""
    import stat
    os.chmod(path, stat.S_IWRITE)
    func(path)


if os.path.isdir(ws):
    try:
        shutil.rmtree(ws, onexc=_rm_readonly)
    except Exception as e:  # a stale vitis-server may still hold _ide/.wsdata/.lock: use a fresh workspace
        print(f"could not clear {ws} ({e}); using a fresh workspace")
        k = 1
        while os.path.isdir(f"{ws}{k}"):
            k += 1
        ws = f"{ws}{k}"
print(f"workspace: {ws}")
client = vitis.create_client()
client.set_workspace(path=ws)
platform = client.create_platform_component(name=pf_name, hw_design=xsa, os="standalone",
                                            cpu="ps7_cortexa9_0", domain_name=domain)
platform.report()
platform.build()
xpfm = client.find_platform_in_repos(pf_name)
print(f"platform: {xpfm}")

app = client.create_app_component(name=app_name, platform=xpfm, domain=domain, template="empty_application")
app.import_files(from_loc=stage, files=["senseedge_zybo.c", "nn_weights.h"], dest_dir_in_cmp="src")
app.set_app_config(key="USER_COMPILE_OPTIMIZATION_LEVEL", values="-O2")
# SE_BAUD=<rate> in the environment compiles the program for a faster UART (host/README.md: 921600
# cuts a dataset replay's transfer time by 8); the ELF is then named senseedge_zybo_<rate>.elf.
baud = os.environ.get("SE_BAUD")
if baud:
    app.set_app_config(key="USER_COMPILE_DEFINITIONS", values=[f"SE_BAUD={int(baud)}"])
app.build()

elf = None
for dp, _, fs in os.walk(os.path.join(ws, app_name)):
    for f in fs:
        if f.endswith(".elf"):
            elf = os.path.join(dp, f)
if not elf:
    sys.exit("build produced no .elf")
out = os.path.join(build, f"senseedge_zybo_{int(baud)}.elf" if baud else "senseedge_zybo.elf")
shutil.copy(elf, out)
print(f"DONE: {out}")
vitis.dispose()
