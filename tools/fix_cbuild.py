# -*- coding: utf-8 -*-
"""
CubeMX 重新生成代码后，修复 VS Code / CMSIS(cbuild) 构建链的三处常见故障。

用法：
    python tools/fix_cbuild.py
    python tools/fix_cbuild.py PID_Test/MDK-ARM/PID_Test.uvprojx

依次做五件事：
  1. 给 .uvprojx 补 <uAC6>1</uAC6>                    治 "configured to use Arm Compiler 5"
  2. 给 .uvprojx 补 <PackID>Keil.STM32H7xx_DFP...</PackID>   治转换后器件包丢失
  3. 修 OnChipMemories 全为 0x0                      治 "scatter file ... ParseInt"
  4. 调 uv2csolution 生成 .csolution.yml              转换失败会打印原因
  5. 校准 .cproject.yml 的包版本                      治 "required pack ARM::CMSIS@^4.5.0"
"""
import io
import os
import re
import sys
import glob
import subprocess

OCM_BLOCK = '<OnChipMemories>\n              <Ocm1>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </Ocm1>\n              <Ocm2>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </Ocm2>\n              <Ocm3>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </Ocm3>\n              <Ocm4>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </Ocm4>\n              <Ocm5>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </Ocm5>\n              <Ocm6>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </Ocm6>\n              <IRAM>\n                <Type>0</Type>\n                <StartAddress>0x20000000</StartAddress>\n                <Size>0x20000</Size>\n              </IRAM>\n              <IROM>\n                <Type>1</Type>\n                <StartAddress>0x8000000</StartAddress>\n                <Size>0x100000</Size>\n              </IROM>\n              <XRAM>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </XRAM>\n              <OCR_RVCT1>\n                <Type>1</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </OCR_RVCT1>\n              <OCR_RVCT2>\n                <Type>1</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </OCR_RVCT2>\n              <OCR_RVCT3>\n                <Type>1</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </OCR_RVCT3>\n              <OCR_RVCT4>\n                <Type>1</Type>\n                <StartAddress>0x8000000</StartAddress>\n                <Size>0x100000</Size>\n              </OCR_RVCT4>\n              <OCR_RVCT5>\n                <Type>1</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </OCR_RVCT5>\n              <OCR_RVCT6>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </OCR_RVCT6>\n              <OCR_RVCT7>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </OCR_RVCT7>\n              <OCR_RVCT8>\n                <Type>0</Type>\n                <StartAddress>0x0</StartAddress>\n                <Size>0x0</Size>\n              </OCR_RVCT8>\n              <OCR_RVCT9>\n                <Type>0</Type>\n                <StartAddress>0x20000000</StartAddress>\n                <Size>0x20000</Size>\n              </OCR_RVCT9>\n              <OCR_RVCT10>\n                <Type>0</Type>\n                <StartAddress>0x24000000</StartAddress>\n                <Size>0x50000</Size>\n              </OCR_RVCT10>\n            </OnChipMemories>'


def find_projects(root):
    hits = glob.glob(os.path.join(root, "**", "*.uvprojx"), recursive=True)
    return [h for h in hits if "PID_Test" in os.path.basename(h)]


def patch_uvprojx(path):
    s = io.open(path, encoding="utf-8").read()
    done = []

    if "<uAC6>" not in s:
        marker = "      <ToolsetName>ARM-ADS</ToolsetName>\n"
        if marker in s:
            s = s.replace(marker, marker + "      <uAC6>1</uAC6>\n", 1)
            done.append("uAC6=1")

    if "<PackID>" not in s:
        marker = "          <Vendor>STMicroelectronics</Vendor>\n"
        if marker in s:
            s = s.replace(marker,
                          marker
                          + "          <PackID>Keil.STM32H7xx_DFP.4.1.3</PackID>\n"
                          + "          <PackURL>https://www.keil.com/pack/</PackURL>\n",
                          1)
            done.append("PackID")

    m = re.search(r"<OnChipMemories>.*?</OnChipMemories>", s, re.S)
    if m:
        iram = re.search(r"<IRAM>.*?</IRAM>", m.group(0), re.S)
        if iram and "<Size>0x0</Size>" in iram.group(0):
            s = s[:m.start()] + OCM_BLOCK + s[m.end():]
            done.append("memory map")

    if done:
        io.open(path, "w", encoding="utf-8", newline="").write(s)
    return done


def find_uv2csolution():
    pattern = os.path.expanduser(
        "~/.vcpkg/artifacts/*/tools.arm.mdk.toolbox/*/bin/uv2csolution.exe")
    hits = sorted(glob.glob(pattern))
    return hits[-1] if hits else None


def convert(uvdir):
    exe = find_uv2csolution()
    if not exe:
        print("[!] 找不到 uv2csolution.exe，请在 VS Code 里手动执行 Convert")
        return False
    uv = glob.glob(os.path.join(uvdir, "PID_Test.uvprojx"))
    if not uv:
        print("[!] 目录里没有 PID_Test.uvprojx")
        return False
    r = subprocess.run([exe, os.path.basename(uv[0])], cwd=uvdir,
                       capture_output=True, text=True, errors="ignore")
    out = ((r.stdout or "") + (r.stderr or "")).strip()
    print(out)
    return "converted" in out and "0 out of" not in out


def patch_packs(cproject):
    if not os.path.exists(cproject):
        return []
    s = io.open(cproject, encoding="utf-8").read()
    done = []
    if "Keil::STM32H7xx_DFP" not in s:
        s = re.sub(r"(\n    - pack: ARM::CMSIS@[^\n]*)",
                   r"\1\n    - pack: Keil::STM32H7xx_DFP@^4.1.3", s, count=1)
        done.append("STM32H7xx_DFP")
    if "ARM::CMSIS@^4.5.0" in s:
        s = s.replace("ARM::CMSIS@^4.5.0", "ARM::CMSIS@^5.9.0")
        done.append("CMSIS 5.9")
    if done:
        io.open(cproject, "w", encoding="utf-8", newline="").write(s)
    return done


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if len(sys.argv) > 1:
        uvprojx = os.path.abspath(sys.argv[1])
    else:
        hits = find_projects(root)
        if not hits:
            print("[!] 没找到 PID_Test.uvprojx")
            return 1
        uvprojx = hits[0]

    uvdir = os.path.dirname(uvprojx)
    print("工程:", uvprojx)

    print("[1] uvprojx 修补:", patch_uvprojx(uvprojx) or "无需修改")

    ok = convert(uvdir)
    print("[2] 转换:", "成功" if ok else "失败")

    if ok:
        name = os.path.splitext(os.path.basename(uvprojx))[0]
        packs = patch_packs(os.path.join(uvdir, name + ".cproject.yml"))
        print("[3] 器件包修补:", packs or "无需修改")
        print("\n完成，回 VS Code 点小锤子即可。")
    else:
        print("\n转换失败，按上面的报错处理后重跑本脚本。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
