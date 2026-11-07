#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""CMSIS-SVD to athrill-device map registers, and map-vs-SVD comparison.

The SVD files are inputs fetched by tools/fetch_svd.sh at the revisions pinned in
spec/svd-sources.yaml; they are not committed here.

  svd_map.py extract --svd STM32H743.svd --peripheral SPI1
        print the registers of one peripheral in the map's `registers` format (YAML)
  svd_map.py check --svd STM32H743.svd --peripheral SPI1 --map spec/maps/st-stm32-spi-v2.yaml
        compare the map's registers with the SVD: offsets, field positions and widths,
        reset values, access; exit 1 on a mismatch
  svd_map.py check-all
        run `check` for every (map, soc, peripheral) listed in spec/svd-sources.yaml
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent


def num(text: str | None, default: int = 0) -> int:
    if text is None:
        return default
    t = text.strip().lower()
    if t.startswith("#"):
        return int(t[1:].replace("x", "0"), 2)
    if t.isdigit():
        return int(t, 10)
    return int(t, 0)


class Svd:
    def __init__(self, path: Path):
        self.root = ET.parse(path).getroot()
        self.periph = {p.findtext("name"): p for p in self.root.iter("peripheral")}
        self.default_size = num(self.root.findtext("size"), 32)
        self.default_reset = num(self.root.findtext("resetValue"), 0)
        self.default_access = self.root.findtext("access") or "read-write"

    def resolve(self, name: str) -> ET.Element:
        p = self.periph[name]
        base = p.get("derivedFrom")
        if base and p.find("registers") is None:
            return self.resolve(base)
        return p

    def registers(self, name: str) -> dict[str, dict]:
        p = self.resolve(name)
        out: dict[str, dict] = {}

        def walk(node: ET.Element, offset: int, prefix: str, inherit: dict):
            for child in node:
                if child.tag == "cluster":
                    dim = num(child.findtext("dim"), 1)
                    inc = num(child.findtext("dimIncrement"), 0)
                    for i in range(dim):
                        cname = (child.findtext("name") or "").replace("[%s]", str(i)).replace("%s", str(i))
                        walk(child, offset + num(child.findtext("addressOffset")) + i * inc, prefix + cname + ".", inherit)
                elif child.tag == "register":
                    dim = num(child.findtext("dim"), 1)
                    inc = num(child.findtext("dimIncrement"), 0)
                    for i in range(dim):
                        rname = (child.findtext("name") or "").replace("[%s]", str(i)).replace("%s", str(i))
                        reg = {
                            "offset": offset + num(child.findtext("addressOffset")) + i * inc,
                            "size": num(child.findtext("size"), inherit["size"]),
                            "reset": num(child.findtext("resetValue"), inherit["reset"]),
                            "access": child.findtext("access") or inherit["access"],
                            "fields": {},
                        }
                        fields = child.find("fields")
                        if fields is not None:
                            for f in fields.findall("field"):
                                lsb, width = field_pos(f)
                                reg["fields"][f.findtext("name")] = {
                                    "lsb": lsb, "width": width,
                                    "access": f.findtext("access") or reg["access"],
                                }
                        out[prefix + rname] = reg

        regs = p.find("registers")
        inherit = {"size": self.default_size, "reset": self.default_reset, "access": self.default_access}
        if regs is not None:
            walk(regs, 0, "", inherit)
        return out


def field_pos(f: ET.Element) -> tuple[int, int]:
    if f.find("bitOffset") is not None:
        return num(f.findtext("bitOffset")), num(f.findtext("bitWidth"), 1)
    if f.find("lsb") is not None:
        lsb, msb = num(f.findtext("lsb")), num(f.findtext("msb"))
        return lsb, msb - lsb + 1
    rng = f.findtext("bitRange").strip("[]").split(":")
    msb, lsb = int(rng[0]), int(rng[1])
    return lsb, msb - lsb + 1


def map_registers(m: dict, block_names: dict[str, str] | None = None) -> dict[str, dict]:
    """Flatten the map's registers; repeated blocks (stream, channel) are expanded
    with the SVD naming pattern given in block_names, e.g. {"stream": "S{i}{r}"}."""
    regs = {k: v for k, v in (m.get("registers") or {}).items() if isinstance(v, dict) and "offset" in v}
    for key in ("stream", "channel"):
        block = m.get(key) or (m.get("registers") or {}).get(key)
        if not (isinstance(block, dict) and "registers" in block):
            continue
        pattern = (block_names or {}).get(key, key[0].upper() + "{i}{r}")
        base, stride = int(str(block["base"]), 0), int(str(block["stride"]), 0)
        for i in range(int(block["count"])):
            for rname, r in block["registers"].items():
                rr = dict(r)
                rr["offset"] = hex(base + i * stride + int(str(r["offset"]), 0))
                regs[pattern.format(i=i, r=rname)] = rr
    return regs


def check(svd: Svd, peripheral: str, map_path: Path, aliases: dict[str, str], block_names: dict[str, str] | None = None):
    m = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    s = svd.registers(peripheral)
    problems: list[str] = []
    notes: list[str] = []
    for rname, r in map_registers(m, block_names).items():
        sname = aliases.get(rname, rname)
        if sname not in s:
            problems.append(f"register {rname}: not in SVD {peripheral}")
            continue
        sr = s[sname]
        reset = str(r.get("reset", "0"))
        if int(str(r["offset"]), 0) != sr["offset"]:
            problems.append(f"register {rname}: offset map {r['offset']} svd 0x{sr['offset']:02X}")
        if reset in ("per-instance", "disputed"):
            notes.append(f"register {rname}: reset {reset} (svd 0x{sr['reset']:08X})")
        elif int(reset, 0) != sr["reset"]:
            problems.append(f"register {rname}: reset map {reset} svd 0x{sr['reset']:08X}")
        for fname, fv in (r.get("fields") or {}).items():
            sf = sr["fields"].get(fname)
            if sf is None:
                notes.append(f"field {rname}.{fname}: not in SVD")
                continue
            if (fv["lsb"], fv["width"]) != (sf["lsb"], sf["width"]):
                problems.append(f"field {rname}.{fname}: map {fv['lsb']}:{fv['width']} svd {sf['lsb']}:{sf['width']}")
        for fname in sr["fields"]:
            if fname not in (r.get("fields") or {}):
                notes.append(f"field {rname}.{fname}: in SVD, not in map")
    return problems, notes


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("extract"); e.add_argument("--svd", required=True, type=Path); e.add_argument("--peripheral", required=True)
    c = sub.add_parser("check"); c.add_argument("--svd", required=True, type=Path); c.add_argument("--peripheral", required=True); c.add_argument("--map", required=True, type=Path)
    a = sub.add_parser("check-all"); a.add_argument("--svd-dir", type=Path, default=ROOT / "build" / "svd")
    args = ap.parse_args()
    if args.cmd == "extract":
        regs = Svd(args.svd).registers(args.peripheral)
        out = {n: {"offset": f"0x{r['offset']:02X}", "reset": f"0x{r['reset']:08X}", "access": r["access"],
                   "fields": {f: {"lsb": v["lsb"], "width": v["width"]} for f, v in r["fields"].items()}}
               for n, r in sorted(regs.items(), key=lambda kv: kv[1]["offset"])}
        print(yaml.safe_dump(out, sort_keys=False, width=120))
        return 0
    if args.cmd == "check":
        problems, notes = check(Svd(args.svd), args.peripheral, args.map, {})
        for n in notes:
            print("  note:", n)
        for p in problems:
            print("MISMATCH", p)
        return 1 if problems else 0
    src = yaml.safe_load((ROOT / "spec" / "svd-sources.yaml").read_text(encoding="utf-8"))
    cache: dict[str, Svd] = {}
    total = 0
    for chk in src["checks"]:
        svd_file = args.svd_dir / src["svd"][chk["soc"]]["file"]
        svd = cache.setdefault(chk["soc"], Svd(svd_file))
        problems, notes = check(svd, chk["peripheral"], ROOT / chk["map"], chk.get("aliases", {}), chk.get("blocks"))
        known = {(k["peripheral"], k["register"]) for k in src.get("known_svd_errors", []) if k["soc"] == chk["soc"]}
        problems = [p for p in problems if not any(p.startswith(f"register {reg}:") for per, reg in known if per == chk["peripheral"])]
        print(f"{chk['map']} vs {chk['soc']} {chk['peripheral']}: {len(problems)} mismatches, {len(notes)} notes")
        for p in problems:
            print("  MISMATCH", p)
        total += len(problems)
    return 1 if total else 0


if __name__ == "__main__":
    raise SystemExit(main())
