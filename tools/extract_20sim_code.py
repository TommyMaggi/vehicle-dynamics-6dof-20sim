#!/usr/bin/env python3
"""
Extract the equation code of every submodel from a 20-sim model (.emx).

A 20-sim .emx file is XML; the model itself is stored as SIDOPS source in a
<Sidops><![CDATA[ ... ]]></Sidops> element. This script walks the SIDOPS
hierarchy (graphical "bg" submodels that contain equation "eq" submodels) and
writes one readable file per piece of equation code:

  <out>/<Block>.sidops            model-specific equation blocks
  <out>/<group>/<Block>.sidops    blocks shared by several identical submodels
                                  (e.g. the four wheels -> wheel/)
  <out>/library/<Block>.sidops    standard bond-graph elements
                                  (junctions, sensors, sources) kept for reference
  <out>/INDEX.md                  model tree with types and links

Identical blocks (same type + same code) are written once and list every
instance that uses them.

Usage:
    python tools/extract_20sim_code.py model/vehicle_6dof.emx code/ [--rename OLD=NEW ...]

Example:
    python tools/extract_20sim_code.py model/vehicle_6dof.emx code/ --rename Constant=ROAD_SURFACE
"""
import argparse
import html
import os
import re
import sys
from collections import OrderedDict

HEADER_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s+-?\d+(?:\.\d+)?\s+-?\d+(?:\.\d+)?\s*$")
TYPE_RE = re.compile(r"^\s*type\s+'?([^';]+)'?\s*$")
LIB_RE = re.compile(r"<LibraryPath>([^<]*)</LibraryPath>")
STANDARD_TYPES = {"OneJunction", "ZeroJunction", "EffortSensor", "FlowSensor", "MSe", "MSf", "Se", "Sf"}


def load_sidops(path):
    with open(path, encoding="utf-8") as f:
        xml = f.read()
    m = re.search(r"<Sidops><!\[CDATA\[(.*?)\]\]></Sidops>", xml, re.S)
    if not m:
        sys.exit("No <Sidops> section found: is this a 20-sim .emx file?")
    return m.group(1).replace("\r\n", "\n")


def parse(sidops):
    """Return the list of submodels in file order (dicts)."""
    lines = sidops.split("\n")
    blocks, stack, pending = [], [], None
    i, n = 0, len(lines)

    def new(name):
        return {"name": name, "type": None, "library": "", "ports": []}

    while i < n:
        line, s = lines[i], lines[i].strip()
        hm = HEADER_RE.match(line)
        if s.startswith("model ") and not stack and pending is None:
            pending = new("MAIN")
        elif hm:
            pending = new(hm.group(1))
        elif pending is not None:
            if s.startswith("type ") and pending["type"] is None:
                tm = TYPE_RE.match(line)
                if tm:
                    pending["type"] = tm.group(1).strip()
            elif "<LibraryPath>" in line and not pending["library"]:
                lm = LIB_RE.search(line)
                if lm:
                    pending["library"] = html.unescape(lm.group(1))
            elif s == "ports":
                i += 1
                while i < n and lines[i].strip() != "end;":
                    if lines[i].strip():
                        pending["ports"].append(lines[i].strip().rstrip(";"))
                    i += 1

        if s == "implementation bg":
            node = pending or new("?")
            node.update(path="/".join([b["name"] for b in stack] + [node["name"]]), impl="bg")
            blocks.append(node)
            stack.append(node)
            pending = None
        elif s == "implementation eq":
            node = pending or new("?")
            body = []
            i += 1
            while i < n:
                if "implementation_end;" in lines[i]:
                    before = lines[i].split("implementation_end;")[0]
                    if before.strip():
                        body.append(before)
                    break
                body.append(lines[i])
                i += 1
            if node["type"] is None and node["library"]:
                node["type"] = os.path.splitext(node["library"].replace("\\", "/").split("/")[-1])[0]
            node.update(path="/".join([b["name"] for b in stack] + [node["name"]]), impl="eq",
                        body="\n".join(body).strip("\n"))
            blocks.append(node)
            pending = None
        elif "implementation_end;" in line and stack:
            stack.pop()
        i += 1
    return blocks


def common_group(parents):
    names = [p.split("/")[-1] for p in parents]
    prefix = os.path.commonprefix(names).rstrip("_")
    return (prefix or "shared").lower()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("emx")
    ap.add_argument("out")
    ap.add_argument("--rename", action="append", default=[], metavar="OLD=NEW",
                    help="display/file name for a top-level submodel")
    args = ap.parse_args()
    rename = dict(r.split("=", 1) for r in args.rename)

    blocks = parse(load_sidops(args.emx))
    for b in blocks:
        parts = b["path"].split("/")
        parts = [rename.get(p, p) if k == 1 else p for k, p in enumerate(parts)]
        b["path"] = "/".join(parts[1:]) if parts[0] == "MAIN" else "/".join(parts)
        b["name"] = parts[-1]

    groups = OrderedDict()
    for b in blocks:
        if b["impl"] == "eq":
            groups.setdefault((b["type"] or b["name"], b["body"]), []).append(b)

    os.makedirs(args.out, exist_ok=True)
    file_of, used = {}, set()
    for (btype, body), members in groups.items():
        first = members[0]
        parents = sorted({os.path.dirname(m["path"]) for m in members})
        if btype in STANDARD_TYPES:
            folder = "library"
        elif len(parents) > 1:
            folder = common_group(parents)
        else:
            folder = parents[0]
        rel = os.path.join(folder, first["name"] + ".sidops") if folder else first["name"] + ".sidops"
        k = 2
        while rel in used:
            base = f"{first['name']}_{k}.sidops"
            rel = os.path.join(folder, base) if folder else base
            k += 1
        used.add(rel)
        dst = os.path.join(args.out, rel)
        os.makedirs(os.path.dirname(dst) or args.out, exist_ok=True)
        with open(dst, "w", encoding="utf-8", newline="\n") as f:
            f.write(f"// Extracted from {os.path.basename(args.emx)} by tools/extract_20sim_code.py\n")
            f.write(f"// Type:    {btype}\n")
            if first["library"]:
                f.write(f"// Library: {first['library']}\n")
            f.write("// Used by: " + ", ".join(m["path"] for m in members) + "\n")
            if first["ports"]:
                f.write("// Ports:\n")
                for p in first["ports"]:
                    f.write(f"//   {p}\n")
            f.write("\n" + body.rstrip() + "\n")
        for m in members:
            file_of[m["path"]] = rel.replace(os.sep, "/")

    # INDEX.md: collapse graphical submodels whose content is identical to an earlier one
    signature, seen_sig = {}, {}
    for b in blocks:
        if b["impl"] == "bg" and b["path"]:
            kids = [c for c in blocks if c["path"].startswith(b["path"] + "/")]
            signature[b["path"]] = tuple(sorted(file_of.get(c["path"], "") for c in kids))

    with open(os.path.join(args.out, "INDEX.md"), "w", encoding="utf-8") as f:
        f.write(f"# Model structure: `{os.path.basename(args.emx)}`\n\n")
        f.write("Generated by [`tools/extract_20sim_code.py`](../tools/extract_20sim_code.py). "
                "Bold entries are graphical (bond graph) submodels, the others are equation blocks.\n\n")
        skip_prefix = None
        for b in blocks:
            if not b["path"]:
                continue
            if skip_prefix and b["path"].startswith(skip_prefix + "/"):
                continue
            skip_prefix = None
            indent = "  " * b["path"].count("/")
            t = f" `{b['type']}`" if b["type"] else ""
            if b["impl"] == "bg":
                sig = signature.get(b["path"])
                if sig in seen_sig:
                    f.write(f"{indent}- **{b['name']}**{t}: same structure and code as **{seen_sig[sig]}**\n")
                    skip_prefix = b["path"]
                    continue
                seen_sig[sig] = b["name"]
                f.write(f"{indent}- **{b['name']}**{t}\n")
            else:
                f.write(f"{indent}- {b['name']}{t} → [`{file_of[b['path']]}`]({file_of[b['path']]})\n")

    n_eq = sum(1 for b in blocks if b["impl"] == "eq")
    print(f"{n_eq} equation submodels -> {len(groups)} unique code files in {args.out}")


if __name__ == "__main__":
    main()
