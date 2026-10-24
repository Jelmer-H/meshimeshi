#!/usr/bin/env python3
"""Convert a Meshimeshi node-info JSON export to a nodes CSV for meshsim.

Input: JSON list of node entries, e.g.
    [{"name":"naam","public_key":"8EFF...","short_id":"8E",
      "location":{"latitude":52.32576,"longitude":6.95849},
      "last_seen":"2026-10-23T08:52:33.109Z","node_weight":250.474,"sf":7}]

Entries with sf != 7 are dropped. The hoogte_m column is left empty and the
node_weight value is written as an extra column.

Output format: id,lat,lon,hoogte_m,naam,node_weight
"""

import argparse
import csv
import json
import sys
from binascii import crc_hqx


def convert(in_path, out_path):
    with open(in_path, "r", encoding="utf-8") as f:
        nodes = json.load(f)

    rows = []
    skipped = 0
    seen_ids = {}
    for node in nodes:
        if node.get("sf") != 7:
            skipped += 1
            continue
        loc = node.get("location") or {}
        pubkey = node.get("public_key", "")
        # Deterministic 16-bit node id from the full public key (design.md:
        # node ID is 2 bytes). Collisions are reported as a warning.
        nid = "0x%04X" % (crc_hqx(pubkey.encode("utf-8"), 0xFFFF) & 0xFFFF)
        if nid in seen_ids:
            print("WARNING: id collision: %s for %s and %s" %
                  (nid, seen_ids[nid], node.get("name", "?")), file=sys.stderr)
        else:
            seen_ids[nid] = node.get("name", "?")
        rows.append([
            nid,
            loc.get("latitude", ""),
            loc.get("longitude", ""),
            "",  # hoogte_m: not available in the JSON export
            "NODE#%04d" % (len(rows) + 1),
            node.get("node_weight", ""),
        ])

    with open(out_path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "lat", "lon", "hoogte_m", "naam", "node_weight"])
        w.writerows(rows)

    print(f"Wrote {len(rows)} nodes to {out_path} ({skipped} skipped, sf != 7)")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", help="input JSON file")
    p.add_argument("output", help="output CSV file")
    args = p.parse_args()
    convert(args.input, args.output)


if __name__ == "__main__":
    main()
