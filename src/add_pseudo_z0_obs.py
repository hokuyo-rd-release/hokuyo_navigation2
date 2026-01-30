#!/usr/bin/env python3
# sample script that adds pseudo z=0 observations

import sys

def parse_vertex_se3_quat(line: str):
    parts = line.strip().split()
    assert(parts[0] == "VERTEX_SE3:QUAT")

    try:
        node_id = int(parts[1])
        x = float(parts[2])
        y = float(parts[3])
        z = float(parts[4])
    except ValueError:
        return None

    return node_id, x, y, z

def main():
    if len(sys.argv) != 2:
        sys.exit(1)

    p2o_lines = []
    z0_edges = []
    path = sys.argv[1]
    with open(path) as f:
        for line in f:
            p2o_lines.append(line)
            if not line.startswith("VERTEX_SE3:QUAT"):
                continue
            parsed = parse_vertex_se3_quat(line)
            if parsed is None:
                continue
            node_id, x, y, z = parsed
            if node_id != 0:
                z0_edges.append(f"EDGE_LIN3D 0 {node_id} {x} {y} 0 1e-6 0 0 1e-6 0 1e-2")
    p2o_lines.extend(z0_edges)
    for l in p2o_lines:
        print(l)

if __name__ == "__main__":
    main()

