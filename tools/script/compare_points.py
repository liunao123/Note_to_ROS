#!/usr/bin/env python3
import math
import sys
from pathlib import Path
from statistics import mean, pstdev, pvariance

RTK = '/mnt/nvme0n1p2/nongan_rtk_points.pcd'
MANUAL = '/mnt/nvme0n1p2/na_manual_0.txt'

THRESH = 0.35  # 2d meters


def read_manual(path):
    pts = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            # if len(parts) < 4:
            #     continue
            # assume format: id,x,y,z
            try:
                # idx = parts[0]
                idx = 0
                x = float(parts[1 - 1])
                y = float(parts[2 - 1])
                z = float(parts[3 - 1])
                pts.append((idx, x, y, z))
            except Exception:
                continue
    return pts


def read_pcd(path):
    pts = []
    with open(path, 'r') as f:
        in_data = False
        for line in f:
            line = line.strip()
            if not line:
                continue
            if not in_data:
                if line.lower() == 'data ascii' or line.lower().startswith('data ascii'):
                    in_data = True
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                pts.append((x, y, z))
            except Exception:
                continue
    return pts


def euclid(a, b):
    # return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2  )


def main():
    rtk_pts = read_pcd(RTK)
    manual_pts = read_manual(MANUAL)
    print(f"Read {len(rtk_pts)} rtk points and {len(manual_pts)} manual points")

    # Build a simple matching: for each rtk point, find the nearest manual point within THRESH
    matches = []  # list of tuples (rtk_idx, manual_idx, rtk_pt, manual_pt, dx,dy,dz)
    used_manual = set()

    for i, r in enumerate(rtk_pts):
        best = None
        best_j = None
        best_d = None
        for j, m in enumerate(manual_pts):
            if j in used_manual:
                continue
            _, mx, my, mz = m
            d = euclid(r, (mx, my, mz))
            if d <= THRESH and (best_d is None or d < best_d):
                best = m
                best_j = j
                best_d = d
        if best is not None:
            _, mx, my, mz = best
            dx = r[0] - mx
            dy = r[1] - my
            dz = r[2] - mz
            matches.append((i, best_j, r, (mx, my, mz), dx, dy, dz))
            used_manual.add(best_j)

    print(f"Found {len(matches)} one-to-one matches within {THRESH} m")

    if not matches:
        print("No matches found. Exiting.")
        return

    dxs = [m[4] for m in matches]
    dys = [m[5] for m in matches]
    dzs = [m[6] for m in matches]
    dists = [euclid((dxs[i], dys[i], dzs[i]), (0,0,0)) for i in range(len(dxs))]

    def stats(arr):
        return {
            'count': len(arr),
            'mean': mean(arr),
            'variance_pop': pvariance(arr),
            'stddev_pop': pstdev(arr)
        }

    stats_x = stats(dxs)
    stats_y = stats(dys)
    stats_z = stats(dzs)
    stats_dist = stats(dists)

    print('\nPer-axis differences (rtk - manual):')
    print('X:', stats_x)
    print('Y:', stats_y)
    print('Z:', stats_z)
    print('\nEuclidean distance between matched points:')
    print(stats_dist)

    # also print per-match detail
    print('\nMatches detail: (rtk_idx, manual_idx, rtk_xyz, manual_xyz, dx,dy,dz,dist)')
    for m in matches:
        i, j, r, ma, dx, dy, dz = m
        # print( r, ma, dx, dy, dz, euclid(r, ma))
        print(i, j, r, ma, dx, dy, dz, euclid(r, ma))

if __name__ == '__main__':
    main()
