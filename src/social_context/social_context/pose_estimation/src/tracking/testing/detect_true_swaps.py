#!/usr/bin/env python3
"""
Scans tracking_eval_results.csv for "true swap" events: two ground-truth
people trade tracker IDs at the exact same logged timestamp, e.g.

    t=12.345  gt=1  tracker 3 -> 7
    t=12.345  gt=2  tracker 7 -> 3

This is the signature of a genuine identity swap (the tracker literally
confused the two people), as opposed to two unrelated switches that just
happen to land in the same CSV but don't reciprocate each other's IDs.

Note: 'time' in the CSV comes from a separate time.time() call per record
within the same callback, so exact float equality across two gt_ids only
happens when both records land on the same wall-clock tick. If this finds
few/no matches, rerun categorize_switches.py's rapid-oscillation output
and check for swaps within a small time window instead of exact equality.
"""

import csv
import sys
from collections import defaultdict


def find_true_swaps(csv_path):
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                'time': float(row['time']),
                'gt_id': row['gt_id'],
                'tracker_id': row['tracker_id'],
            })

    rows.sort(key=lambda r: r['time'])

    last_seen = {}  # gt_id -> tracker_id
    switches_by_time = defaultdict(list)  # time -> [switch entries]

    for row in rows:
        gt_id = row['gt_id']
        t = row['time']
        tracker_id = row['tracker_id']

        if gt_id in last_seen and last_seen[gt_id] != tracker_id:
            switches_by_time[t].append({
                'gt_id': gt_id,
                'from_tracker': last_seen[gt_id],
                'to_tracker': tracker_id,
            })

        last_seen[gt_id] = tracker_id

    true_swaps = []
    seen_pairs_at_time = set()

    for t, switches in switches_by_time.items():
        if len(switches) < 2:
            continue
        for i in range(len(switches)):
            for j in range(i + 1, len(switches)):
                a, b = switches[i], switches[j]
                if a['gt_id'] == b['gt_id']:
                    continue
                key = (t, a['gt_id'], b['gt_id'])
                if key in seen_pairs_at_time:
                    continue
                if a['from_tracker'] == b['to_tracker'] and a['to_tracker'] == b['from_tracker']:
                    seen_pairs_at_time.add(key)
                    true_swaps.append({
                        'time': t,
                        'gt_a': a['gt_id'],
                        'gt_b': b['gt_id'],
                        'tracker_x': a['from_tracker'],
                        'tracker_y': a['to_tracker'],
                    })

    concurrent_timestamps = sum(1 for switches in switches_by_time.values() if len(switches) >= 2)

    print("=" * 70)
    print("TRUE SWAP DETECTION (two ground-truth people trade tracker IDs")
    print("at the exact same timestamp)")
    print("=" * 70)
    print(f"Timestamps with 2+ concurrent switches: {concurrent_timestamps}")
    print(f"True swaps found: {len(true_swaps)}")
    print()

    if true_swaps:
        print("-" * 70)
        print("SWAP EVENTS:")
        print("-" * 70)
        for s in true_swaps:
            print(f"  t={s['time']:.6f}  gt={s['gt_a']} <-> gt={s['gt_b']}   "
                  f"tracker {s['tracker_x']} <-> tracker {s['tracker_y']}")
    else:
        print("No exact-timestamp swaps found.")
        if concurrent_timestamps == 0:
            print("(No timestamps even had 2+ switches logged concurrently -- ")
            print(" 'time' is likely too high-resolution for exact ties. Consider")
            print(" a small-tolerance window instead of exact equality.)")

    print("=" * 70)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 detect_true_swaps.py <path_to_csv>")
        sys.exit(1)
    find_true_swaps(sys.argv[1])
