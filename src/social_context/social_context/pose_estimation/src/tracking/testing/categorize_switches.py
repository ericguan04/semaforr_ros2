#!/usr/bin/env python3
"""
Categorizes ID switches from tracking_eval_results.csv into two types:
  - Rapid oscillation: switch happens within RAPID_THRESHOLD_SECONDS of
    the previous frame logged for that same ground-truth person. Likely
    a real tracking failure (brief loss/reacquisition of someone who
    never actually left view) rather than a genuine absence.
  - Genuine absence: switch happens after a longer gap, consistent with
    the person actually leaving and re-entering camera view.

"""

import csv
import sys

RAPID_THRESHOLD_SECONDS = 2.0


def categorize(csv_path):
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                'time': float(row['time']),
                'gt_id': row['gt_id'],
                'tracker_id': row['tracker_id'],
                'position_error': float(row['position_error']),
            })

    rows.sort(key=lambda r: r['time'])

    last_seen = {}  # gt_id -> (time, tracker_id)
    rapid_switches = []
    genuine_switches = []

    for row in rows:
        gt_id = row['gt_id']
        t = row['time']
        tracker_id = row['tracker_id']

        if gt_id in last_seen:
            prev_time, prev_tracker_id = last_seen[gt_id]
            if prev_tracker_id != tracker_id:
                gap = t - prev_time
                entry = {
                    'gt_id': gt_id,
                    'time': t,
                    'gap_seconds': gap,
                    'from_tracker': prev_tracker_id,
                    'to_tracker': tracker_id,
                }
                if gap < RAPID_THRESHOLD_SECONDS:
                    rapid_switches.append(entry)
                else:
                    genuine_switches.append(entry)

        last_seen[gt_id] = (t, tracker_id)

    total = len(rapid_switches) + len(genuine_switches)

    print("=" * 70)
    print("ID SWITCH CATEGORIZATION")
    print("=" * 70)
    print(f"Rapid threshold: switches within {RAPID_THRESHOLD_SECONDS}s of the ")
    print(f"previous logged frame for that person are categorized as 'rapid'")
    print(f"(implausible for a genuine walk-out-and-back-in).\n")

    print(f"Total switches: {total}")
    print(f"  Rapid oscillation (likely real bug):  {len(rapid_switches)}")
    print(f"  Genuine absence (likely expected):     {len(genuine_switches)}")
    print()

    if rapid_switches:
        print("-" * 70)
        print("RAPID OSCILLATION SWITCHES (worth investigating):")
        print("-" * 70)
        for s in rapid_switches:
            print(f"  gt={s['gt_id']}  t={s['time']:.3f}  gap={s['gap_seconds']:.3f}s  "
                  f"tracker {s['from_tracker']} -> {s['to_tracker']}")

    if genuine_switches:
        print("-" * 70)
        print("GENUINE ABSENCE SWITCHES (likely expected, not a bug):")
        print("-" * 70)
        for s in genuine_switches:
            print(f"  gt={s['gt_id']}  t={s['time']:.3f}  gap={s['gap_seconds']:.3f}s  "
                  f"tracker {s['from_tracker']} -> {s['to_tracker']}")

    print("=" * 70)
    if total > 0:
        pct_rapid = 100.0 * len(rapid_switches) / total
        print(f"SUMMARY: {pct_rapid:.1f}% of switches are rapid oscillation, "
              f"{100 - pct_rapid:.1f}% are genuine absence")
    print("=" * 70)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 categorize_switches.py <path_to_csv>")
        sys.exit(1)
    categorize(sys.argv[1])