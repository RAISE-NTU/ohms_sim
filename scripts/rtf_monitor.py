#!/usr/bin/env python3
"""Real-time factor monitor for Ignition Gazebo — live histogram.

Usage: ign topic -e -t /stats | python3 rtf_monitor.py
"""

import sys
import os
import re
import math

BINS = 30
HEIGHT = 16
all_values = []
all_min = float('inf')
all_max = 0.0
total = 0.0


def draw():
    os.system('clear')
    n = len(all_values)
    if n < 2:
        print(f"  Collecting samples... ({n})")
        return

    mean = total / n
    variance = sum((v - mean) ** 2 for v in all_values) / n
    std = math.sqrt(variance)

    lo = max(0.0, mean - 4 * std) if std > 0 else all_min - 0.01
    hi = mean + 4 * std if std > 0 else all_max + 0.01
    if lo >= hi:
        lo, hi = all_min - 0.01, all_max + 0.01
    bin_width = (hi - lo) / BINS

    counts = [0] * BINS
    for v in all_values:
        idx = int((v - lo) / bin_width)
        idx = max(0, min(BINS - 1, idx))
        counts[idx] += 1

    max_count = max(counts)

    print(f"  RTF Histogram  ({n} samples)")
    print(f"  Mean: {mean:.4f}  Std: {std:.4f}  Min: {all_min:.4f}  Max: {all_max:.4f}")
    print()

    for row in range(HEIGHT, 0, -1):
        threshold = (row / HEIGHT) * max_count
        label = f"  {int(threshold):>4} │"
        bar = ""
        for c in counts:
            if c >= threshold:
                bar += "██"
            else:
                bar += "  "
        print(f"{label}{bar}│")

    print(f"       └{'──' * BINS}┘")

    # x-axis labels
    labels = f"       {lo:<.2f}"
    mid = lo + (hi - lo) / 2
    pad = BINS - len(f"{lo:.2f}") - len(f"{hi:.2f}")
    labels += f"{mid:^{pad}.2f}"
    labels += f"{hi:.2f}"
    print(labels)

    # mark mean with arrow
    mean_bin = int((mean - lo) / bin_width)
    mean_bin = max(0, min(BINS - 1, mean_bin))
    arrow_pad = 7 + mean_bin * 2
    print(f"{' ' * arrow_pad}▲ mean")

    print(f"\n  (Ctrl+C to exit)")


def main():
    global all_min, all_max, total

    try:
        for line in sys.stdin:
            match = re.search(r'real_time_factor:\s*([\d.]+)', line.strip())
            if not match:
                continue

            rtf = float(match.group(1))
            all_values.append(rtf)
            total += rtf
            all_min = min(all_min, rtf)
            all_max = max(all_max, rtf)
            draw()

    except KeyboardInterrupt:
        if all_values:
            n = len(all_values)
            mean = total / n
            variance = sum((v - mean) ** 2 for v in all_values) / n
            std = math.sqrt(variance)
            print(f"\n--- Summary ({n} samples) ---")
            print(f"  Mean: {mean:.4f}  Std: {std:.4f}")
            print(f"  Min:  {all_min:.4f}  Max: {all_max:.4f}")


if __name__ == '__main__':
    main()
