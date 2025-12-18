#!/usr/bin/env python3
"""
Plot 4 graphs for EVERY log file in the same folder as this script and save PNGs.

Fixes vs previous version:
- Robust parsing: supports float values for pos/posRaw (e.g. pos:0.465) and ints for motorPos/target.
- Stable layout: keeps a consistent figure size and prevents suptitle from being clipped (no bbox_inches="tight").
- Title format exactly: "test_YYYY-MM-DD_HH-MM-SS" (extracted from filename)

Behavior:
- Reads all *.log, *.txt, *.csv in the script folder (sorted by filename).
- X axis is time (dt default = 50 ms).
- Swaps posRaw and pos plots (posRaw first, then pos).
- Saves one PNG per input file: <stem>_4plots.png (or into --out-dir).

Expected line format (tabs/spaces both ok):
  pos:0.465    posRaw:0.465    motorPos:1892    target:1011
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # reliable PNG saving, avoids IDE backend issues
import matplotlib.pyplot as plt


# float/int number, optionally with exponent
_NUM = r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?"

# allow any whitespace between fields, tabs/spaces ok
PATTERN = re.compile(
    rf"pos:({_NUM})\s+posRaw:({_NUM})\s+motorPos:({_NUM})\s+target:({_NUM})"
)


def parse_log(file_path: Path):
    pos, pos_raw, motor_pos, target = [], [], [], []
    with file_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = PATTERN.search(line)
            if not m:
                continue
            a, b, c, d = map(float, m.groups())
            pos.append(a)
            pos_raw.append(b)
            motor_pos.append(c)
            target.append(d)
    return pos, pos_raw, motor_pos, target


def moving_average_trailing(x: list[float], n: int) -> list[float]:
    """Trailing (causal) moving average with window n, same length as input."""
    if n <= 1:
        return list(x)
    out = []
    s = 0.0
    for i, v in enumerate(x):
        s += float(v)
        if i >= n:
            s -= float(x[i - n])
            out.append(s / n)
        else:
            out.append(s / (i + 1))
    return out


def datetime_from_filename(p: Path) -> str:
    """
    Extract YYYY-MM-DD_HH-MM-SS from filename.
    Examples:
      output_2025-12-17_11-51-16.log -> 2025-12-17_11-51-16
      output_2025_12_17_11_51_16.log -> 2025-12-17_11-51-16
    Fallback: stem
    """
    name = p.name
    # date + time as 6 groups
    m = re.search(r"(\d{4})[-_](\d{2})[-_](\d{2})[-_](\d{2})[-_](\d{2})[-_](\d{2})", name)
    if m:
        y, mo, d, hh, mm, ss = m.groups()
        return f"{y}-{mo}-{d}_{hh}-{mm}-{ss}"

    # common: YYYY-MM-DD_HH-MM-SS (already)
    m = re.search(r"(\d{4}[-_]\d{2}[-_]\d{2})[-_](\d{2})[-_](\d{2})[-_](\d{2})", name)
    if m:
        date_part = m.group(1).replace("_", "-")
        hh, mm, ss = m.group(2), m.group(3), m.group(4)
        return f"{date_part}_{hh}-{mm}-{ss}"

    return p.stem


def plot_one(file_path: Path, dt_ms: float, ma_n: int, dpi: int, out_dir: Path | None) -> Path | None:
    pos, pos_raw, motor_pos, target = parse_log(file_path)
    n = len(pos)
    if n == 0:
        print(f"Skip (no samples matched): {file_path.name}")
        return None

    dt_s = dt_ms / 1000.0
    t = [i * dt_s for i in range(n)]

    # Smooth only posRaw and pos (set --ma-n 1 to disable)
    pos_raw_plot = moving_average_trailing(pos_raw, ma_n)
    pos_plot = moving_average_trailing(pos, ma_n)

    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(11, 8))

    dt_str = datetime_from_filename(file_path)
    fig.suptitle(f"test_{dt_str}")

    # swapped order: posRaw first, then pos
    axs[0].plot(t, pos_raw_plot)
    axs[0].set_ylabel("posRaw")
    axs[0].grid(True)

    axs[1].plot(t, pos_plot)
    axs[1].set_ylabel("pos")
    axs[1].grid(True)

    axs[2].plot(t, target)
    axs[2].set_ylabel("target")
    axs[2].grid(True)

    axs[3].plot(t, motor_pos)
    axs[3].set_ylabel("motorPos")
    axs[3].set_xlabel("time (s)")
    axs[3].grid(True)

    # Keep room for suptitle; keep output stable (no bbox_inches="tight")
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    if out_dir is None:
        out_path = file_path.with_name(file_path.stem + "_4plots.png")
    else:
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / (file_path.stem + "_4plots.png")

    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    print(f"Saved: {out_path}")
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dt-ms", type=float, default=50.0, help="Sample period in ms (default 50).")
    ap.add_argument("--ma-n", type=int, default=15, help="Moving average window N (default 15). Use 1 to disable.")
    ap.add_argument("--dpi", type=int, default=160, help="PNG dpi (default 160).")
    ap.add_argument("--out-dir", default=None, help="Optional output directory for PNGs.")
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else None

    files = []
    for ext in ("*.log", "*.txt", "*.csv"):
        files.extend(script_dir.glob(ext))
    files = sorted(files, key=lambda p: p.name.lower())

    if not files:
        print(f"No .log/.txt/.csv files found in: {script_dir}", file=sys.stderr)
        return 2

    for fp in files:
        try:
            plot_one(fp, dt_ms=args.dt_ms, ma_n=args.ma_n, dpi=args.dpi, out_dir=out_dir)
        except Exception as e:
            print(f"Error processing {fp.name}: {e}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
