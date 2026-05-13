#!/usr/bin/env python3
import argparse
import json
import math
import os
from collections import Counter
from statistics import mean, median


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round((pct / 100.0) * (len(ordered) - 1)))))
    return ordered[idx]


def latest_log(log_dir):
    candidates = [
        os.path.join(log_dir, name)
        for name in os.listdir(log_dir)
        if name.startswith("jetson_pipeline_") and name.endswith(".jsonl")
    ]
    if not candidates:
        raise FileNotFoundError(f"no jetson_pipeline_*.jsonl in {log_dir}")
    return max(candidates, key=os.path.getmtime)


def parse_log(path):
    rows = []
    with open(path, "r", encoding="utf-8") as fp:
        for line in fp:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Analyze ultra_yubin pipeline logs and recommend tracking tuning."
    )
    parser.add_argument(
        "path",
        nargs="?",
        default=None,
        help="pipeline jsonl path. Defaults to latest benchmark_logs/jetson_pipeline_*.jsonl",
    )
    parser.add_argument("--log-dir", default="benchmark_logs")
    parser.add_argument("--stable-window", type=int, default=80)
    args = parser.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    log_dir = args.log_dir if os.path.isabs(args.log_dir) else os.path.join(root, args.log_dir)
    path = args.path or latest_log(log_dir)
    rows = parse_log(path)

    target_rows = []
    no_target_rows = 0
    for row in rows:
        if row.get("event") == "no_target":
            no_target_rows += 1
        if row.get("event") != "target_track":
            continue
        target = row.get("target") or {}
        camera = row.get("camera") or {}
        ultra = row.get("ultra96") or {}
        if not target.get("detected"):
            continue
        if "cx" not in target or "cy" not in target or "cx" not in camera or "cy" not in camera:
            continue
        err_x = int(target["cx"]) - int(camera["cx"])
        err_y = int(target["cy"]) - int(camera["cy"])
        target_rows.append((row, target, camera, ultra, err_x, err_y))

    print(f"log={path}")
    print(f"frames={len(rows)} target_frames={len(target_rows)} no_target_frames={no_target_rows}")
    if not target_rows:
        print("no target rows to analyze")
        return

    err_xs = [x[4] for x in target_rows]
    err_ys = [x[5] for x in target_rows]
    widths = [int((x[1] or {}).get("w") or 0) for x in target_rows]
    heights = [int((x[1] or {}).get("h") or 0) for x in target_rows]
    confs = [float((x[1] or {}).get("conf") or 0.0) for x in target_rows]
    areas = [w * h for w, h in zip(widths, heights)]
    abs_xs = [abs(v) for v in err_xs]
    abs_ys = [abs(v) for v in err_ys]
    ultra_rows = [x[3] for x in target_rows]
    reply_kinds = Counter((u.get("reply") or "").split(",", 1)[0] or "NONE" for u in ultra_rows)
    srcs = Counter(u.get("src") or "unknown" for u in ultra_rows)
    usb_sent = sum(1 for u in ultra_rows if int(u.get("usb") or 0) == 1)
    rtts = [float(u.get("rtt_ms") or 0.0) for u in ultra_rows if float(u.get("rtt_ms") or 0.0) > 0.0]
    infer_ms = [
        float(row.get("fps") or 0.0)
        for row, *_ in target_rows
        if 0.0 < float(row.get("fps") or 0.0) < 200.0
    ]

    print(
        "error_px "
        f"mean=({mean(err_xs):.1f},{mean(err_ys):.1f}) "
        f"median=({median(err_xs):.1f},{median(err_ys):.1f}) "
        f"mean_abs=({mean(abs_xs):.1f},{mean(abs_ys):.1f}) "
        f"p90_abs=({percentile(abs_xs, 90):.1f},{percentile(abs_ys, 90):.1f})"
    )
    print(f"reply={dict(reply_kinds)} src={dict(srcs)} usb_sent={usb_sent}/{len(target_rows)}")
    print(
        "bbox "
        f"w_p10/med/p90=({percentile(widths, 10):.0f},{median(widths):.0f},{percentile(widths, 90):.0f}) "
        f"h_p10/med/p90=({percentile(heights, 10):.0f},{median(heights):.0f},{percentile(heights, 90):.0f}) "
        f"area_p10/med/p90=({percentile(areas, 10):.0f},{median(areas):.0f},{percentile(areas, 90):.0f}) "
        f"conf_p10/med/p90=({percentile(confs, 10):.2f},{median(confs):.2f},{percentile(confs, 90):.2f})"
    )
    small_rows = sum(1 for w, h, area in zip(widths, heights, areas) if h < 25 or w < 35 or area < 1500)
    weak_rows = sum(1 for c in confs if c < 0.60)
    print(f"target_quality small_or_thin={small_rows}/{len(target_rows)} weak_conf_lt_0.60={weak_rows}/{len(target_rows)}")
    if rtts:
        print(f"ultra96_rtt_ms mean={mean(rtts):.1f} median={median(rtts):.1f} p90={percentile(rtts, 90):.1f}")
    if infer_ms:
        infer_mean = mean(infer_ms)
        infer_median = median(infer_ms)
        infer_fps_est = 1000.0 / infer_mean if infer_mean > 0.0 else 0.0
        print(
            "infer_ms "
            f"mean={infer_mean:.1f} median={infer_median:.1f} "
            f"fps_est_from_mean={infer_fps_est:.1f}"
        )

    stable = target_rows[-min(args.stable_window, len(target_rows)) :]
    sx = [x[4] for x in stable]
    sy = [x[5] for x in stable]
    stable_abs_x = [abs(v) for v in sx]
    stable_abs_y = [abs(v) for v in sy]
    print(
        f"last_{len(stable)}_target_frames "
        f"median_err=({median(sx):.1f},{median(sy):.1f}) "
        f"mean_abs=({mean(stable_abs_x):.1f},{mean(stable_abs_y):.1f})"
    )

    # Offset convention: target error is target - camera_center. To aim the
    # camera so the target lands closer to center, shift aim center by the
    # observed median error.
    rec_off_x = int(round(median(sx)))
    rec_off_y = int(round(median(sy)))
    print("recommendation:")
    if srcs and (srcs.get("pl", 0) != len(target_rows)):
        print("  pl_path: WARNING not all target rows used src=pl.")
    if small_rows > len(target_rows) * 0.10:
        print("  detection: many target boxes are small/thin; raise TRACK_TARGET_MIN_AREA/H or inspect YOLO false positives.")
    if weak_rows > len(target_rows) * 0.10:
        print("  detection: many target boxes are below 0.60 confidence; raise target filter or improve lighting/model.")
    print(f"  fixed_offset_test: ULTRA_YUBIN_AIM_OFFSET_X={rec_off_x} ULTRA_YUBIN_AIM_OFFSET_Y={rec_off_y}")
    if mean(abs_xs) > 90 or mean(abs_ys) > 70:
        print("  speed: tracking error is large; try lower control period first, then faster PL step table if needed.")
        print("  run_env: ULTRA_YUBIN_CONTROL_PERIOD_SEC=0.02 ULTRA_YUBIN_DEADBAND_PX=5")
    elif abs(rec_off_x) > 8 or abs(rec_off_y) > 8:
        print("  offset: center convergence is close, but has a repeatable bias. Test the fixed offset above.")
    else:
        print("  center: offset is small. Avoid chasing noise; tune speed/damping only.")


if __name__ == "__main__":
    main()
