"""Read-only HTTP capture of real source health and stationary drift. No ROS publishers."""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8766')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 2 <= args.duration <= 300:
        parser.error('duration must be 2..300 seconds')
    samples = []
    sample_times = []
    deadline = time.monotonic()+args.duration
    while time.monotonic() < deadline:
        with urllib.request.urlopen(args.url+'/healthz', timeout=5) as response:
            samples.append(json.load(response))
            sample_times.append(time.monotonic())
        time.sleep(.25)
    first, last = samples[0], samples[-1]
    elapsed = sample_times[-1]-sample_times[0]
    report = dict(duration_s=elapsed, polls=len(samples),
                  all_inputs_healthy=all(s['input_health'] for s in samples),
                  faults=[s['fault'] for s in samples if s['fault']],
                  alignment=last['alignment'], sources={},
                  publisher_counts=last['publisher_counts'],
                  measurement_covariances=last['measurement_covariances'],
                  tf_edges=last['tf_edges'], diagnostics=last['diagnostics'])
    okay = report['all_inputs_healthy'] and not report['faults']
    for key in ('wheel', 'raw_lidar', 'lidar', 'fused'):
        a, b = first['sources'][key], last['sources'][key]
        poses = [s['sources'][key]['latest'] for s in samples if s['sources'][key]['latest']]
        if not poses:
            okay = False
            report['sources'][key] = dict(messages=0)
            continue
        start, end = poses[0], poses[-1]
        report['sources'][key] = dict(messages=b['received']-a['received'],
            received_hz=(b['received']-a['received'])/elapsed,
            all_live=all(s['sources'][key]['status']=='live' for s in samples),
            first=start, last=end,
            displacement_m=math.hypot(end['x']-start['x'], end['y']-start['y']),
            heading_change_rad=math.atan2(math.sin(end['yaw']-start['yaw']), math.cos(end['yaw']-start['yaw'])),
            max_abs_vx=max(abs(p['linear_x']) for p in poses),
            max_abs_wz=max(abs(p['angular_z']) for p in poses))
        okay = okay and report['sources'][key]['messages']>0 and report['sources'][key]['all_live']
    report['stream_check_passed'] = okay
    wheel = report['sources']['wheel']
    report['observed_stationary'] = wheel.get('max_abs_vx', math.inf)<.01 and wheel.get('max_abs_wz', math.inf)<.03
    report['interpretation'] = 'Stream/drift evidence; stationary status from wheel speeds only. No ground-truth or moving-robot accuracy claim.'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if okay else 1)


if __name__ == '__main__':
    main()
