#!/usr/bin/env python3
"""
eval_analysis.py
================
Turn trial JSONL logs into success rates with Wilson 95% confidence intervals
— the numbers the Week-6/7 writeup reports.

Understands BOTH record kinds (mixed files/args are fine):
  * operator records from eval_runner (~/eval_trials.jsonl) — field `verdict`
    (ground truth; this is what headline numbers come from)
  * robot self-reports from run_pick   (~/pick_trials.jsonl) — field `outcome`
    (per-stage detail: sectors, wrist deltas, timings, LLM latency)

Pure stdlib, NO ROS — runs on the laptop against files copied from the Pi:
    scp pi@192.168.137.X:~/eval_trials.jsonl .
    python3 eval_analysis.py eval_trials.jsonl pick_trials.jsonl
    python3 eval_analysis.py eval_trials.jsonl --md report.md

Self-check (no files needed):
    python3 eval_analysis.py --selftest
"""

import argparse
import json
import math
import os
import sys
from collections import Counter, defaultdict

SUCCESS_VERDICTS = {'success', 'placed'}


def wilson(k, n, z=1.96):
    """Wilson score interval: (rate, lo, hi). Correct for small n / p near
    0 or 1, unlike the naive normal interval — which is why we use it."""
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return p, max(0.0, centre - half), min(1.0, centre + half)


def load(paths):
    """Split records into (operator, robot) lists by their fields."""
    operator, robot = [], []
    for path in paths:
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            print(f'warning: {path} not found, skipping', file=sys.stderr)
            continue
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                (operator if 'verdict' in rec else robot).append(rec)
    return operator, robot


def rate_table(recs, key, result_field):
    """[(group, k, n, rate, lo, hi)] sorted by group; None-keyed recs skipped."""
    groups = defaultdict(list)
    for r in recs:
        g = r.get(key)
        if g is not None:
            groups[str(g)].append(r)
    rows = []
    for g in sorted(groups):
        rs = groups[g]
        k = sum(1 for r in rs if r.get(result_field) in SUCCESS_VERDICTS)
        rows.append((g, k, len(rs)) + wilson(k, len(rs)))
    return rows


def fmt_rows(title, rows, out):
    if not rows:
        return
    out.append(f'\n### {title}\n')
    out.append('| group | success | n | rate | 95% CI |')
    out.append('|---|---|---|---|---|')
    for g, k, n, p, lo, hi in rows:
        out.append(f'| {g} | {k} | {n} | {p * 100:.1f}% '
                   f'| [{lo * 100:.1f}%, {hi * 100:.1f}%] |')


def summarize(operator, robot):
    out = ['# Pick evaluation report']

    if operator:
        n = len(operator)
        k = sum(1 for r in operator if r.get('verdict') in SUCCESS_VERDICTS)
        p, lo, hi = wilson(k, n)
        out.append(f'\n## Operator-verified trials (ground truth, n={n})\n')
        out.append(f'**Overall: {k}/{n} = {p * 100:.1f}% '
                   f'(95% CI [{lo * 100:.1f}%, {hi * 100:.1f}%])**')
        fmt_rows('By color', rate_table(operator, 'color', 'verdict'), out)
        fmt_rows('By position', rate_table(operator, 'label', 'verdict'), out)
        fmt_rows('By block orientation (rotated?)',
                 rate_table(operator, 'rotated', 'verdict'), out)
        fmt_rows('By phrasing', rate_table(operator, 'phrasing', 'verdict'), out)
        out.append('\n### Verdict distribution\n')
        for verdict, cnt in Counter(r.get('verdict') for r in operator).most_common():
            out.append(f'- {verdict}: {cnt}')
        mismatches = [r for r in operator
                      if r.get('robot_claim') and r['robot_claim'] != r['verdict']]
        out.append(f'\nRobot self-report disagreed with the operator in '
                   f'{len(mismatches)}/{n} trials '
                   '(verification-layer error rate).')

    if robot:
        n = len(robot)
        k = sum(1 for r in robot if r.get('outcome') in SUCCESS_VERDICTS)
        p, lo, hi = wilson(k, n)
        out.append(f'\n## Robot self-reported picks (n={n})\n')
        out.append(f'**Overall: {k}/{n} = {p * 100:.1f}% '
                   f'(95% CI [{lo * 100:.1f}%, {hi * 100:.1f}%])**')
        fmt_rows('By color', rate_table(robot, 'color', 'outcome'), out)
        fmt_rows('By scan sector', rate_table(robot, 'sector', 'outcome'), out)
        out.append('\n### Outcome distribution\n')
        for outcome, cnt in Counter(r.get('outcome') for r in robot).most_common():
            out.append(f'- {outcome}: {cnt}')
        durations = sorted(r['duration_s'] for r in robot if 'duration_s' in r)
        if durations:
            mid = durations[len(durations) // 2]
            out.append(f'\nPick duration: median {mid:.1f}s, '
                       f'max {durations[-1]:.1f}s.')
        llm = sorted(r['llm_s'] for r in robot if r.get('llm_s'))
        if llm:
            out.append(f'LLM latency: median {llm[len(llm) // 2]:.1f}s, '
                       f'max {llm[-1]:.1f}s.')

    if not operator and not robot:
        out.append('\nNo records found — check the file paths.')
    return '\n'.join(out)


def selftest():
    """Synthesize a plausible 120-trial session and sanity-check the math."""
    import random
    rng = random.Random(42)
    operator = []
    for i in range(120):
        pos = ['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'R1', 'R2', 'R3', 'R4'][i % 10]
        rotated = pos.startswith('R')
        p_success = 0.85 if not rotated else 0.7    # rotated picks are harder
        verdict = 'success' if rng.random() < p_success else \
            rng.choice(['missed', 'knocked', 'not_seen'])
        operator.append({
            'trial_id': f'r{i // 30 + 1}:{pos}:red', 'label': pos,
            'rotated': rotated, 'color': ['red', 'green', 'blue'][i % 3],
            'phrasing': f'phrase {i % 4}', 'verdict': verdict,
            'robot_claim': verdict if rng.random() < 0.95 else 'success',
        })
    robot = [{'color': 'red', 'outcome': 'success', 'sector': 0,
              'duration_s': 21.0 + i, 'llm_s': 1.5} for i in range(5)]

    # Wilson sanity: known values (e.g. 8/10 -> [0.49, 0.94] roughly).
    p, lo, hi = wilson(8, 10)
    assert abs(p - 0.8) < 1e-9 and 0.4 < lo < 0.6 and 0.9 < hi < 1.0, (p, lo, hi)
    p, lo, hi = wilson(0, 0)
    assert (p, lo, hi) == (0.0, 0.0, 0.0)
    p, lo, hi = wilson(10, 10)
    assert hi == 1.0 and lo > 0.6

    report = summarize(operator, robot)
    assert 'Operator-verified' in report and 'Wilson' not in report
    assert '| P1 |' in report and 'By phrasing' in report
    print(report)
    print('\nSELF-TEST PASSED', file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[3])
    ap.add_argument('files', nargs='*', help='JSONL trial logs (mixed kinds OK)')
    ap.add_argument('--md', help='also write the report to this markdown file')
    ap.add_argument('--selftest', action='store_true',
                    help='run on synthetic data; no files needed')
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return
    if not args.files:
        ap.error('give at least one JSONL file (or --selftest)')

    operator, robot = load(args.files)
    report = summarize(operator, robot)
    print(report)
    if args.md:
        with open(os.path.expanduser(args.md), 'w') as f:
            f.write(report + '\n')
        print(f'\n(report written to {args.md})', file=sys.stderr)


if __name__ == '__main__':
    main()
