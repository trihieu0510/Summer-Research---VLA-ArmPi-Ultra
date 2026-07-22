#!/usr/bin/env python3
"""
trial_log.py
============
One-line-per-trial JSONL logging for pick evaluation (the Week 5–6 harness).

Every pick attempt — CLI, chat, or a formal eval session — appends ONE json
line with everything the writeup needs: what was asked, what was seen, where
the arm went, what happened, and how long each stage took. eval_analysis.py
turns these files into success rates with Wilson confidence intervals.

Design rule: logging must NEVER break a pick — every file operation is
swallowed on failure (a full disk should cost data, not a grasp).

No ROS imports: usable from any process, including the laptop.
"""

import json
import os
import time

PICK_LOG_DEFAULT = '~/pick_trials.jsonl'


class TrialLog:
    """Accumulates one trial's facts; writes a single JSONL line on finish().

    path=None collects in memory only (a no-op sink — run_pick logs
    unconditionally, callers decide whether it lands on disk).

    Appends assume a SINGLE writer per path at a time (true today: the
    agent's worker thread and the CLI process never run picks concurrently).
    """

    def __init__(self, path=None, **fixed):
        self.path = os.path.expanduser(path) if path else None
        self._t0 = time.time()
        self._done = False
        self.rec = {'ts': round(self._t0, 2), **fixed}

    def note(self, **fields):
        """Merge facts into the record (later calls overwrite same keys)."""
        self.rec.update(fields)

    def stage(self, name):
        """Stamp the elapsed seconds at which a pipeline stage was reached."""
        self.rec.setdefault('stages', {})[name] = round(time.time() - self._t0, 2)

    def finish(self, outcome):
        """Set the outcome and append the line. First outcome wins."""
        if self._done:
            return
        self._done = True
        self.rec['outcome'] = outcome
        self.rec['duration_s'] = round(time.time() - self._t0, 2)
        if not self.path:
            return
        try:
            with open(self.path, 'a') as f:
                f.write(json.dumps(self.rec) + '\n')
        except OSError:
            pass
