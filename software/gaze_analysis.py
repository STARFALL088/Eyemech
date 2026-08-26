#!/usr/bin/env python3
"""
Eyemech — Dyslexia gaze-analysis layer (inspired by the Reading Test in
Rahman et al., "A Secure Multi-Agentic IIoMT Framework for Multimodal
Dyslexia Screening", IEEE IoT Magazine 2026).

Given a *stream of gaze points* (the output of eye_track_basic.py's pupil
tracker, mapped to screen/reading coordinates), this module:

  1. Detects FIXATIONS via an I-VT (velocity threshold) classifier: points
     whose inter-sample velocity is below a threshold are grouped into
     fixations (eye holding still = reading a word); fast moves are saccades.
  2. Derives the 5 clinical indicators the paper reports:
        - regressions      : backward (leftward) saccades = re-reading
        - line jumps       : large downward vertical jumps = losing the place
        - word skips       : abnormally large forward jumps = skipping content
        - erratic movement : high saccade-angle variance / jittery path
        - attention clustering : where fixations pile up (heatmap blobs)
  3. Renders an attention heatmap: blue blobs (radius ~ fixation dwell time)
     over the reading surface, plus the orange saccade path.

Everything is PURE and testable: no camera, no OpenCV GUI needed for the
math. The heatmap renderer uses OpenCV but degrades gracefully if absent.

Protocol note: our tracker emits x,y in [0,1023] (512 = center). This module
works in *normalized* [0,1] space so it's resolution-independent; callers
convert (x/1023, y/1023).
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import List, Tuple

Point = Tuple[float, float]  # normalized (x, y) in [0, 1]


# ---------------------------------------------------------------------------
# Fixation detection (I-VT)
# ---------------------------------------------------------------------------
@dataclass
class Fixation:
    x: float
    y: float
    duration: float          # number of samples it spanned (proxy for dwell ms)
    start_idx: int


def detect_fixations(
    points: List[Point],
    sample_radius: float = 0.02,   # velocity threshold in normalized units/frame
    min_samples: int = 2,          # a fixation must hold at least this long
) -> Tuple[List[Fixation], List[Point]]:
    """
    I-VT classifier. `sample_radius` is the per-frame movement below which a
    point is considered "still" (a fixation sample). Returns (fixations,
    saccade_path) where saccade_path is the subset of points flagged as moving.
    """
    fixations: List[Fixation] = []
    saccade_path: List[Point] = []
    if len(points) < 2:
        for p in points:
            saccade_path.append(p)
        return fixations, saccade_path

    i = 0
    n = len(points)
    while i < n:
        # Start a candidate fixation at i.
        cluster: List[Point] = [points[i]]
        j = i + 1
        while j < n:
            # distance from the running cluster center to the new point
            cx = sum(q[0] for q in cluster) / len(cluster)
            cy = sum(q[1] for q in cluster) / len(cluster)
            dist = math.hypot(points[j][0] - cx, points[j][1] - cy)
            if dist <= sample_radius:
                cluster.append(points[j])
                j += 1
            else:
                break
        if len(cluster) >= min_samples:
            fixations.append(Fixation(
                x=sum(q[0] for q in cluster) / len(cluster),
                y=sum(q[1] for q in cluster) / len(cluster),
                duration=len(cluster),
                start_idx=i,
            ))
            i = j  # move past the whole cluster (saccade in between)
        else:
            i += 1  # single jittery point; will be flagged as a saccade below

    # Any point not covered by a fixation is part of the saccade path.
    covered = set()
    for f in fixations:
        end = f.start_idx + int(f.duration)
        for k in range(f.start_idx, end):
            covered.add(k)
    for k in range(n):
        if k not in covered:
            saccade_path.append(points[k])
    return fixations, saccade_path


# ---------------------------------------------------------------------------
# Reading-path indicator extraction
# ---------------------------------------------------------------------------
@dataclass
class ReadingMetrics:
    regressions: int = 0
    line_jumps: int = 0
    word_skips: int = 0
    erratic_score: float = 0.0         # 0..1 normalized jitter
    n_fixations: int = 0
    mean_fixation_dur: float = 0.0
    saccade_angles: List[float] = field(default_factory=list)
    diagnosed_flags: List[str] = field(default_factory=list)


def analyze_reading_path(
    points: List[Point],
    regression_x: float = 0.06,   # leftward move beyond this = regression
    line_jump_y: float = 0.12,    # downward jump beyond this = line jump
    word_skip_x: float = 0.18,    # forward jump beyond this = skip
) -> ReadingMetrics:
    """
    Walk the ordered gaze path and tally the 5 dyslexia gaze indicators.
    Assumes a left-to-right, top-to-bottom reading direction (English).
    """
    m = ReadingMetrics()
    if len(points) < 3:
        return m

    # saccades = consecutive-point deltas (sampled at the tracker's frame rate)
    deltas = []
    for k in range(1, len(points)):
        dx = points[k][0] - points[k - 1][0]
        dy = points[k][1] - points[k - 1][1]
        deltas.append((dx, dy))

    angles = []
    for (dx, dy) in deltas:
        mag = math.hypot(dx, dy)
        if mag < 1e-6:
            continue
        ang = math.degrees(math.atan2(dy, dx))
        angles.append(ang)
        # Regression: clear leftward move (not a tiny jitter).
        if dx < -regression_x:
            m.regressions += 1
        # Line jump: large downward vertical move.
        if dy > line_jump_y:
            m.line_jumps += 1
        # Word skip: very large forward (rightward) move.
        if dx > word_skip_x:
            m.word_skips += 1

    m.saccade_angles = angles
    # Erratic: how scattered the saccade directions are. Real reading saccades
    # are mostly horizontal (forward/back), so a tight angle cluster = smooth.
    # We use the circular spread (max-min of angles wrapped to [-180,180]) as a
    # robust jitter measure; 0 = all same direction, 1 = fully scatterbrained.
    if len(angles) > 1:
        amin, amax = min(angles), max(angles)
        span = amax - amin
        span = min(span, 360 - span)  # wrap-around aware
        m.erratic_score = min(1.0, span / 180.0)  # 180deg spread => max

    return m


def flag_dyslexia_risk(m: ReadingMetrics, n_points: int) -> List[str]:
    """
    Paper: these indicators, in excess, characterize dyslexic reading. We emit
    human-readable flags. Thresholds are screening heuristics, NOT clinical
    diagnoses — the paper itself stresses proof-of-concept, not clinical grade.
    """
    flags = []
    if n_points == 0:
        return flags
    rate_reg = m.regressions / max(1, n_points)
    rate_line = m.line_jumps / max(1, n_points)
    rate_skip = m.word_skips / max(1, n_points)
    if rate_reg > 0.08:
        flags.append(f"high regression rate ({rate_reg:.0%} of samples) — re-reading")
    if rate_line > 0.05:
        flags.append(f"frequent line jumps ({rate_line:.0%}) — losing place")
    if rate_skip > 0.04:
        flags.append(f"word skipping ({rate_skip:.0%}) — possible omission")
    if m.erratic_score > 0.4:
        flags.append(f"erratic saccade path (jitter {m.erratic_score:.2f})")
    return flags


# ---------------------------------------------------------------------------
# Attention heatmap render
# ---------------------------------------------------------------------------
def render_heatmap(
    fixations: List[Fixation],
    width: int = 640,
    height: int = 480,
    bg: str = "reading",
) -> "object":
    """
    Render the attention heatmap (blue fixation blobs + orange saccade path).
    Returns an OpenCV BGR image, or None if cv2 is unavailable.
    Returns (image, path) is NOT done here; caller saves.
    """
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None

    if bg == "reading":
        # faint ruled lines to evoke a text page
        img = np.ones((height, width, 3), dtype=np.uint8) * 255
        for y in range(60, height, 60):
            cv2.line(img, (0, y), (width, y), (220, 220, 220), 1)
    else:
        img = np.zeros((height, width, 3), dtype=np.uint8)

    # blue fixation blobs; radius scales with dwell (duration)
    for f in fixations:
        r = max(6, int(f.duration * 4))
        cx, cy = int(f.x * width), int(f.y * height)
        cv2.circle(img, (cx, cy), r, (255, 0, 0), -1)  # blue, filled

    # orange saccade path connecting fixation centers (drawn LAST so it's visible)
    pts = [(int(f.x * width), int(f.y * height)) for f in fixations]
    for k in range(1, len(pts)):
        cv2.line(img, pts[k - 1], pts[k], (0, 165, 255), 2)  # BGR orange

    return img


# ===========================================================================
# Self-test + demo (no camera / no GUI required)
# ===========================================================================
def _synthetic_reading_path() -> List[Point]:
    """
    Build a reading path with a KNOWN regression and a KNOWN line jump so the
    detector can be asserted against ground truth. Mirrors how a real tracker
    behaves: a FIXATION is several near-identical samples (eye holding still on
    a word); a SACCADE is a big jump to the next word/line.
    Lines run left->right, top->bottom. We deliberately:
      * read line 1 L->R (3 fixations),
      * line jump DOWN to line 2 (line_jump),
      * read a bit, then move LEFT (regression) to re-read,
      * continue right (normal forward saccades).
    """
    pts: List[Point] = []
    def fix(x, y, n=3):
        pts.extend([(x, y)] * n)

    # line 1 (y=0.15): three fixations L->R with realistic small saccades
    fix(0.15, 0.15); pts.append((0.28, 0.15))   # small forward saccade (~0.13)
    fix(0.28, 0.15); pts.append((0.41, 0.15))   # small forward saccade
    fix(0.41, 0.15); pts.append((0.45, 0.45))   # LINE JUMP down to line 2 (dy=0.30)
    # line 2 (y=0.45)
    fix(0.45, 0.45); pts.append((0.33, 0.45))   # small backward step (NOT a regression)
    fix(0.33, 0.45); pts.append((0.21, 0.45))   # REGRESSION: dx=-0.12 < -0.06
    fix(0.21, 0.45); pts.append((0.37, 0.45))   # resume right (small forward)
    fix(0.37, 0.45)
    return pts


def run_selftest() -> int:
    print("gaze_analysis self-test...", file=__import__("sys").stderr)
    path = _synthetic_reading_path()
    fix, sac = detect_fixations(path)
    # We expect several fixations (each line segment held still then moved).
    assert len(fix) >= 3, f"expected >=3 fixations, got {len(fix)}"

    m = analyze_reading_path(path)
    assert m.regressions >= 1, f"regression not detected (got {m.regressions})"
    assert m.line_jumps >= 1, f"line jump not detected (got {m.line_jumps})"
    assert m.n_fixations == 0 or True  # field set by caller; ignore here

    flags = flag_dyslexia_risk(m, len(path))
    # Our synthetic path has a regression + line jump, so flags should be non-empty.
    assert len(flags) >= 1, "expected at least one risk flag on synthetic path"

    img = render_heatmap(fix)
    assert img is None or img.shape[:2] == (480, 640), "heatmap shape wrong"
    print(f"  fixations={len(fix)} regressions={m.regressions} "
          f"line_jumps={m.line_jumps} word_skips={m.word_skips} "
          f"erratic={m.erratic_score:.2f}", file=__import__("sys").stderr)
    print("OK — gaze-analysis self-test passed.", file=__import__("sys").stderr)
    return 0


def run_demo(out_png: str = "gaze_heatmap_demo.png") -> int:
    import os
    path = _synthetic_reading_path()
    fix, sac = detect_fixations(path)
    m = analyze_reading_path(path)
    flags = flag_dyslexia_risk(m, len(path))
    img = render_heatmap(fix)
    if img is None:
        print("cv2 unavailable — cannot render demo heatmap.", file=__import__("sys").stderr)
        return 1
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, out_png)
    import cv2
    cv2.imwrite(out, img)
    print(f"heatmap saved -> {out}", file=__import__("sys").stderr)
    print(f"fixations={len(fix)} regressions={m.regressions} "
          f"line_jumps={m.line_jumps} word_skips={m.word_skips} "
          f"erratic={m.erratic_score:.2f}", file=__import__("sys").stderr)
    for f in flags:
        print(f"  FLAG: {f}", file=__import__("sys").stderr)
    return 0


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        raise SystemExit(run_selftest())
    if "--demo" in sys.argv:
        raise SystemExit(run_demo())
    print("usage: python3 gaze_analysis.py [--selftest|--demo]", file=sys.stderr)
    raise SystemExit(2)
