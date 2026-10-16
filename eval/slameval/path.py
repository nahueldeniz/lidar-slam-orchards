"""Distance travelled, heading along the ground-truth path and turn labels."""
from dataclasses import dataclass

import numpy as np


def wrap_angle(a):
    """Wrap angles to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class ArcLength:
    """Arc length along a noisy path.

    A vertex is accepted when the position is at least `min_step` from the
    previous vertex, so jitter while stationary does not add distance.
    s: (N,) distance travelled at every input sample (value of the last
    accepted vertex, so it is non-decreasing); s_v, p_v: the vertices, with
    strictly increasing s_v, used to look up positions by distance.
    """

    s: np.ndarray
    s_v: np.ndarray
    p_v: np.ndarray

    @property
    def total(self):
        return float(self.s_v[-1]) if len(self.s_v) else 0.0

    def position_at(self, s_query):
        s_query = np.asarray(s_query, dtype=float)
        return np.column_stack([np.interp(s_query, self.s_v, self.p_v[:, k]) for k in range(3)])


def arc_length(p, min_step):
    p = np.asarray(p, dtype=float)
    n = len(p)
    s = np.zeros(n)
    if n == 0:
        return ArcLength(s, np.zeros(0), np.zeros((0, 3)))
    verts = [0]
    s_v = [0.0]
    last = p[0]
    for i in range(1, n):
        d = np.linalg.norm(p[i] - last)
        if d >= min_step:
            s_v.append(s_v[-1] + d)
            verts.append(i)
            last = p[i]
        s[i] = s_v[-1]
    return ArcLength(s, np.asarray(s_v), p[verts])


def heading_at(arc: ArcLength, s_query, baseline):
    """Horizontal course (rad, ENU: 0 = east, counter-clockwise) at distance s,
    from positions baseline/2 behind and baseline/2 ahead along the path."""
    s_query = np.asarray(s_query, dtype=float)
    a = arc.position_at(s_query - baseline / 2.0)
    b = arc.position_at(s_query + baseline / 2.0)
    d = b - a
    psi = np.arctan2(d[:, 1], d[:, 0])
    psi[np.hypot(d[:, 0], d[:, 1]) < 1e-9] = np.nan
    return psi


def turn_flags(arc: ArcLength, s_query, baseline, window, threshold_deg):
    """True where the course changes by more than threshold over the window
    centred on s (heading at s - window/2 compared with s + window/2)."""
    s_query = np.asarray(s_query, dtype=float)
    before = heading_at(arc, s_query - window / 2.0, baseline)
    after = heading_at(arc, s_query + window / 2.0, baseline)
    change = np.abs(wrap_angle(after - before))
    return np.nan_to_num(change, nan=0.0) > np.deg2rad(threshold_deg)


def segment_class(turn_fraction, mixed_low, mixed_high):
    if turn_fraction < mixed_low:
        return "row"
    if turn_fraction > mixed_high:
        return "turn"
    return "mixed"
