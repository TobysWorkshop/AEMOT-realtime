"""
Offline tracklet stitching for the AEMOT output.

Given the per-track summary log (.beesum file) and the per-event log (.bees file),
this builds a start state and end state for every track, gates all time-ordered
pairs by Mahalanobis distance (under the same constant-velocity dynamics pulled straight
from the tracker's YAML file), and finds the globally-optimal stitching via the Hungarian
algorithm. Match chains are then chained into full stitched groups if more than two are joined
in a chain.
"""

import argparse
from dataclasses import dataclass
import os
import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2
from extract_tracks import read_log, read_summary_log

STATE_DIM = 4 ## we stitch on [x, y, vx, vy] only

@dataclass
class DynamicsConfig:
    """
    The subset of the tracker's YAML config needed to reproduce its constant-
    velocity F/Q and initial P0.
    """
    dt: float
    var_x: float
    var_y: float
    var_vx: float
    var_vy: float
    q_x: float
    q_y: float
    q_vx: float
    q_vy: float

    @classmethod
    def from_yaml(cls, path):
        with open(path, "r") as f:
            config = yaml.safe_load(f)
        try:
            return cls(
                dt=float(config["dt"]),
                var_x=float(config["var_x"]),
                var_y=float(config["var_y"]),
                var_vx=float(config["var_vx"]),
                var_vy=float(config["var_vy"]),
                q_x=float(config["q_x"]), q_y=float(config["q_y"]),
                q_vx=float(config["q_vx"]), q_vy=float(config["q_vy"]),
            )
        except KeyError as e:
            raise KeyError(
                f"Config at {path} is missing expected key {e}."
                f"This loader assumes flat top-level keys matching parameters"
                f"(dt, var_x, var_y, var_vx, var_vy, q_x, q_y, q_vx, q_vy)."
            ) from e

    def P0(self):
        return np.diag([self.var_x, self.var_y, self.var_vx, self.var_vy])

    def Q(self, gap_seconds):
        scale = max(gap_seconds, 0.0) / self.dt
        return np.diag([self.q_x, self.q_y, self.q_vx, self.q_vy]) * scale


def build_F(dt):
    """
    Constant-velcity state transition for arbitrary elapsed time dt,
    taking the general structure of F in processing.cpp::setup() -
    (F(j*n_state,2)=dt for x+=vx*dt, F(j*n_state+1,3)=dt for y+=vy*dt)
    """
    return np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=float)

def extract_track_endpoints(summary_data, event_data, config: DynamicsConfig):
    """
    Builds for every track in summary_data:
        start: (t_created, x_start[4], P_start = config.P0())
        end: (t_deleted, x_end[4], P_end = summary's P_at_deletion, sliced to 4x4)
    x_start comes from that track's earliest record in the dense per-event log (event_data).
    matched by track_id.
    """
    endpoints = {}
    for rec in summary_data:
        tid = int(rec["track_id"])
        mask = event_data["track_id"] == tid
        if not np.any(mask):
            #raise ValueError("Warning! Mask was empty!! This shouldnt happen for validated tracks...")
            continue
        track_rows = event_data[mask]
        first_row = track_rows[np.argmin(track_rows["ts"])]

        x_start = first_row["x_hat"][:STATE_DIM].astype(float)
        P_start = config.P0()
        t_start = float(first_row["ts"])

        x_end = rec["x_hat_at_deletion"][:STATE_DIM].astype(float)
        P_full_end = rec["P_at_deletion"].reshape(8, 8)
        P_end = P_full_end[:STATE_DIM, :STATE_DIM]
        t_end = float(rec["t_deleted"])

        endpoints[tid] = {
            "t_start": t_start, "x_start": x_start, "P_start": P_start,
            "t_end": t_end, "x_end": x_end, "P_end": P_end,
        }
    return endpoints

def mahalanobis_sq(residual, S):
    try:
        solved = np.linalg.solve(S, residual)
    except np.linalg.LinAlgError:
        solved = np.linalg.pinv(S) @ residual
    return float(residual @ solved)

def build_cost_matrix(endpoints, config: DynamicsConfig, max_gap_seconds, confidence=0.99):
    track_ids = sorted(endpoints.keys())
    n = len(track_ids)
    cost = np.full((n, n), np.inf)
    threshold = chi2.ppf(confidence, df=STATE_DIM)

    for i, tid_a in enumerate(track_ids):
        a = endpoints[tid_a]
        for j, tid_b in enumerate(track_ids):
            if tid_a == tid_b:
                continue # don't compare a tracklet to itself
            b = endpoints[tid_b]
            gap = b["t_start"] - a["t_end"]
            if gap <= 0 or gap > max_gap_seconds:
                continue # B doesnt start after A ends, or the gap is too large to bridge

            F = build_F(gap)
            x_pred = F @ a["x_end"]
            P_pred = F @ a["P_end"] @ F.T + config.Q(gap)

            residual = b["x_start"] - x_pred
            S = P_pred + b["P_start"]
            d2 = mahalanobis_sq(residual, S)

            if d2 <= threshold:
                cost[i, j] = d2

    return track_ids, cost

def solve_assignment(track_ids, cost):
    """
    Runs the Hungarian assignment over the (possibly mostly inf) cost
    matrix and returns {predecessor_track_id: successor_track_id} for only
    the pairs that were actually within the gate (finite cost).

    The solver is forced to produce a full assignment even where every option is inf,
    so these need to be filtered out afterwards rather than trustinf the output as real matches.
    """

    n = len(track_ids)
    finite_mask = np.isfinite(cost)
    sentinel = (cost[finite_mask].max() if finite_mask.any() else 1.0) + 1e6
    cost_finite = np.where(finite_mask, cost, sentinel)

    row_idx, col_idx = linear_sum_assignment(cost_finite)

    matches = {}
    for r, c in zip(row_idx, col_idx):
        if finite_mask[r, c]:
            matches[track_ids[r]] = track_ids[c]
    return matches

def chain_matches(track_ids, matches):
    """
    Follows predecessor->successor links to build full stitched groups.
    Returns (groups, track_to_group) where groups is a list of ordered
    track_id lists (each a full stitched track), and track_to_group maps
    every track_id (stitched or standalone) to a group id (the first track_id
    in its chain).
    """
    successors = dict(matches) # a -> b
    predecessors = {b: a for a, b in matches.items()} # b -> a

    groups = []
    track_to_group = {}
    visited = set()

    # start each chain from a track with no predecessor, so every chain is
    # walked exactly once, start to finish
    chain_starts = [tid for tid in track_ids if tid not in predecessors]
    for start in chain_starts:
        if start in visited:
            continue
        chain = [start]
        visited.add(start)
        cur = start
        while cur in successors:
            cur = successors[cur]
            if cur in visited:
                break # shouldn't happen, but guard anyways
            chain.append(cur)
            visited.add(cur)
        groups.append(chain)
        for tid in chain:
            track_to_group[tid] = start

    return groups, track_to_group


def stitch_tracks(summary_data, event_data, config_path, max_gap_seconds=1.0, confidence=0.99):
    """
    Full stitching pipeline. 
    """

    config = DynamicsConfig.from_yaml(config_path)
    endpoints = extract_track_endpoints(summary_data, event_data, config)
    track_ids, cost = build_cost_matrix(endpoints, config, max_gap_seconds, confidence)
    matches = solve_assignment(track_ids, cost)
    groups, track_to_group = chain_matches(track_ids, matches)

    return groups, track_to_group

## entry point
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", help="path to the .bees per-event log")
    parser.add_argument("summary_path", help="path to the .beesum track summary log")
    parser.add_argument("config_name", help="name of the tracker's YAML config in the config/ folder (without .yaml !)")
    parser.add_argument("--max_gap_seconds", type=float, default=1.0)
    parser.add_argument("--confidence", type=float, default=0.99)
    args = parser.parse_args()

    event_data = read_log(args.log_path)
    summary_data, _ = read_summary_log(args.summary_path)

    this_file_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(this_file_dir, '..', 'configs', args.config_name + ".yaml")
    config_path = os.path.normpath(config_path)

    groups, track_to_group = stitch_tracks(
        summary_data, event_data, config_path,
        max_gap_seconds=args.max_gap_seconds, confidence=args.confidence,
    )

    multi = [g for g in groups if len(g) > 1]
    print(f"{len(summary_data)} tracks -> {len(groups)} groups"
          f"({len(multi)} actually stitched, {len(groups) - len(multi)} standalone)")
    for g in multi:
        print("  " + " -> ".join(str(t) for t in g))