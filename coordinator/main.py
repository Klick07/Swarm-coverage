"""Coordinator: events.jsonl -> decisions, notifications and coverage records.

  python -m coordinator.main <run_dir> [--speed 4] [--follow] [--rules-only]

--speed replays a recorded run faster than real time; --follow tails a live run.
Bedrock decides by default; --rules-only (or a Bedrock failure) uses the same rules
in code, and the decision log says which one acted.
"""
import argparse
import json
import os
import statistics
import time
from collections import deque

import cv2

from coordinator.store import Store
from coordinator.verify import verify_pair

# Weak tracking = tracking inliers < WEAK_TRACK, or almost no features (a blank wall leaves nothing
# to track). Blur is shown but no longer triggers: tuned at 90 in the bright Machine Hall, it flagged
# 71-83% of normal frames in the darker Vicon Room (median blur 48-59) and caused 52 warnings there vs
# 14 for tracking alone, while both rules caught both injected failures (eval/quality_rules.py).
WEAK_BLUR = 90.0          # informational only (Machine Hall ~5th percentile)
WEAK_TRACK = 0.50         # between the 1st and 5th percentile of track_inliers (runs c, v1)
MIN_FEATURES = 100
WINDOW = 6                # frames (~2.4 s at 2.5 fps)
QUALITY_COOLDOWN_S = 20   # per camera, in data time
CELL_M = 2.0              # place = 2 m grid cell on the merged map's ground plane (x, z)
# OpenCV 5 re-verification: below this an overlap needs a human. Tuned on run euroc3_r025_d:
# keeps 83% of Swarm-SLAM-accepted closures (median 210 inliers), passes 0.7% of 150 random
# cross-robot pairs >5 m apart (median 12).
MIN_INLIERS = 50
# Failure case "similar-looking places" (eval/similar_places.py, run d): the 6 far-apart pairs
# (>5 m in ground truth) that look most alike all had 51-99 inliers, but their matches crowded
# into one patch (same object seen from far away). Requiring inliers to span >= 10% of each image
# keeps 19/20 true overlaps with >= 50 inliers and stops 4/6 of those look-alikes.
MIN_SPREAD = 0.10
# SLAM tracking lost while the images look fine (failure run euroc3_fail_e: after the blank
# wall the frames were sharp again but RTAB-Map odometry never recovered). Clean runs have
# at most 21 good frames between Swarm-SLAM keyframes; the failure run had 372-491.
LOST_FRAMES = 30


class Coordinator:
    def __init__(self, run_dir, use_bedrock=True, model_id=None):
        self.run = run_dir
        self.store = Store(f"{run_dir}/coverage.db")
        self.kf_stamp, self.kf_pos, self.placed = {}, {}, set()
        self.window = {r: deque(maxlen=WINDOW) for r in range(3)}
        self.last_alert = {}
        self.since_kf = {r: 0 for r in range(3)}   # good frames since this robot's last keyframe
        self.last_kf = {}
        self.lost = set()
        self.seen_lc = set()
        self.ctx = {}
        self.agent = None
        if use_bedrock:
            from coordinator.agent import Agent
            self.agent = Agent(self.tool_table(), **({"model_id": model_id} if model_id else {}))

    # --- tools (shared by Bedrock and the rule fallback) ----------------------
    def tool_table(self):
        return {"get_quality": self.get_quality, "get_overlaps": self.get_overlaps,
                "verify_overlap": self.verify_overlap, "mark_covered": self.mark_covered,
                "request_approval": self.request_approval, "notify": self.notify,
                "log_decision": self.log_decision}

    def _log(self, camera, decision, action, reason=""):
        self.store.log_decision(camera, self.ctx.get("trigger"), self.ctx.get("values"),
                                decision, action, reason, self.ctx.get("source"))

    def get_quality(self, camera):
        w = list(self.window.get(int(camera), []))
        if not w:
            return {"camera": camera, "frames": 0}
        def med(k):  # track_inliers is None on a camera's first frame
            vals = [x[k] for x in w if x[k] is not None]
            return round(statistics.median(vals), 3) if vals else None
        blur, track, features = med("blur"), med("track_inliers"), med("features")
        return {"camera": camera, "frames": len(w), "blur": blur, "features": features,
                "track_inliers": track, "stereo_inliers": med("stereo_inliers"),
                "weak": features < MIN_FEATURES or (track is not None and track < WEAK_TRACK)}

    def get_overlaps(self):
        return {"overlaps": self.store.rows(
            "SELECT overlap_id, a, b, place_id, cv_inliers, confidence, state FROM overlaps "
            "ORDER BY overlap_id DESC LIMIT 10")}

    def verify_overlap(self, overlap_id):
        o = self.store.overlap(int(overlap_id))
        if o is None:
            return {"error": "no such overlap"}
        imgs = [self._keyframe_image(o["a"], o["ka"]), self._keyframe_image(o["b"], o["kb"])]
        if any(i is None for i in imgs):
            res = {"matches": 0, "inliers": 0, "inlier_ratio": 0.0, "note": "keyframe image missing"}
        else:
            os.makedirs(f"{self.run}/matches", exist_ok=True)
            res = verify_pair(*imgs, draw_path=f"{self.run}/matches/overlap_{o['overlap_id']}.jpg")
        res["confidence"] = "high" if res["inliers"] >= MIN_INLIERS and res["spread"] >= MIN_SPREAD else "low"
        res["min_inliers"], res["min_spread"] = MIN_INLIERS, MIN_SPREAD
        # The model once cited invented inlier-ratio thresholds; state the actual rule.
        res["confidence_rule"] = (f"high if inliers >= {MIN_INLIERS} and spread >= {MIN_SPREAD}; "
                                  "inlier_ratio is informational only")
        # Explicit roles: in replay euroc3_r025_d the model notified the lower-numbered camera in
        # 2 of 24 overlaps where camera 2 arrived first. The facts now come from the tool.
        res["arrived_first"], res["arrived_second"] = o["a"], o["b"]
        res["if_covered_notify"] = {"camera": o["b"],
                                    "message": f"Area covered by camera {o['a']}, camera {o['b']} move on."}
        self.store.set_overlap(o["overlap_id"], cv_inliers=res["inliers"], cv_ratio=res["inlier_ratio"],
                               cv_spread=res["spread"], confidence=res["confidence"])
        self.ctx["values"] = {**self.ctx.get("values", {}), "opencv": res}
        return res

    def mark_covered(self, overlap_id):
        o = self.store.overlap(int(overlap_id))
        if o is None:
            return {"error": "no such overlap"}
        if o["confidence"] != "high" and o["state"] != "approved":
            self._log(o["b"], "refused mark_covered", "blocked: needs human approval")
            return {"refused": True, "reason": "low-confidence overlap needs human approval first"}
        rec = self.store.mark_covered(o["place_id"], [o["a"], o["b"]], self._place_quality(o))
        self.store.set_overlap(o["overlap_id"], state="applied", resolved=time.time())
        self._log(o["b"], f"coverage updated for {o['place_id']}", "mark_covered")
        return rec

    def request_approval(self, overlap_id, reason):
        o = self.store.overlap(int(overlap_id))
        if o is None:
            return {"error": "no such overlap"}
        self.store.set_overlap(o["overlap_id"], state="pending")
        self._log(o["b"], "approval requested", "request_approval", reason)
        return {"pending": True, "overlap_id": o["overlap_id"]}

    def notify(self, camera, message):
        if "t_event" in self.ctx:  # measured decision latency: event handled -> notification
            self.ctx["values"] = {**self.ctx.get("values", {}),
                                  "decision_latency_s": round(time.time() - self.ctx["t_event"], 3)}
        self.store.notify(int(camera), message, self.ctx.get("overlap_id"))
        self._log(int(camera), "notify", f"notify: {message}")
        print(f"  -> camera {camera}: {message}")
        return {"sent": True}

    def log_decision(self, camera, decision, reason):
        self._log(int(camera), decision, "log_decision", reason)
        return {"logged": True}

    # --- helpers ---------------------------------------------------------------
    def _keyframe_image(self, robot, kf):
        stamp = self.kf_stamp.get((robot, kf))
        path = next((f"{self.run}/frames/r{robot}/{stamp}{e}" for e in (".png", ".jpg")
                     if os.path.exists(f"{self.run}/frames/r{robot}/{stamp}{e}")), None)
        return cv2.imread(path, cv2.IMREAD_GRAYSCALE) if stamp and os.path.exists(path) else None

    def _place(self, robot, kf):
        p = self.kf_pos.get((robot, kf))
        return f"cell_{int(p[0] // CELL_M)}_{int(p[2] // CELL_M)}" if p else f"unplaced_r{robot}_kf{kf}"

    def _place_quality(self, o):
        q = [self.get_quality(c).get("blur") for c in (o["a"], o["b"])]
        return min(v for v in q if v is not None) if any(v is not None for v in q) else None

    # --- event handling ---------------------------------------------------------
    def on_event(self, ev):
        kind = ev["type"]
        if kind == "quality":
            self._on_quality(ev)
        elif kind == "keyframe":
            self.kf_stamp[(ev["robot"], ev["kf"])] = ev["stamp_ns"]
            self._on_keyframe(ev["robot"], ev["kf"])
        elif kind == "poses":
            self._on_poses(ev)
        elif kind == "lc":
            self._on_lc(ev)
        else:
            return
        self._apply_approvals()

    def _on_poses(self, ev):
        r = ev["robot"]
        for kf, x, y, z in ev["poses"]:
            self.kf_pos[(r, kf)] = (x, y, z)
        self.store.set_trajectory(r, [[x, z] for _, x, _, z in ev["poses"]])
        q = self.get_quality(r).get("blur")
        for kf, *_ in ev["poses"]:
            if (r, kf) not in self.placed:
                self.placed.add((r, kf))
                self.store.see_place(self._place(r, kf), r, q)

    def _on_keyframe(self, r, kf):
        self.since_kf[r] = 0
        self.last_kf[r] = kf
        if r in self.lost:
            self.lost.discard(r)
            self.ctx = {"trigger": "tracking_recovered", "values": {"robot": r, "keyframe": kf}, "overlap_id": None}
            self._decide(r, f"Tracking-recovered event: camera {r} is producing Swarm-SLAM keyframes again "
                            f"(keyframe {kf}).",
                         lambda: self.notify(r, f"Camera {r}, tracking recovered, carry on."))

    def _check_lost(self, r, q):
        if r in self.lost or r not in self.last_kf or self.since_kf[r] < LOST_FRAMES:
            return
        self.lost.add(r)
        place = self._place(r, self.last_kf[r])
        self.ctx = {"trigger": "slam_tracking_lost", "values": {
            "opencv": q, "frames_without_keyframe": self.since_kf[r], "last_keyframe": self.last_kf[r],
            "last_place": place}, "overlap_id": None}
        self._decide(r, f"Tracking-lost event: camera {r} has sent {self.since_kf[r]} good frames (threshold "
                        f"{LOST_FRAMES}) but Swarm-SLAM made no new keyframe, so its odometry is lost even though "
                        f"OpenCV image quality is blur {q['blur']}, features {q['features']}. Its last mapped "
                        f"place was {place}.",
                     lambda: self.notify(r, f"Camera {r}, tracking lost: walk back to your last mapped area "
                                            f"({place}) and hold still."))

    def _on_quality(self, ev):
        r = ev["robot"]
        self.window[r].append(ev)
        q = self.get_quality(r)
        if ev.get("passed", True):  # only passed frames are saved as JPEGs by the front end
            self.store.set_camera(r, ev["stamp_ns"], q)
            self.since_kf[r] += 1
            self._check_lost(r, q)
        t = ev["stamp_ns"] / 1e9
        if len(self.window[r]) < WINDOW or not q["weak"] or t - self.last_alert.get(r, -1e9) < QUALITY_COOLDOWN_S:
            return
        self.last_alert[r] = t
        self.ctx = {"trigger": "quality_drop", "values": {"opencv": q, "frame_stamp_ns": ev["stamp_ns"]},
                    "overlap_id": None}
        self._decide(r, f"Quality event: camera {r} OpenCV 5 window medians over {WINDOW} frames: "
                        f"track_inliers {q['track_inliers']} (weak below {WEAK_TRACK}), features {q['features']} "
                        f"(weak below {MIN_FEATURES}), blur {q['blur']} (informational).",
                     lambda: self.notify(r, f"Camera {r}, slow down, tracking is weak."))

    def _on_lc(self, ev):
        key = (ev["a"], ev["ka"], ev["b"], ev["kb"])
        if key in self.seen_lc:
            return
        self.seen_lc.add(key)
        a, b = ev["a"], ev["b"]
        if not ev["success"]:  # Swarm-SLAM's own geometric check failed: nothing to act on, but keep the trace
            self.ctx = {"trigger": "overlap_rejected_by_swarm_slam", "values": {"swarm_slam": ev}, "source": "rules"}
            self._log(b, "ignored overlap", "none", "Swarm-SLAM PnP verification failed")
            return
        oid = self.store.add_overlap(a, ev["ka"], b, ev["kb"], self._place(a, ev["ka"]), True)
        self.ctx = {"trigger": "overlap", "values": {"swarm_slam": ev}, "overlap_id": oid}
        print(f"overlap #{oid}: camera {a} kf {ev['ka']} <-> camera {b} kf {ev['kb']}")
        self._decide(b, f"Overlap event: overlap_id {oid}. Swarm-SLAM verified an inter-robot loop closure "
                        f"between camera {a} (keyframe {ev['ka']}) and camera {b} (keyframe {ev['kb']}), "
                        f"place {self._place(a, ev['ka'])}. Camera {b} arrived second.",
                     lambda: self._overlap_rules(oid, a, b))

    def _overlap_rules(self, oid, a, b):
        res = self.verify_overlap(oid)
        if res["confidence"] == "high":
            self.mark_covered(oid)
            self.notify(b, f"Area covered by camera {a}, camera {b} move on.")
        else:
            self.request_approval(oid, f"OpenCV 5 inliers {res['inliers']} (need {MIN_INLIERS}), "
                                       f"spread {res['spread']} (need {MIN_SPREAD})")

    def _decide(self, camera, text, rules):
        self.ctx["t_event"] = time.time()
        if self.agent:
            self.ctx["source"] = "bedrock"
            try:
                trace = self.agent.handle(text)
                if trace:
                    return
                self._log(camera, "model made no tool call", "fallback to rules")
            except Exception as e:
                self._log(camera, "bedrock error", "fallback to rules", str(e)[:200])
        self.ctx["source"] = "rules"
        rules()

    def _apply_approvals(self):
        """A human answered on the dashboard: apply or drop the blocked coverage change."""
        for o in self.store.rows("SELECT * FROM overlaps WHERE state IN ('approved', 'rejected') AND resolved IS NULL"):
            self.ctx = {"trigger": "human_answer", "values": {"overlap": o}, "overlap_id": o["overlap_id"],
                        "source": "human"}
            if o["state"] == "approved":
                self.mark_covered(o["overlap_id"])
                self.notify(o["b"], f"Area covered by camera {o['a']}, camera {o['b']} move on.")
            else:
                self.store.set_overlap(o["overlap_id"], resolved=time.time())
                self._log(o["b"], "overlap rejected by human", "coverage unchanged")


def events(path, speed, follow):
    while follow and not os.path.exists(path):   # job runs: the recorder creates the file after the coordinator starts
        time.sleep(0.5)
    with open(path) as f:
        prev = None
        while True:
            line = f.readline()
            if not line:
                if not follow:
                    return
                time.sleep(0.2)
                continue
            ev = json.loads(line)
            if prev is not None and speed > 0:
                time.sleep(max(0.0, (ev["t"] - prev) / speed))
            prev = ev["t"]
            yield ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--speed", type=float, default=0, help="0 = as fast as possible")
    ap.add_argument("--follow", action="store_true")
    ap.add_argument("--rules-only", action="store_true")
    ap.add_argument("--model-id")
    ap.add_argument("--wait-approvals", action="store_true")
    ap.add_argument("--approvals-only", action="store_true", help="don't replay; only apply human answers")
    args = ap.parse_args()
    c = Coordinator(args.run_dir, use_bedrock=not args.rules_only, model_id=args.model_id)
    for ev in ([] if args.approvals_only else events(f"{args.run_dir}/events.jsonl", args.speed, args.follow)):
        c.on_event(ev)
    if c.agent:
        print("bedrock usage:", c.agent.usage)
    while args.wait_approvals:  # keep applying the dashboard's answers after the replay ends
        c._apply_approvals()
        time.sleep(1)


if __name__ == "__main__":
    main()
