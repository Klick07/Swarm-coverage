"""Coverage records, overlaps, notifications and the decision log, in one SQLite file.

DynamoDB is blocked on this AWS account (see docs/FINDINGS.md item 6), so the spec's
coverage table lives here; the dashboard reads the same file.
"""
import json
import sqlite3
import time

SCHEMA = """
CREATE TABLE IF NOT EXISTS places (
  place_id TEXT PRIMARY KEY, cameras TEXT, verified_matches INTEGER, quality REAL,
  last_seen REAL, status TEXT);
CREATE TABLE IF NOT EXISTS overlaps (
  overlap_id INTEGER PRIMARY KEY AUTOINCREMENT, a INTEGER, ka INTEGER, b INTEGER, kb INTEGER,
  place_id TEXT, cslam_success INTEGER, cv_inliers INTEGER, cv_ratio REAL, cv_spread REAL, confidence TEXT,
  state TEXT, created REAL, resolved REAL, resolved_by TEXT);
CREATE TABLE IF NOT EXISTS notifications (
  id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL, camera INTEGER, message TEXT, overlap_id INTEGER);
CREATE TABLE IF NOT EXISTS decisions (
  id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL, camera INTEGER, trigger TEXT, values_json TEXT,
  decision TEXT, action TEXT, reason TEXT, source TEXT);
CREATE TABLE IF NOT EXISTS trajectories (robot INTEGER PRIMARY KEY, points TEXT, t REAL);
CREATE TABLE IF NOT EXISTS cameras (robot INTEGER PRIMARY KEY, stamp_ns INTEGER, quality TEXT, t REAL);
"""


class Store:
    def __init__(self, path):
        self.db = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)

    def rows(self, sql, *args):
        return [dict(r) for r in self.db.execute(sql, args)]

    # --- overlaps -------------------------------------------------------------
    def add_overlap(self, a, ka, b, kb, place_id, cslam_success):
        cur = self.db.execute(
            "INSERT INTO overlaps (a, ka, b, kb, place_id, cslam_success, state, created) VALUES (?,?,?,?,?,?,?,?)",
            (a, ka, b, kb, place_id, int(cslam_success), "new", time.time()))
        return cur.lastrowid

    def overlap(self, overlap_id):
        r = self.rows("SELECT * FROM overlaps WHERE overlap_id=?", overlap_id)
        return r[0] if r else None

    def set_overlap(self, overlap_id, **fields):
        sets = ", ".join(f"{k}=?" for k in fields)
        self.db.execute(f"UPDATE overlaps SET {sets} WHERE overlap_id=?", (*fields.values(), overlap_id))

    # --- coverage -------------------------------------------------------------
    def mark_covered(self, place_id, cameras, quality):
        old = self.rows("SELECT * FROM places WHERE place_id=?", place_id)
        cams = sorted(set(json.loads(old[0]["cameras"]) if old else []) | set(cameras))
        n = (old[0]["verified_matches"] if old else 0) + 1
        self.db.execute(
            "INSERT OR REPLACE INTO places VALUES (?,?,?,?,?,?)",
            (place_id, json.dumps(cams), n, quality, time.time(), "covered"))
        return {"place_id": place_id, "cameras": cams, "verified_matches": n}

    def see_place(self, place_id, camera, quality):
        """A keyframe landed in this place: record who has seen it, without claiming coverage."""
        old = self.rows("SELECT * FROM places WHERE place_id=?", place_id)
        if old:
            cams = sorted(set(json.loads(old[0]["cameras"])) | {camera})
            self.db.execute("UPDATE places SET cameras=?, quality=?, last_seen=? WHERE place_id=?",
                            (json.dumps(cams), quality, time.time(), place_id))
        else:
            self.db.execute("INSERT INTO places VALUES (?,?,?,?,?,?)",
                            (place_id, json.dumps([camera]), 0, quality, time.time(), "seen"))

    # --- outputs --------------------------------------------------------------
    def notify(self, camera, message, overlap_id=None):
        self.db.execute("INSERT INTO notifications (t, camera, message, overlap_id) VALUES (?,?,?,?)",
                        (time.time(), camera, message, overlap_id))

    def log_decision(self, camera, trigger, values, decision, action, reason, source):
        self.db.execute(
            "INSERT INTO decisions (t, camera, trigger, values_json, decision, action, reason, source) "
            "VALUES (?,?,?,?,?,?,?,?)",
            (time.time(), camera, trigger, json.dumps(values), decision, action, reason, source))

    def set_camera(self, robot, stamp_ns, quality):
        """The frame the coordinator is looking at now, for the dashboard's live camera tiles."""
        self.db.execute("INSERT OR REPLACE INTO cameras VALUES (?,?,?,?)",
                        (robot, stamp_ns, json.dumps(quality), time.time()))

    def set_trajectory(self, robot, points):
        self.db.execute("INSERT OR REPLACE INTO trajectories VALUES (?,?,?)",
                        (robot, json.dumps(points), time.time()))
