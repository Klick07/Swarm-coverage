"""Dashboard: trajectories, coverage per area, notification feed, pending approvals.

  uvicorn dashboard.app:app --port 8000      (RUN_DIR=runs/<run> selects the run)

Approve/Reject only writes the human's answer; the coordinator applies it, so the
coverage record stays unchanged until a person has answered.
"""
import hmac
import json
import os
import sqlite3
import time
from pathlib import Path

from fastapi import FastAPI, Header, HTTPException
from fastapi.responses import FileResponse

RUN_DIR = os.environ.get("RUN_DIR", "runs/euroc3_r025_d")
APPROVAL_TOKEN = os.environ.get("APPROVAL_TOKEN", "")
app = FastAPI(title="Swarm-SLAM coordinator")


def db():
    con = sqlite3.connect(f"{RUN_DIR}/coverage.db", isolation_level=None)
    con.row_factory = sqlite3.Row
    return con


def rows(sql, *args):
    with db() as con:
        return [dict(r) for r in con.execute(sql, args)]


@app.get("/")
def index():
    return FileResponse(Path(__file__).with_name("index.html"))


@app.get("/api/state")
def state():
    places = rows("SELECT * FROM places")
    for p in places:
        p["cameras"] = json.loads(p["cameras"])
    trajectories = {r["robot"]: json.loads(r["points"]) for r in rows("SELECT * FROM trajectories")}
    decisions = rows("SELECT * FROM decisions WHERE action != 'none' ORDER BY id DESC LIMIT 40")
    for d in decisions:
        d["values"] = json.loads(d.pop("values_json") or "null")
    cameras = rows("SELECT * FROM cameras ORDER BY robot")
    for c in cameras:
        c["quality"] = json.loads(c["quality"])
        last = rows("SELECT message, t FROM notifications WHERE camera=? ORDER BY id DESC LIMIT 1", c["robot"])
        c["last_notification"] = last[0] if last else None
    return {
        "run": os.path.basename(RUN_DIR),
        "cameras": cameras,
        "places": places,
        "trajectories": trajectories,
        "notifications": rows("SELECT * FROM notifications ORDER BY id DESC LIMIT 40"),
        "pending": rows("SELECT * FROM overlaps WHERE state='pending' ORDER BY overlap_id"),
        "decisions": decisions,
        "counts": rows("SELECT state, COUNT(*) AS n FROM overlaps GROUP BY state"),
        "ignored": rows("SELECT COUNT(*) AS n FROM decisions WHERE trigger='overlap_rejected_by_swarm_slam'")[0]["n"],
    }


@app.get("/api/frame/{robot}")
def frame(robot: int):
    """The rectified left image (OpenCV 5 front end output) the coordinator is processing now."""
    r = rows("SELECT stamp_ns FROM cameras WHERE robot=?", robot)
    path = Path(f"{RUN_DIR}/frames/r{robot}/{r[0]['stamp_ns']}.jpg") if r else None
    if path is None or not path.exists():
        raise HTTPException(404, "no frame yet")
    return FileResponse(path, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/overlap/{overlap_id}/matches")
def overlap_matches(overlap_id: int):
    """Both keyframes side by side with the OpenCV 5 inlier matches the decision rests on."""
    path = Path(f"{RUN_DIR}/matches/overlap_{overlap_id}.jpg")
    if not path.exists():
        raise HTTPException(404, "no match image")
    return FileResponse(path, media_type="image/jpeg")


@app.post("/api/approve/{overlap_id}")
def approve(overlap_id: int, approve: bool, x_approval_token: str = Header(default="")):
    # The dashboard is public through the tunnel: viewing is open, answering needs the token.
    if APPROVAL_TOKEN and not hmac.compare_digest(x_approval_token, APPROVAL_TOKEN):
        raise HTTPException(403, "approval token required")
    with db() as con:
        cur = con.execute("UPDATE overlaps SET state=?, resolved_by='human' WHERE overlap_id=? AND state='pending'",
                          ("approved" if approve else "rejected", overlap_id))
    if cur.rowcount == 0:
        raise HTTPException(409, "overlap is not pending")
    return {"overlap_id": overlap_id, "state": "approved" if approve else "rejected", "t": time.time()}
