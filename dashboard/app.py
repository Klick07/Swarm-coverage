"""Dashboard: trajectories, coverage per area, notification feed, pending approvals.

  uvicorn dashboard.app:app --port 8000      (RUN_DIR=runs/<run> selects the run)

Approve/Reject only writes the human's answer; the coordinator applies it, so the
coverage record stays unchanged until a person has answered.
"""
import json
import os
import sqlite3
import time
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse

RUN_DIR = os.environ.get("RUN_DIR", "runs/euroc3_r025_d")
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
    return {
        "run": os.path.basename(RUN_DIR),
        "places": places,
        "trajectories": trajectories,
        "notifications": rows("SELECT * FROM notifications ORDER BY id DESC LIMIT 40"),
        "pending": rows("SELECT * FROM overlaps WHERE state='pending' ORDER BY overlap_id"),
        "decisions": decisions,
        "counts": rows("SELECT state, COUNT(*) AS n FROM overlaps GROUP BY state"),
        "ignored": rows("SELECT COUNT(*) AS n FROM decisions WHERE trigger='overlap_rejected_by_swarm_slam'")[0]["n"],
    }


@app.post("/api/approve/{overlap_id}")
def approve(overlap_id: int, approve: bool):
    with db() as con:
        cur = con.execute("UPDATE overlaps SET state=?, resolved_by='human' WHERE overlap_id=? AND state='pending'",
                          ("approved" if approve else "rejected", overlap_id))
    if cur.rowcount == 0:
        raise HTTPException(409, "overlap is not pending")
    return {"overlap_id": overlap_id, "state": "approved" if approve else "rejected", "t": time.time()}
