"""Dashboard: camera views, coverage per area, notifications, approvals, uploads, devices.

  uvicorn dashboard.app:app --port 8000      (RUN_DIR=runs/<run> selects the run)

Approve/Reject only writes the human's answer; the coordinator applies it, so the
coverage record stays unchanged until a person has answered.
"""
import hmac
import json
import os
import re
import shutil
import sqlite3
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml
from fastapi import FastAPI, Form, Header, HTTPException, UploadFile
from fastapi.responses import FileResponse

from dashboard.uploads import analyze_video, write_meta

RUN_DIR = os.environ.get("RUN_DIR", "runs/euroc3_r025_d")
APPROVAL_TOKEN = os.environ.get("APPROVAL_TOKEN", "")
UPLOAD_DIR = Path(os.environ.get("UPLOAD_DIR", os.path.expanduser("~/uploads")))
# Cloudflare's free quick tunnel rejects request bodies over 100 MB.
MAX_UPLOAD_BYTES = 95 * 1024 * 1024
MIN_FREE_BYTES_AFTER = 1024 ** 3      # the EC2 disk is small; keep 1 GB free for runs
VIDEO_TYPES = {".mp4", ".mov", ".avi", ".mkv", ".m4v", ".webm"}
ANALYSIS = ThreadPoolExecutor(max_workers=1)   # one video at a time on 2 vCPU
UPLOAD_ID = re.compile(r"^[0-9a-f]{12}$")
SESSION_ID = re.compile(r"^[0-9a-f]{8}$")


def run_sequences(run_dir):
    """EuRoC sequence per camera from the run's params.txt (older runs: Machine Hall 01-03)."""
    try:
        for tok in open(f"{run_dir}/params.txt").read().split():
            if tok.startswith("seqs="):
                return tok[5:].split(",")
    except FileNotFoundError:
        pass
    return ["MH_01_easy", "MH_02_easy", "MH_03_medium"]


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
    # no-store: a browser once kept a cached copy from before the approval images were added.
    return FileResponse(Path(__file__).with_name("index.html"), headers={"Cache-Control": "no-store"})


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
        "sequences": run_sequences(RUN_DIR),
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


def require_token(token):
    # The dashboard is public through the tunnel: viewing is open, acting needs the operator token.
    if APPROVAL_TOKEN and not hmac.compare_digest(token, APPROVAL_TOKEN):
        raise HTTPException(403, "operator token required")


def upload_dir(upload_id):
    if not UPLOAD_ID.match(upload_id):
        raise HTTPException(404, "no such upload")
    d = UPLOAD_DIR / upload_id
    if not (d / "meta.json").exists():
        raise HTTPException(404, "no such upload")
    return d


@app.post("/api/uploads", status_code=201)
async def upload(file: UploadFile, camera: int | None = Form(None), session: str | None = Form(None),
                 x_approval_token: str = Header(default="")):
    """Store an uploaded video and queue its OpenCV 5 quality analysis.

    camera (0-2) says which walker it belongs to; session groups the videos uploaded together
    (one per camera). Each upload is its own request, so the 95 MB limit is per video.
    """
    require_token(x_approval_token)
    if camera is not None and camera not in (0, 1, 2):
        raise HTTPException(422, "camera must be 0, 1 or 2")
    if session is not None and not SESSION_ID.match(session):
        raise HTTPException(422, "bad session id")
    name = os.path.basename(file.filename or "video")
    ext = os.path.splitext(name)[1].lower()
    if ext not in VIDEO_TYPES:
        raise HTTPException(415, f"unsupported file type {ext or '(none)'}; use {', '.join(sorted(VIDEO_TYPES))}")
    UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(UPLOAD_DIR).free - MAX_UPLOAD_BYTES < MIN_FREE_BYTES_AFTER:
        raise HTTPException(507, "not enough disk space on the server for another upload")
    upload_id = uuid.uuid4().hex[:12]
    d = UPLOAD_DIR / upload_id
    d.mkdir()
    size, path = 0, d / f"video{ext}"
    try:
        with open(path, "wb") as out:
            while chunk := await file.read(1024 * 1024):
                size += len(chunk)
                if size > MAX_UPLOAD_BYTES:
                    raise HTTPException(413, f"file is larger than {MAX_UPLOAD_BYTES // 2**20} MB")
                out.write(chunk)
    except HTTPException:
        shutil.rmtree(d, ignore_errors=True)
        raise
    meta = write_meta(str(d), id=upload_id, name=name[:120], size_bytes=size, created=time.time(), status="queued",
                      camera=camera, session=session or upload_id[:8])
    ANALYSIS.submit(analyze_video, str(path), str(d))
    return meta


@app.get("/api/uploads")
def list_uploads():
    if not UPLOAD_DIR.exists():
        return []
    metas = []
    for meta_path in UPLOAD_DIR.glob("*/meta.json"):
        try:
            m = json.load(open(meta_path))
        except (OSError, ValueError):
            continue  # being written
        m.pop("timeline", None)
        metas.append(m)
    return sorted(metas, key=lambda m: m.get("created", 0), reverse=True)


@app.get("/api/uploads/{upload_id}")
def upload_detail(upload_id: str):
    return json.load(open(upload_dir(upload_id) / "meta.json"))


@app.get("/api/uploads/{upload_id}/thumb")
def upload_thumb(upload_id: str):
    path = upload_dir(upload_id) / "thumb.jpg"
    if not path.exists():
        raise HTTPException(404, "no thumbnail yet")
    return FileResponse(path, media_type="image/jpeg")


@app.delete("/api/uploads/{upload_id}")
def delete_upload(upload_id: str, x_approval_token: str = Header(default="")):
    require_token(x_approval_token)
    d = upload_dir(upload_id)
    if json.load(open(d / "meta.json")).get("status") in ("queued", "processing"):
        raise HTTPException(409, "still being analyzed")
    shutil.rmtree(d)
    return {"deleted": upload_id}


@app.get("/api/devices")
def devices():
    """Wi-Fi camera agents from config/cameras.yaml. Live connection is not built yet."""
    cfg_path = Path(__file__).resolve().parent.parent / "config" / "cameras.yaml"
    cfg = yaml.safe_load(open(cfg_path)) if cfg_path.exists() else {"cameras": []}
    d = cfg.get("defaults", {})
    return [{"id": c["id"], "url": c.get("url", ""), "layout": d.get("layout"), "width": d.get("width"),
             "height": d.get("height"), "fps": d.get("fps"), "status": "not connected", "assumed": True}
            for c in cfg.get("cameras", [])]


@app.post("/api/approve/{overlap_id}")
def approve(overlap_id: int, approve: bool, x_approval_token: str = Header(default="")):
    require_token(x_approval_token)
    with db() as con:
        cur = con.execute("UPDATE overlaps SET state=?, resolved_by='human' WHERE overlap_id=? AND state='pending'",
                          ("approved" if approve else "rejected", overlap_id))
    if cur.rowcount == 0:
        raise HTTPException(409, "overlap is not pending")
    return {"overlap_id": overlap_id, "state": "approved" if approve else "rejected", "t": time.time()}
