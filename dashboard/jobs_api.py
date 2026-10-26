"""HTTP API for upload-driven mapping jobs (see jobs/runner.py). Mounted by dashboard/app.py.

Reading is open (like the rest of the dashboard); everything that changes state needs the operator token.
"""
import os
from concurrent.futures import ThreadPoolExecutor

from fastapi import APIRouter, Header, HTTPException, Request

from jobs import runner

INGEST = ThreadPoolExecutor(max_workers=1)   # decoding video is CPU heavy; one at a time


def build_router(require_token, select_run, default_run):  # default_run: callable returning the saved demo path
    r = APIRouter()

    def guard(fn, *a, **k):
        try:
            return fn(*a, **k)
        except runner.JobError as e:
            raise HTTPException(409, str(e))

    @r.post("/api/jobs", status_code=201)
    def create(n_robots: int = 3, x_approval_token: str = Header(default="")):
        require_token(x_approval_token)
        return guard(lambda: runner.read_meta(runner.create_job(n_robots)))

    @r.get("/api/jobs")
    def list_jobs():
        out = []
        if runner.JOBS.exists():
            for d in sorted(runner.JOBS.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)[:20]:
                if (d / "meta.json").exists():
                    out.append(guard(runner.status, d.name))
        return out

    @r.get("/api/jobs/{job_id}")
    def get(job_id: str):
        return guard(runner.status, job_id)

    @r.post("/api/jobs/{job_id}/files/{slot}")
    async def chunk(job_id: str, slot: str, request: Request, offset: int, total: int,
                    x_approval_token: str = Header(default="")):
        require_token(x_approval_token)
        data = await request.body()
        if len(data) > 95 * 1024 * 1024:
            raise HTTPException(413, "chunk larger than 95 MB; send smaller pieces")
        return {"stored": guard(runner.save_chunk, job_id, slot, offset, total, data), "total": total}

    @r.post("/api/jobs/{job_id}/ingest/{robot}", status_code=202)
    def ingest(job_id: str, robot: int, swap_eyes: bool = False, calib_swap_eyes: bool = False,
               target_hz: float = 10.0, max_seconds: float | None = None, x_approval_token: str = Header(default="")):
        require_token(x_approval_token)
        meta = guard(runner.read_meta, job_id)
        if meta.get("status") == "running":
            raise HTTPException(409, "job is running")
        if not 0 <= robot < meta["n_robots"] or not 1 <= target_hz <= 30:
            raise HTTPException(422, "bad robot or target_hz (1-30)")
        if swap_eyes:   # exchange the two uploaded videos for this robot
            inc = runner.jdir(job_id) / "incoming"
            a, b, t = inc / f"r{robot}_left", inc / f"r{robot}_right", inc / f"r{robot}_tmp"
            if a.exists() and b.exists():
                a.rename(t); b.rename(a); t.rename(b)
        robots = meta["robots"]; robots[str(robot)] = {"state": "queued"}
        runner.write_meta(job_id, robots=robots)
        INGEST.submit(runner.ingest_robot, job_id, robot, False, calib_swap_eyes, target_hz, max_seconds)
        return {"queued": robot}

    @r.post("/api/jobs/{job_id}/start")
    def start(job_id: str, rate: float = 0.25, max_wall_s: int | None = None, x_approval_token: str = Header(default="")):
        require_token(x_approval_token)
        if max_wall_s is not None and not 60 <= max_wall_s <= 3600:
            raise HTTPException(422, "max_wall_s must be 60-3600")
        meta = guard(runner.start, job_id, rate, max_wall_s)
        select_run(str(runner.RUNS / meta["run"]), True)
        return runner.status(job_id)

    @r.post("/api/jobs/{job_id}/stop")
    def stop(job_id: str, x_approval_token: str = Header(default="")):
        require_token(x_approval_token)
        return guard(runner.stop, job_id)

    @r.post("/api/jobs/{job_id}/show")
    def show(job_id: str, x_approval_token: str = Header(default="")):
        require_token(x_approval_token)
        meta = guard(runner.read_meta, job_id)
        if not meta.get("run") or not (runner.RUNS / meta["run"]).exists():
            raise HTTPException(409, "this job has no run output yet")
        select_run(str(runner.RUNS / meta["run"]), True)
        return {"showing": meta["run"], "live": True}

    @r.post("/api/runs/default")
    def back_to_default(x_approval_token: str = Header(default="")):
        """Show the saved demo run again (what the dashboard served at server start)."""
        require_token(x_approval_token)
        select_run(default_run(), False)
        return {"showing": os.path.basename(default_run()), "live": False}

    return r
