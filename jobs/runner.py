"""Upload job lifecycle: create -> ingest robots -> start (Swarm-SLAM compute) -> status / stop.

One job runs at a time (2 vCPU). A job's compute is the same stack as scripts/run_euroc.sh
(replay -> OpenCV front end -> RTAB-Map stereo odometry -> Swarm-SLAM -> build_pointcloud --watch),
plus the rules-only coordinator in --follow mode. Nothing here calls Bedrock/Nova.
Run output: ~/runs/job_<id>/ (separate from every other run).
"""
import json
import os
import shutil
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path

import numpy as np

from jobs import ingest as ing

REPO = Path(__file__).resolve().parent.parent
JOBS = Path(os.environ.get("JOBS_DIR", os.path.expanduser("~/jobs")))
RUNS = Path(os.environ.get("RUNS_DIR", os.path.expanduser("~/runs")))
MIN_FREE_BYTES = 3 * 1024 ** 3
START_DELAY_S = 20           # euroc_replay --start-delay
MIN_RATE, MAX_RATE = 0.05, 2.0


class JobError(ValueError):
    pass


def jdir(job_id):
    if not (len(job_id) == 12 and all(c in "0123456789abcdef" for c in job_id)):
        raise JobError("bad job id")
    return JOBS / job_id


def read_meta(job_id):
    try:
        return json.load(open(jdir(job_id) / "meta.json"))
    except FileNotFoundError:
        raise JobError("no such job")


_lock = threading.Lock()


def write_meta(job_id, **fields):
    with _lock:
        d = jdir(job_id)
        path = d / "meta.json"
        meta = json.load(open(path)) if path.exists() else {}
        meta.update(fields)
        with open(f"{path}.tmp", "w") as f:
            json.dump(meta, f)
        os.replace(f"{path}.tmp", path)
        return meta


def create_job(n_robots=3):
    if not 1 <= n_robots <= 3:
        raise JobError("1 to 3 robots")
    job_id = uuid.uuid4().hex[:12]
    (JOBS / job_id / "incoming").mkdir(parents=True)
    write_meta(job_id, id=job_id, created=time.time(), n_robots=n_robots, status="created", robots={}, run=f"job_{job_id}")
    return job_id


def save_chunk(job_id, slot, offset, total, data, max_total=8 * 1024 ** 3):
    """Append one upload chunk (client sends <=90 MB pieces; the tunnel rejects bigger requests)."""
    meta = read_meta(job_id)
    r, kind = parse_slot(slot, meta["n_robots"])
    if total > max_total or total <= 0:
        raise JobError("bad total size")
    d = jdir(job_id) / "incoming"
    d.mkdir(exist_ok=True)
    path = d / slot
    have = path.stat().st_size if path.exists() else 0
    if offset != have:
        raise JobError(f"offset {offset} but {have} bytes stored; resume from {have}")
    if offset == 0 and shutil.disk_usage(d).free - total < MIN_FREE_BYTES:
        raise JobError("not enough disk space for this file")
    if have + len(data) > total:
        raise JobError("chunk goes past the declared total")
    with open(path, "ab") as f:
        f.write(data)
    return have + len(data)


def parse_slot(slot, n_robots):
    """r<i>_left | r<i>_right | r<i>_calib | r<i>_left_stamps | r<i>_right_stamps"""
    try:
        head, kind = slot.split("_", 1)
        r = int(head[1:])
        assert head[0] == "r" and 0 <= r < n_robots
        assert kind in ("left", "right", "calib", "left_stamps", "right_stamps")
    except (ValueError, AssertionError):
        raise JobError(f"bad slot {slot!r}")
    return r, kind


def ingest_robot(job_id, r, swap_eyes=False, calib_swap_eyes=False, target_hz=10.0, max_seconds=None):
    """Blocking; call from a worker thread. Records the report (or the error) in meta."""
    inc, out = jdir(job_id) / "incoming", jdir(job_id) / f"robot{r}"
    f = lambda k: str(inc / f"r{r}_{k}") if (inc / f"r{r}_{k}").exists() else None
    robots = read_meta(job_id)["robots"]
    robots[str(r)] = {"state": "ingesting"}
    write_meta(job_id, robots=robots)
    try:
        for k in ("left", "right", "calib"):
            if not f(k):
                raise ing.IngestError(f"missing upload: {k}")
        calib = ing.load_calibration(f("calib"), swap_eyes=calib_swap_eyes)
        if out.exists():
            shutil.rmtree(out)
        out.mkdir()
        rep = ing.ingest_robot(f("left"), f("right"), str(out), calib, f("left_stamps"), f("right_stamps"),
                               target_hz=target_hz, max_seconds=max_seconds)
        robots = read_meta(job_id)["robots"]
        robots[str(r)] = {"state": "ready", "report": rep}
    except (ing.IngestError, OSError, KeyError, ValueError) as e:
        robots = read_meta(job_id)["robots"]
        robots[str(r)] = {"state": "error", "error": str(e)[:400]}
    write_meta(job_id, robots=robots)
    return robots[str(r)]


# ---------- running ----------
_procs = {}


def _alive(pgid):
    p = _procs.get(pgid)
    if p is not None:
        p.poll()  # reap the group leader so it does not linger as a zombie that still answers killpg
    try:
        os.killpg(pgid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def active_job():
    try:
        jid = (JOBS / "ACTIVE").read_text().strip()
        m = read_meta(jid)
        if m.get("pgid") and _alive(m["pgid"]):
            return jid
    except (FileNotFoundError, JobError):
        pass
    return None


def estimate_wall_s(duration_s, rate):
    return int(duration_s / rate + START_DELAY_S + 120)


def start(job_id, rate, max_wall_s=None, cmd=None):
    """Launch the compute. `rate` is the replay speed vs the footage (1.0 = real time; unproven here).
    `cmd` replaces the launch command (tests only)."""
    meta = read_meta(job_id)
    if not MIN_RATE <= rate <= MAX_RATE:
        raise JobError(f"rate must be between {MIN_RATE} and {MAX_RATE}")
    ready = [i for i in range(meta["n_robots"]) if meta["robots"].get(str(i), {}).get("state") == "ready"]
    if len(ready) != meta["n_robots"]:
        raise JobError("every robot must be ingested without error before Start")
    if meta.get("pgid") and _alive(meta["pgid"]):
        raise JobError("this job is already running")
    other = active_job()
    if other:
        raise JobError(f"job {other} is running; stop it first (one job at a time)")
    if shutil.disk_usage(RUNS if RUNS.exists() else JOBS).free < MIN_FREE_BYTES:
        raise JobError("not enough free disk space to start")
    d = jdir(job_id)
    duration = min(meta["robots"][str(i)]["report"]["duration_s"] for i in ready)
    max_wall = int(max_wall_s or estimate_wall_s(duration, rate))
    calib_dir = d / "calib"
    calib_dir.mkdir(exist_ok=True)
    w, h = meta["robots"]["0"]["report"]["video_size"]
    for i in ready:
        shutil.copy(d / f"robot{i}" / "calib.yaml", calib_dir / f"robot{i}.yaml")
    crop = max(64, min(w, h) // 32 * 32)
    cfg = (REPO / "config" / "cslam_stereo.yaml").read_text().replace("image_crop_size: 480", f"image_crop_size: {crop}")
    cfg_name = f"job_{job_id}.yaml"
    (REPO / "config" / cfg_name).write_text(cfg)
    out = RUNS / f"job_{job_id}"
    out.mkdir(parents=True, exist_ok=True)
    shutil.copytree(calib_dir, out / "calib", dirs_exist_ok=True)   # build_pointcloud reads calibration from the run
    seqs = ",".join(f"robot{i}" for i in range(meta["n_robots"]))
    log = open(out / "job.log", "ab")
    env = {k: v for k, v in os.environ.items() if k not in ("VIRTUAL_ENV", "VIRTUAL_ENV_PROMPT", "PYTHONHOME", "PYTHONPATH")}
    # The dashboard runs inside the cv5 venv (NumPy 2); ROS python nodes (cv_bridge) need the system python.
    env["PATH"] = os.pathsep.join(x for x in env.get("PATH", "").split(os.pathsep) if "cv5env" not in x)
    env.update(DATA_ROOT=str(d), CALIB_DIR=str(calib_dir), CSLAM_CFG=cfg_name, RUNS_DIR=str(RUNS))
    argv = cmd or ["bash", str(REPO / "scripts" / "run_job.sh"), f"job_{job_id}", str(rate), str(max_wall), seqs]
    p = subprocess.Popen(argv, cwd=REPO, env=env, stdout=log, stderr=log, stdin=subprocess.DEVNULL, start_new_session=True)
    _procs[p.pid] = p
    (JOBS / "ACTIVE").write_text(job_id)
    write_meta(job_id, status="running", pgid=p.pid, rate=rate, max_wall_s=max_wall, started=time.time(),
               run=f"job_{job_id}", stopped=False, live=True, total_pairs=min(meta["robots"][str(i)]["report"]["pairs_written"] for i in ready))
    return read_meta(job_id)


def _tree(root):
    """root plus every descendant pid. `timeout` and ros2 launch start their own process groups, so killpg on
    the job's group alone leaves the whole ROS stack running (found in the first real job run)."""
    out = subprocess.run(["ps", "-eo", "pid=,ppid="], capture_output=True, text=True).stdout
    kids = {}
    for line in out.splitlines():
        pid, ppid = (int(x) for x in line.split())
        kids.setdefault(ppid, []).append(pid)
    seen, todo = [], [root]
    while todo:
        pid = todo.pop()
        seen.append(pid)
        todo.extend(kids.get(pid, []))
    return seen


def _signal_all(pids, sig):
    for pid in pids:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def _any_alive(pids):
    for pid in pids:
        try:
            os.kill(pid, 0)
            if not os.path.exists(f"/proc/{pid}/stat") or open(f"/proc/{pid}/stat").read().split(") ")[-1][0] != "Z":
                return True
        except (ProcessLookupError, PermissionError):
            pass
    return False


def stop(job_id, grace_s=10):
    meta = read_meta(job_id)
    pgid = meta.get("pgid")
    if not pgid or not _alive(pgid):
        return status(job_id)
    write_meta(job_id, stopped=True)
    pids = _tree(pgid)
    for sig, wait in ((signal.SIGINT, grace_s), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        _signal_all(pids, sig)
        end = time.time() + wait
        while time.time() < end and (_alive(pgid) or _any_alive(pids)):
            time.sleep(0.2)
        pids = list(dict.fromkeys(pids + _tree(pgid)))
        if not _alive(pgid) and not _any_alive(pids):
            break
    return status(job_id)


def _points(run):
    try:
        with open(run / "pointcloud.pcd", "rb") as f:
            for line in f.read(600).decode(errors="ignore").splitlines():
                if line.startswith("POINTS"):
                    return int(line.split()[1])
    except (FileNotFoundError, ValueError):
        pass
    return None


def status(job_id):
    meta = read_meta(job_id)
    run = RUNS / meta["run"]
    pgid = meta.get("pgid")
    alive = bool(pgid and _alive(pgid))
    state = meta["status"]
    if state == "running" and not alive:
        done = run / "run.done"
        if done.exists():
            # run_job.sh writes "exit=<code>"; 124 is `timeout` firing, i.e. the wall-clock cap cut the run short
            code = done.read_text().strip()
            state = "stopped" if meta.get("stopped") else "timed_out" if code == "exit=124" else "finished"
        else:
            state = "stopped" if meta.get("stopped") else "failed"
        ended = done.stat().st_mtime if done.exists() else time.time()
        write_meta(job_id, status=state, ended=ended)
        if (JOBS / "ACTIVE").exists() and (JOBS / "ACTIVE").read_text().strip() == job_id:
            (JOBS / "ACTIVE").unlink()
    published = []
    for i in range(meta["n_robots"]):
        try:
            # quality_r<i>.jsonl is written (line-buffered) per frame the front end receives; stamps_r<i>.csv is block-buffered
            published.append(sum(1 for _ in open(run / f"quality_r{i}.jsonl")))
        except FileNotFoundError:
            published.append(0)
    total = meta.get("total_pairs")
    s = {**meta, "status": state, "alive": alive, "frames_published": published, "points": _points(run),
         "all_frames_fed": bool(total) and min(published or [0]) >= total - 6,
         "elapsed_s": round(time.time() - meta["started"], 1) if meta.get("started") and (alive or not meta.get("ended")) else None}
    if state == "failed":
        try:
            s["log_tail"] = (run / "job.log").read_text()[-600:]
        except FileNotFoundError:
            pass
    return s
