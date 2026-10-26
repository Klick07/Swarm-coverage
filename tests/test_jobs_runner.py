import os
import time

import pytest

from jobs import runner

CLIP = os.path.expanduser("~/data/airmuseum/clips/A")


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "JOBS", tmp_path / "jobs")
    monkeypatch.setattr(runner, "RUNS", tmp_path / "runs")
    (tmp_path / "jobs").mkdir(); (tmp_path / "runs").mkdir()
    return tmp_path


def ready_job(n=1, pairs=100, dur=10.0):
    jid = runner.create_job(n)
    robots = {str(i): {"state": "ready", "report": {"duration_s": dur, "pairs_written": pairs, "video_size": [640, 512]}} for i in range(n)}
    runner.write_meta(jid, robots=robots)
    for i in range(n):
        d = runner.jdir(jid) / f"robot{i}"; d.mkdir(); (d / "calib.yaml").write_text("x: 1\n")
    return jid


def test_chunks_resume_and_bad_input(env):
    jid = runner.create_job(2)
    assert runner.save_chunk(jid, "r0_left", 0, 10, b"12345") == 5
    with pytest.raises(runner.JobError, match="resume from 5"):
        runner.save_chunk(jid, "r0_left", 0, 10, b"12345")
    assert runner.save_chunk(jid, "r0_left", 5, 10, b"67890") == 10
    with pytest.raises(runner.JobError, match="past"):
        runner.save_chunk(jid, "r0_right", 0, 3, b"abcd")
    for bad in ("r5_left", "r0_video", "x"):
        with pytest.raises(runner.JobError):
            runner.save_chunk(jid, bad, 0, 3, b"abc")
    with pytest.raises(runner.JobError):
        runner.read_meta("../../etc/passwd")


def test_start_requires_ready_and_rate_bounds(env):
    jid = runner.create_job(2)
    with pytest.raises(runner.JobError, match="ingested"):
        runner.start(jid, 0.25, cmd=["true"])
    jid = ready_job()
    for rate in (0, 5):
        with pytest.raises(runner.JobError, match="rate"):
            runner.start(jid, rate, cmd=["true"])


def test_start_status_stop_and_single_job_lock(env, monkeypatch):
    monkeypatch.setattr(runner, "REPO", env)  # job config is written under REPO/config
    (env / "config").mkdir(); (env / "config" / "cslam_stereo.yaml").write_text("image_crop_size: 480\n")
    a, b = ready_job(), ready_job()
    sleeper = ["bash", "-c", "mkdir -p $RUNS_DIR/job_%s; for i in 1 2 3; do echo x >> $RUNS_DIR/job_%s/stamps_r0.csv; done; sleep 60 & wait"]
    m = runner.start(a, 0.25, cmd=["bash", "-c", f"printf '{{}}\\n{{}}\\n' > {env}/runs/job_{a}/quality_r0.jsonl; sleep 60"])
    assert m["status"] == "running" and m["max_wall_s"] == runner.estimate_wall_s(10.0, 0.25)
    assert (env / "config" / f"job_{a}.yaml").read_text() == "image_crop_size: 512\n"   # crop follows 640x512
    time.sleep(0.5)
    s = runner.status(a)
    assert s["alive"] and s["frames_published"] == [2] and s["total_pairs"] == 100
    with pytest.raises(runner.JobError, match="one job at a time"):
        runner.start(b, 0.25, cmd=["sleep", "5"])
    s = runner.stop(a, grace_s=3)
    assert s["status"] == "stopped" and not s["alive"]
    assert runner.active_job() is None
    # a stopped job frees the slot
    runner.start(b, 0.5, cmd=["bash", "-c", f"touch {env}/runs/job_{b}/run.done"])
    time.sleep(0.5)
    assert runner.status(b)["status"] == "finished"


def test_failed_job_reports_log(env, monkeypatch):
    monkeypatch.setattr(runner, "REPO", env); (env / "config").mkdir(); (env / "config" / "cslam_stereo.yaml").write_text("image_crop_size: 480\n")
    jid = ready_job()
    runner.start(jid, 0.25, cmd=["bash", "-c", "echo boom >&2; exit 3"])
    time.sleep(0.5)
    s = runner.status(jid)
    assert s["status"] == "failed" and "boom" in s["log_tail"]


@pytest.mark.skipif(not os.path.isdir(CLIP), reason="no clip")
def test_ingest_robot_end_to_end_records_error_then_success(env):
    import cv2, csv
    jid = runner.create_job(1)
    inc = runner.jdir(jid) / "incoming"
    rows = [r for r in csv.reader(open(f"{CLIP}/mav0/cam0/data.csv")) if not r[0].startswith("#")][400:460]
    for side, cam in (("left", "cam0"), ("right", "cam1")):
        vw = cv2.VideoWriter(str(inc / f"r0_{side}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (640, 512), True)
        for r in rows:
            vw.write(cv2.cvtColor(cv2.imread(f"{CLIP}/mav0/{cam}/data/{r[1]}", 0), cv2.COLOR_GRAY2BGR))
        vw.release()
    for side in ("left", "right"):
        os.rename(inc / f"r0_{side}.mp4", inc / f"r0_{side}")  # slots carry no extension; cv2 sniffs the container
    assert runner.ingest_robot(jid, 0)["error"].startswith("missing upload: calib")
    (inc / "r0_calib").write_bytes(open(f"{CLIP}/calib.yaml", "rb").read())
    out = runner.ingest_robot(jid, 0)
    assert out["state"] == "ready", out
    assert out["report"]["pairs_written"] == 60


def test_stop_kills_children_in_their_own_process_group(env, monkeypatch):
    """Regression: `timeout` puts its child in a new group, so the first stop left the ROS stack running."""
    monkeypatch.setattr(runner, "REPO", env); (env / "config").mkdir(); (env / "config" / "cslam_stereo.yaml").write_text("image_crop_size: 480\n")
    jid = ready_job()
    marker = env / "child.pid"
    runner.start(jid, 0.25, cmd=["bash", "-c", f"timeout 60 bash -c 'echo $$ > {marker}; sleep 60'"])
    for _ in range(30):
        if marker.exists() and marker.read_text().strip():
            break
        time.sleep(0.1)
    child = int(marker.read_text())
    assert runner._any_alive([child])
    runner.stop(jid, grace_s=3)
    assert not runner._any_alive([child])


def test_wall_clock_cap_is_reported_as_timed_out_not_finished(env, monkeypatch):
    monkeypatch.setattr(runner, "REPO", env)
    (env / "config").mkdir(); (env / "config" / "cslam_stereo.yaml").write_text("image_crop_size: 480\n")
    a, b, c = ready_job(), ready_job(), ready_job()
    runner.start(a, 0.25, max_wall_s=120, cmd=["bash", "-c", f"echo 'exit=124' > {env}/runs/job_{a}/run.done"])
    time.sleep(0.5)
    s = runner.status(a)
    assert s["status"] == "timed_out" and not s["alive"] and s["max_wall_s"] == 120
    # a normal end is still "finished"; an explicit stop still wins over the exit code
    runner.start(b, 0.25, cmd=["bash", "-c", f"echo 'exit=0' > {env}/runs/job_{b}/run.done"])
    time.sleep(0.5)
    assert runner.status(b)["status"] == "finished"
    runner.start(c, 0.25, cmd=["bash", "-c", f"echo 'exit=124' > {env}/runs/job_{c}/run.done"])
    runner.write_meta(c, stopped=True)
    time.sleep(0.5)
    assert runner.status(c)["status"] == "stopped"


def test_run_job_script_records_timeout_exit_code():
    text = (runner.REPO / "scripts" / "run_job.sh").read_text()
    assert 'RC=$?' in text and 'echo "exit=$RC" > "$OUT/run.done"' in text
