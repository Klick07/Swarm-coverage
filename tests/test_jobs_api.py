import os
import time

import pytest
from fastapi.testclient import TestClient

import dashboard.app as dash
from jobs import runner

H = {"X-Approval-Token": "tok"}


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setattr(dash, "APPROVAL_TOKEN", "tok")
    monkeypatch.setattr(runner, "JOBS", tmp_path / "jobs"); monkeypatch.setattr(runner, "RUNS", tmp_path / "runs")
    (tmp_path / "jobs").mkdir(); (tmp_path / "runs").mkdir()
    monkeypatch.setattr(dash, "RUN_DIR", str(tmp_path / "runs" / "demo"))
    monkeypatch.setattr(dash, "DEFAULT_RUN_DIR", str(tmp_path / "runs" / "demo"))
    return TestClient(dash.app)


def test_token_required_for_changes_and_reads_open(client):
    assert client.post("/api/jobs").status_code == 403
    assert client.get("/api/jobs").status_code == 200


def test_chunked_upload_and_resume_over_http(client):
    jid = client.post("/api/jobs?n_robots=1", headers=H).json()["id"]
    u = f"/api/jobs/{jid}/files/r0_left"
    assert client.post(f"{u}?offset=0&total=6", content=b"abc", headers=H).json()["stored"] == 3
    r = client.post(f"{u}?offset=0&total=6", content=b"abc", headers=H)
    assert r.status_code == 409 and "resume from 3" in r.json()["detail"]
    assert client.post(f"{u}?offset=3&total=6", content=b"def", headers=H).json()["stored"] == 6
    assert (runner.jdir(jid) / "incoming" / "r0_left").read_bytes() == b"abcdef"


def test_start_blocked_until_ingested_and_state_switch(client, tmp_path):
    jid = client.post("/api/jobs?n_robots=1", headers=H).json()["id"]
    r = client.post(f"/api/jobs/{jid}/start?rate=0.25", headers=H)
    assert r.status_code == 409 and "ingested" in r.json()["detail"]
    # switch the page to a job run that has no coordinator DB yet: state must not 500 or create a DB
    (tmp_path / "runs" / f"job_{jid}").mkdir()
    assert client.post(f"/api/jobs/{jid}/show", headers=H).json()["live"] is True
    s = client.get("/api/state").json()
    assert s["live"] is True and s["waiting"] is True and s["cameras"] == []
    assert not (tmp_path / "runs" / f"job_{jid}" / "coverage.db").exists()
    back = client.post("/api/runs/default", headers=H).json()
    assert back == {"showing": "demo", "live": False}


def test_bad_rate_is_a_409_not_500(client):
    jid = client.post("/api/jobs?n_robots=1", headers=H).json()["id"]
    runner.write_meta(jid, robots={"0": {"state": "ready", "report": {"duration_s": 5, "pairs_written": 5, "video_size": [640, 512]}}})
    assert client.post(f"/api/jobs/{jid}/start?rate=9", headers=H).status_code == 409


def test_live_state_survives_empty_or_missing_db(client, tmp_path):
    import sqlite3
    run = tmp_path / "runs" / "job_x"; run.mkdir()
    dash.select_run(str(run), True)
    assert client.get("/api/state").json()["waiting"] is True
    assert client.get("/api/frame/0").status_code == 404 and not (run / "coverage.db").exists()
    sqlite3.connect(run / "coverage.db").close()           # empty file, no tables yet
    assert client.get("/api/state").json()["waiting"] is True
    dash.select_run(str(run), False)                       # a saved run with a broken DB must still fail loudly
    with pytest.raises(sqlite3.OperationalError):
        client.get("/api/state")
