import time

import pytest
from fastapi.testclient import TestClient

import dashboard.app as dash
from test_uploads import make_video

TOKEN = "test-token"


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setattr(dash, "APPROVAL_TOKEN", TOKEN)
    monkeypatch.setattr(dash, "UPLOAD_DIR", tmp_path / "uploads")
    monkeypatch.setattr(dash, "MIN_FREE_BYTES_AFTER", 0)
    return TestClient(dash.app)


def post(client, path, name, token=TOKEN, **form):
    with open(path, "rb") as f:
        return client.post("/api/uploads", files={"file": (name, f, "video/x-msvideo")}, data=form,
                           headers={"X-Approval-Token": token})


def test_three_videos_upload_as_one_session(client, tmp_path):
    make_video(tmp_path / "c.avi")
    ids = [post(client, tmp_path / "c.avi", f"walker{c}.avi", camera=c, session="ab12cd34").json()["id"]
           for c in (0, 1, 2)]
    listed = {u["id"]: u for u in client.get("/api/uploads").json()}
    assert [listed[i]["camera"] for i in ids] == [0, 1, 2]
    assert {listed[i]["session"] for i in ids} == {"ab12cd34"}


def test_upload_rejects_bad_camera_or_session(client, tmp_path):
    make_video(tmp_path / "c.avi")
    assert post(client, tmp_path / "c.avi", "c.avi", camera=3).status_code == 422
    assert post(client, tmp_path / "c.avi", "c.avi", session="../x").status_code == 422


def test_upload_needs_the_operator_token(client, tmp_path):
    make_video(tmp_path / "c.avi")
    assert post(client, tmp_path / "c.avi", "c.avi", token="wrong").status_code == 403
    assert client.get("/api/uploads").json() == []


def test_upload_rejects_non_video(client, tmp_path):
    (tmp_path / "notes.txt").write_text("hi")
    assert post(client, tmp_path / "notes.txt", "notes.txt").status_code == 415


def test_upload_is_analyzed_and_can_be_deleted(client, tmp_path):
    make_video(tmp_path / "c.avi")
    r = post(client, tmp_path / "c.avi", "../../evil name.avi")
    assert r.status_code == 201
    uid = r.json()["id"]
    assert r.json()["name"] == "evil name.avi"            # path parts stripped
    for _ in range(100):
        detail = client.get(f"/api/uploads/{uid}").json()
        if detail["status"] in ("done", "failed"):
            break
        time.sleep(0.1)
    assert detail["status"] == "done" and detail["weak_segments"]
    assert client.get(f"/api/uploads/{uid}/thumb").status_code == 200
    listed = client.get("/api/uploads").json()
    assert [u["id"] for u in listed] == [uid] and "timeline" not in listed[0]
    assert client.delete(f"/api/uploads/{uid}", headers={"X-Approval-Token": "wrong"}).status_code == 403
    assert client.delete(f"/api/uploads/{uid}", headers={"X-Approval-Token": TOKEN}).status_code == 200
    assert client.get("/api/uploads").json() == []


def test_upload_ids_are_validated(client):
    assert client.get("/api/uploads/..%2F..%2Fetc").status_code == 404
    assert client.get("/api/uploads/zzzzzzzzzzzz").status_code == 404


def test_devices_are_listed_as_not_connected(client):
    devs = client.get("/api/devices").json()
    assert len(devs) == 3 and all(d["status"] == "not connected" and d["assumed"] for d in devs)
