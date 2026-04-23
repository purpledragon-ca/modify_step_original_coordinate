"""Tests for the new /api/load, /api/loaded, /api/health handlers.

These hit the real BaseHTTPRequestHandler by constructing request/response
BytesIO pairs — no socket, no process.
"""
from io import BytesIO
import json

import find_bottom_center as fbc


class _FakeSock:
    """Minimal BytesIO-backed socket stand-in for BaseHTTPRequestHandler."""
    def __init__(self, request_bytes: bytes):
        self._in = BytesIO(request_bytes)
        self._out = BytesIO()
    def makefile(self, mode, *a, **kw):
        return self._in if "r" in mode else self._out
    def sendall(self, data): self._out.write(data)


def _make_request(method: str, path: str, body: bytes = b"") -> bytes:
    headers = [
        f"{method} {path} HTTP/1.1".encode(),
        b"Host: localhost",
        f"Content-Length: {len(body)}".encode(),
    ]
    if body:
        headers.append(b"Content-Type: application/json")
    return b"\r\n".join(headers) + b"\r\n\r\n" + body


def _call_handler(method: str, path: str, body: bytes = b"") -> tuple[int, dict]:
    sock = _FakeSock(_make_request(method, path, body))
    # Directly drive the handler; it parses the request from makefile().
    fbc._Handler(sock, ("127.0.0.1", 0), None)
    sock._out.seek(0)
    data = sock._out.getvalue()
    header_end = data.index(b"\r\n\r\n")
    status_line, _, _ = data[:header_end].partition(b"\r\n")
    status = int(status_line.split()[1])
    payload = data[header_end + 4:]
    try:
        parsed = json.loads(payload.decode())
    except Exception:
        parsed = {"__raw__": payload}
    return status, parsed


def test_health_endpoint(mock_occ):
    status, body = _call_handler("GET", "/api/health")
    assert status == 200
    assert body["ok"] is True
    assert body["satellite"] == "modify"
    assert body["busy"] is False
    assert body["loaded"] is None


def test_loaded_returns_null_initially(mock_occ):
    status, body = _call_handler("GET", "/api/loaded")
    assert status == 200
    assert body["path"] is None


def test_load_endpoint_parses_and_sets_state(mock_occ, sample_step_path):
    req = json.dumps({"path": sample_step_path}).encode()
    status, body = _call_handler("POST", "/api/load", req)
    assert status == 200, body
    assert body["ok"] is True
    assert body["loaded"] == sample_step_path

    status2, loaded = _call_handler("GET", "/api/loaded")
    assert loaded["path"] == sample_step_path
    assert loaded["filename"] == "cc_waste_bin.step"


def test_load_missing_path_returns_400(mock_occ):
    req = json.dumps({"path": "/nope.step"}).encode()
    status, body = _call_handler("POST", "/api/load", req)
    assert status == 400


def test_load_same_path_twice_short_circuits(mock_occ, sample_step_path):
    req = json.dumps({"path": sample_step_path}).encode()
    _call_handler("POST", "/api/load", req)
    before = mock_occ["load_step"]
    _call_handler("POST", "/api/load", req)
    after = mock_occ["load_step"]
    assert after == before, "second /api/load should not re-parse same path"
