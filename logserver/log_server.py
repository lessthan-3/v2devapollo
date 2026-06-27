#!/usr/bin/env python3
"""
Apollo HVLP – Remote Device Log Server
=======================================
Endpoints:
  POST /log   — append a telemetry entry (requires X-Api-Key header)
  GET  /log   — return all stored entries as a JSON array

Configuration (environment variables):
  LOG_FILE     Path to the NDJSON log file  (default: /var/log/apollo/device_log.ndjson)
  API_KEY      Shared secret that devices must send in X-Api-Key header
               (default: apollo-secret-key  — override in production!)
  PORT         TCP port to listen on        (default: 8081)
  MAX_BYTES    Max log file size in bytes before rotation  (default: 5242880 = 5 MB)
  KEEP_LINES   Number of newest lines to keep after rotation  (default: 500)

Log format:
  Each accepted POST body is validated as JSON, stamped with a server-side
  "_received_at" ISO-8601 UTC timestamp, and appended as a single line
  (NDJSON).  When the file exceeds MAX_BYTES the oldest lines are dropped,
  keeping the newest KEEP_LINES entries.

  GET /log returns the full log as a pretty-printed JSON array, newest last.

To add future log fields: just send them in the device payload — no server
changes are required.
"""

import json
import os
import time
import logging
from http.server import BaseHTTPRequestHandler, HTTPServer

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
LOG_FILE   = os.environ.get("LOG_FILE",   os.path.expanduser("~/apollo-logserver/device_log.ndjson"))
API_KEY    = os.environ.get("API_KEY",    "apollo-secret-key")
PORT       = int(os.environ.get("PORT",   "8081"))
MAX_BYTES  = int(os.environ.get("MAX_BYTES",  str(5 * 1024 * 1024)))  # 5 MB
KEEP_LINES = int(os.environ.get("KEEP_LINES", "500"))

# ---------------------------------------------------------------------------
# Logging (server-side operational log, not device log)
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%SZ",
)
log = logging.getLogger("apollo-logserver")

# ---------------------------------------------------------------------------
# Log file helpers
# ---------------------------------------------------------------------------

def _ensure_dir():
    """Create the log directory if it does not yet exist."""
    try:
        directory = os.path.dirname(LOG_FILE)
        if directory:
            os.makedirs(directory, exist_ok=True)
    except OSError as e:
        log.error("Cannot create log directory: %s", e)


def _rotate_if_needed():
    """If the log file exceeds MAX_BYTES, trim it to the newest KEEP_LINES lines."""
    try:
        if os.path.getsize(LOG_FILE) < MAX_BYTES:
            return
        with open(LOG_FILE, "r", encoding="utf-8") as f:
            lines = f.readlines()
        keep = lines[-KEEP_LINES:] if len(lines) > KEEP_LINES else lines
        with open(LOG_FILE, "w", encoding="utf-8") as f:
            f.writelines(keep)
        log.info("Log rotated: kept %d of %d lines", len(keep), len(lines))
    except OSError:
        pass


def _read_entries():
    """Return all stored log entries as a list of dicts."""
    try:
        entries = []
        with open(LOG_FILE, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    entries.append(json.loads(line))
                except json.JSONDecodeError:
                    pass  # skip malformed lines
        return entries
    except FileNotFoundError:
        return []


def _append_entry(entry: dict) -> bool:
    """Stamp, rotate if needed, then append entry as a single NDJSON line."""
    entry["_received_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    _ensure_dir()
    try:
        _rotate_if_needed()
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, separators=(",", ":")) + "\n")
        return True
    except OSError as e:
        log.error("Failed to write log entry: %s", e)
        return False

# ---------------------------------------------------------------------------
# Request handler
# ---------------------------------------------------------------------------

class LogHandler(BaseHTTPRequestHandler):

    # Suppress the built-in per-request stdout logging; we use our own logger.
    def log_message(self, fmt, *args):  # noqa: D102
        pass

    def _send_json(self, code: int, body: str):
        encoded = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    # ------------------------------------------------------------------
    # GET /log  — return stored entries as a JSON array
    # ------------------------------------------------------------------
    def do_GET(self):  # noqa: N802
        if self.path != "/log":
            self._send_json(404, '{"error":"not found"}')
            return

        entries = _read_entries()
        self._send_json(200, json.dumps(entries, indent=2))
        log.info("GET /log — returned %d entries to %s", len(entries), self.client_address[0])

    # ------------------------------------------------------------------
    # POST /log  — validate, stamp, and append a telemetry entry
    # ------------------------------------------------------------------
    def do_POST(self):  # noqa: N802
        if self.path != "/log":
            self._send_json(404, '{"error":"not found"}')
            return

        # Auth check
        if self.headers.get("X-Api-Key") != API_KEY:
            log.warning("POST /log — unauthorized from %s", self.client_address[0])
            self._send_json(401, '{"error":"unauthorized"}')
            return

        # Size guard (64 KB is far more than any realistic device payload)
        content_length = int(self.headers.get("Content-Length", 0))
        if content_length > 64 * 1024:
            self._send_json(413, '{"error":"payload too large"}')
            return

        raw = self.rfile.read(content_length)

        # Validate JSON
        try:
            entry = json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            log.warning("POST /log — invalid JSON from %s", self.client_address[0])
            self._send_json(400, '{"error":"invalid json"}')
            return

        if not isinstance(entry, dict):
            self._send_json(400, '{"error":"payload must be a JSON object"}')
            return

        # Store
        if not _append_entry(entry):
            self._send_json(500, '{"error":"failed to write log"}')
            return
        log.info(
            "POST /log — stored entry: event=%s fw=%s from %s",
            entry.get("event", "?"),
            entry.get("fw_version", "?"),
            self.client_address[0],
        )
        self._send_json(201, '{"status":"ok"}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    _ensure_dir()
    server = HTTPServer(("0.0.0.0", PORT), LogHandler)
    log.info(
        "Apollo log server started — port=%d  log_file=%s  max_bytes=%d  keep_lines=%d",
        PORT, LOG_FILE, MAX_BYTES, KEEP_LINES,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down")
