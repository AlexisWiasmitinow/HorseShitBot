#!/usr/bin/env bash
set -e
source .venv/bin/activate
uvicorn robot_web.main:app --host 0.0.0.0 --port 8000
