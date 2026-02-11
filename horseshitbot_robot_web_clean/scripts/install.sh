#!/usr/bin/env bash
set -e
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip git
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
echo "Done. Now: cp .env.example .env (edit), then run: uvicorn robot_web.main:app --host 0.0.0.0 --port 8000"
