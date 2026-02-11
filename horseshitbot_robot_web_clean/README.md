# Horseshitbot Robot Web — clean modular structure

Split by mechanism:
- `robot_web/wheels/` : differential drive (motors 1 & 2)
- `robot_web/lift/` : lift (motors 3 & 5 counter-rotate)
- `robot_web/brush/` : brush (motor 4)
- `robot_web/side_door/` : side door (motor 6)

Shared Modbus bus + utilities are in `robot_web/core/`.

## Run
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# edit .env
uvicorn robot_web.main:app --host 0.0.0.0 --port 8000
```

Open: http://<PI_IP>:8000
