from __future__ import annotations

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

def sign(x: float) -> int:
    if x > 0: return 1
    if x < 0: return -1
    return 0

def get_env_str(name: str, default: str) -> str:
    import os
    return os.getenv(name, default)

def get_env_int(name: str, default: int) -> int:
    import os
    try:
        return int(os.getenv(name, str(default)))
    except Exception:
        return default

def get_env_float(name: str, default: float) -> float:
    import os
    try:
        return float(os.getenv(name, str(default)))
    except Exception:
        return default

def get_env_bool(name: str, default: bool) -> bool:
    import os
    v = os.getenv(name)
    if v is None:
        return default
    return v.strip().lower() in ("1","true","yes","y","on")
