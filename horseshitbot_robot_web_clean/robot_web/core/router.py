from __future__ import annotations
from fastapi import APIRouter
from .bus import MODE_SR_VFOC, MksBus

def build_router(bus: MksBus, motor_ids: list[int]):
    r = APIRouter(prefix="/api", tags=["system"])

    @r.post("/clear_errors")
    def clear_errors():
        results = {}
        for mid in motor_ids:
            try:
                bus.clear_error_state(int(mid), mode=MODE_SR_VFOC)
                results[str(mid)] = "ok"
            except Exception as e:
                results[str(mid)] = str(e)
        return {"ok": True, "results": results}

    return r
