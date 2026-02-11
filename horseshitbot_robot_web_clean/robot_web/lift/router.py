from __future__ import annotations
from fastapi import APIRouter
from pydantic import BaseModel, Field
from .controller import LiftController

class SpeedCmd(BaseModel):
    pct: float = Field(..., ge=-100.0, le=100.0)

def build_router(ctrl: LiftController) -> APIRouter:
    r = APIRouter(prefix="/api/lift", tags=["lift"])

    @r.post("/speed")
    def speed(cmd: SpeedCmd):
        ctrl.set_pct(cmd.pct)
        st = ctrl.get_status()
        return {"ok": True, "pct": cmd.pct, **st}

    @r.post("/stop")
    def stop():
        ctrl.stop()
        return {"ok": True}

    return r
