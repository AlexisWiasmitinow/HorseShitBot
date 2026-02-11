from __future__ import annotations
from fastapi import APIRouter
from pydantic import BaseModel, Field
from .controller import AuxSpeedController

class SpeedCmd(BaseModel):
    pct: float = Field(..., ge=-100.0, le=100.0)

def build_router(ctrl: AuxSpeedController) -> APIRouter:
    r = APIRouter(prefix="/api/brush", tags=["brush"])

    @r.post("/speed")
    def speed(cmd: SpeedCmd):
        ctrl.set_pct(cmd.pct)
        return {"ok": True, "pct": cmd.pct, **ctrl.get_status()}

    @r.post("/stop")
    def stop():
        ctrl.stop()
        return {"ok": True}

    return r
