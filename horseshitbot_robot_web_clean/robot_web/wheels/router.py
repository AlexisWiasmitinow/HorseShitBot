from __future__ import annotations
from fastapi import APIRouter
from pydantic import BaseModel, Field
from .controller import WheelsController

class DriveCmd(BaseModel):
    x: float = Field(0.0, ge=-1.0, le=1.0)
    y: float = Field(0.0, ge=-1.0, le=1.0)

def build_router(ctrl: WheelsController) -> APIRouter:
    r = APIRouter(prefix="/api", tags=["wheels"])

    @r.post("/drive")
    def drive(cmd: DriveCmd):
        ctrl.set_drive(cmd.x, cmd.y)
        return {"ok": True, **ctrl.get_status()}

    @r.post("/stop")
    def stop():
        ctrl.set_stop()
        return {"ok": True}

    @r.post("/stop_fast")
    def stop_fast():
        ctrl.set_stop_fast()
        return {"ok": True}

    return r
