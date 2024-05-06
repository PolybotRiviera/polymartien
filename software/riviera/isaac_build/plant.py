from typing import Sequence

from omni.isaac.core.prims import XFormPrim
from omni.physx.scripts import utils
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np


class Plant(XFormPrim):
    count = 0

    def __init__(
        self,
        prim_path: str,
        name: str = "plant",
        position: Sequence[float] | None = None,
        translation: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        scale: Sequence[float] | None = None,
        visible: bool | None = True,
    ) -> None:
        super().__init__(
            prim_path, name, position, translation, orientation, scale, visible
        )
        Plant.count += 1
        prim_path = f"{prim_path}/plant_body/plant_{Plant.count}"
        add_reference_to_stage(
            usd_path="/home/ekter/Documents/to_convert/plante v1.usdz",
            prim_path=prim_path,
        )
        self.plant_body = XFormPrim(prim_path)
        self.plant_body.set_local_scale(np.array([1, 1, 1]) * 1 / 100)
        self.plant_body.set_local_pose(orientation=[1, 0, 0, 0]
        )
        # print(np.random.rand(3) * np.array([0, 0, 0]))
        # utils.setRigidBody(self.plant_body, "convexDecomposition", False)
