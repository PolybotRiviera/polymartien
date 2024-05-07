from typing import Sequence
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import VisualCylinder

import numpy as np
import random

from .obstacle import Obstacle


class ObstaclesArea(XFormPrim):
    def __init__(self, prim_path: str, name: str = "obstacles_area", position: Sequence[float] | None = None, translation: Sequence[float] | None = None, orientation: Sequence[float] | None = None, scale: Sequence[float] | None = None, visible: bool | None = None) -> None:
        super().__init__(prim_path, name, position, translation, orientation, scale, visible)
        self.cylinder = VisualCylinder(
            prim_path+"/cylinder_obstacle",
            "cylinder_obstacle",
            translation=np.array([0, 0, 0]),
            scale=np.array([150, 150, 150]),
            visible=False,
        )

        self.obstacles: list[Obstacle] = []
        for i in range(13):
            self.obstacles.append(Obstacle(prim_path+f"/obstacle_{i}", name=f"obstacle_{i}", translation=np.random.random(3)*np.array([2, 2, 0])-np.array([1, 1, 0])))


    def toggle_visibility(self):
        self.cylinder.set_visibility(not self.cylinder.get_visibility())


    def get_obstacles(self):
        return self.obstacles
