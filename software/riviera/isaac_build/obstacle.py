from typing import Sequence
import random
import os

from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np


class Obstacle(XFormPrim):
    count = 0

    def __init__(
        self,
        prim_path: str,
        name: str = "obstacle",
        position: Sequence[float] | None = None,
        translation: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        scale: Sequence[float] | None = None,
        visible: bool | None = True,
    ) -> None:
        super().__init__(
            prim_path, name, position, translation, orientation, scale, visible
        )
        Obstacle.count += 1
        prim_path = f"{prim_path}/obstacle_body/obstacle_{Obstacle.count}"
        add_reference_to_stage(
            usd_path=Obstacle.get_rand_obstacle_path(),
            prim_path=prim_path,
        )
        self.obstacle_body = XFormPrim(prim_path)
        self.obstacle_body.set_local_scale(np.array([1, 1, 1]) * 1 / 100)
        self.obstacle_body.set_local_pose(orientation=[1, 0, 0, 0]
        )
        # print(np.random.rand(3) * np.array([0, 0, 0]))
        # utils.setRigidBody(self.plant_body, "convexDecomposition", False)

    @staticmethod
    def get_rand_obstacle_path():
        obstacles_folder="/home/ekter/Documents/Omniverse/polybot/exts/polybot.riviera/polybot/riviera/models/obstacles/"
        return random.choice(os.listdir(obstacles_folder))

    def get_contour_rectangle_corners(self):
        return [self.get_world_pose()[0]]           #TODO fix this
