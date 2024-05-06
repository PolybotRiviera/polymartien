from typing import Sequence
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import VisualCylinder

import numpy as np

from .plant import Plant


class PlantsArea(XFormPrim):
    count = 0
    def __init__(self, prim_path: str, name: str = "plant_area", position: Sequence[float] | None = None, translation: Sequence[float] | None = None, orientation: Sequence[float] | None = None, scale: Sequence[float] | None = None, visible: bool | None = None) -> None:
        super().__init__(prim_path, name, position, translation, orientation, scale, visible)
        PlantsArea.count+=1 # pls do not load async
        cylinder_name = "cylinder_plantarea_"+str(PlantsArea.count)
        self.cylinder = VisualCylinder(
            prim_path+"/"+cylinder_name,
            cylinder_name,
            translation=np.array([0, 0, 0]),
            scale=np.array([150, 150, 150]),
            visible=False,
        )
        
        plants_names = "plant_plantarea_"+str(PlantsArea.count)
        self.plants: list[Plant] = []
        for i in range(6):
            self.plants.append(Plant(prim_path+f"/plant_{i}", name=f"plant_{i}_"+plants_names, translation=np.random.random(3)*np.array([2, 2, 0])-np.array([1, 1, 0])))


    def toggle_visibility(self):
        self.cylinder.set_visibility(not self.cylinder.get_visibility())


    def get_plants(self):
        return self.plants