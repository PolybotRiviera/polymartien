import omni.ext
import omni.ui as ui
import os
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.prims import XFormPrim

import numpy as np
import random

# import time
# import threading

from .isaac_build import arena


EXT_PATH = (
    "/home/ekter/Documents/Omniverse/polybot/exts/polybot.riviera/polybot/riviera"
)


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class PolybotRivieraExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[polybot.riviera] polybot riviera startup")
        print(ext_id)

        self._count = 0
        self.lights = []
        self.arena: arena.RArena = None

        self._window = ui.Window("POLYBOTTTT", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                label_count = ui.Label("")

                # label_path = ui.Label(os.getcwd())
                # label_name = ui.Label(__file__)
                def toggle():
                    if hasattr(self, "arena"):
                        self.arena.toggle_visibility_()

                ui.Button("toggle arena hitboxes visibility", clicked_fn=toggle)

                def five_lights(five=5):
                    print(five)
                    for _ in range(five):
                        self.lights.append(
                            prim_utils.create_prim(
                                f"/World/Lights/Light_{random.randint(0,4567382)}",
                                "SphereLight",
                                position=np.random.random((3)) * 1000,
                                attributes={
                                    "inputs:radius": (int(random.expovariate(1 / 500))),
                                    "inputs:intensity": (
                                        int(random.expovariate(1 / 5e4))
                                    ),
                                    "inputs:color": tuple(
                                        random.uniform(0, 1) for _ in range(3)
                                    ),
                                },
                            ).GetPath()
                        )
                    lightsbut.text = f"add five lights(already {len(self.lights)})"

                lightsbut = ui.Button("add five lights", clicked_fn=five_lights)

                def randomize_plants():
                    print("random?")
                    self.arena.randomize_plants()

                ui.Button("random plants", clicked_fn=randomize_plants)

                def picture():
                    print("data: ")
                    for pos, orient in self.arena.get_plants_poses():
                        print(pos - np.array([0, -1500, 500]))

                # ui.Button("make data", clicked_fn=picture)

                # def rotate_cam():
                #     print(slider.model.as_float)
                #     self.arena.set_camera_angle(slider.model.as_float)
                # slider = ui.FloatSlider(min=-3.14, max=3.14)
                # ui.Button("rotate",clicked_fn=rotate_cam)
                # slider.model.subscribe_value_changed_fn(rotate_cam)

                def make_one_pictures(one=1):
                    for i in range(one):
                        self._count += 1
                        print(i)

                        print("changing lights")
                        destroy_lights()
                        for i in range(random.randint(0, 2)):
                            five_lights()
                        print("re-orienting")
                        orient_env_light()
                        # photo
                        print("cwd: ", os.getcwd())
                        csv_file = (
                            EXT_PATH
                            + "/data/"
                            + self.field_path.model.get_value_as_string()
                        )
                        folder = csv_file.replace(".csv", "/")
                        if not os.path.exists(csv_file):
                            with open(csv_file, "w") as f:
                                f.write(
                                    "index,image0_path,image1_path,image2_path,image3_path"
                                )
                                for i in range(36):
                                    f.write(f",plant{i}.r,plant{i}.phi")
                                for i in range(36):
                                    f.write(
                                        f",obstacle{i}.x1,obstacle{i}.y1,obstacle{i}.x2,obstacle{i}.y2,obstacle{i}.x3,obstacle{i}.y3,obstacle{i}.x4,obstacle{i}.y4"
                                    )
                                f.write("\n")
                        if not os.path.exists(folder):
                            os.makedirs(folder)
                        with open(csv_file, "r") as f:
                            num = len(f.readlines())
                        print("taking photos")
                        with open(csv_file, "a") as f:
                            f.write(f"{num}")
                            for index, cam in enumerate(self.arena.get_cameras()):
                                self.arena.photo_to(
                                    folder + f"img{num}_{index}.png", cam
                                )
                                f.write(f",img{num}_{index}.png")
                            print(f"{index+1} photos taken")
                            coos = self.arena.get_plants_in_camera_ref()
                            for r, phi in coos:
                                f.write(f",{r},{phi}")  # TODO: fix thi
                            for _ in range(36):
                                f.write(",0,0,0,0,0,0,0,0")
                            f.write("\n")
                        label_count.text = f"count: {self._count}"

                # def upadate_sync():
                #     print("updating")
                #     from omni.isaac.core import World

                #     world = World(set_defaults=True)
                #     print(type(world))
                #     print(world.__dict__)
                #     print(world.get_physics_dt())
                #     for _ in range(100):

                #         world.step(render=True)
                #     print("updated")

                with ui.HStack():

                    ui.Button("picture to ext path/data/", clicked_fn=make_one_pictures)
                    self.field_path = ui.StringField(
                        ui.SimpleStringModel(defaultValue="images.csv")
                    )
                    # ui.Button("upadate sync", clicked_fn=upadate_sync)

                def load():
                    light = XFormPrim("/Environment/defaultLight")
                    light.set_visibility(False)
                    if self.arena is None:
                        self.arena = arena.RArena()
                    self.arena.populate()
                    self._count += 1
                    label_count.text = f"count: {self._count}"

                def on_reset():
                    self._count = 0
                    label_count.text = "empty"
                    destroy_lights()
                    lightsbut.text = f"add five lights(already {len(self.lights)})"

                def destroy_lights():
                    print(self.lights)
                    for light in self.lights:
                        prim_utils.delete_prim(light)
                    self.lights = []

                def orient_env_light():
                    light = XFormPrim("/Environment/defaultLight")
                    light.set_visibility(False)
                    light.set_local_pose(
                        orientation=np.array(
                            [
                                0.707,
                                random.random() / 4 - 0.25,
                                1 + random.random() / 4 - 0.25,
                                0,
                            ]
                        )
                    )
                    setattr(
                        light, "color", tuple(random.random() * 2 for _ in range(3))
                    )
                    print(light.__dict__)

                # on_reset()

                with ui.HStack():
                    ui.Button("Load world", clicked_fn=load)
                    ui.Button("Reset", clicked_fn=on_reset)

    def on_shutdown(self):
        print("[polybot.riviera] polybot riviera shutdown (5+2=5)")
