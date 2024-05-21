import numpy as np
import math
# from time import sleep
import random
import sys
print(sys.version)
import os
print(os.getcwd())
# os.system("/home/ekter/Documents/Omniverse/polybot/app/kit/python/bin/python3 -m pip install opencv-python-headless")
# os.system("/home/ekter/Documents/Omniverse/polybot/app/kit/python/bin/python3 -m pip install imageio")
sys.path.append("/home/ekter/.local/lib/python3.10/site-packages")
import cv2

from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import FixedCuboid
# from omni.isaac.core.world import World
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import add_reference_to_stage

#import euler to quaternion conversion
import omni.isaac.core.utils.numpy.rotations as rot_utils

from omni.syntheticdata import _syntheticdata
sdi = _syntheticdata.acquire_syntheticdata_interface()

import omni.kit.app
app = omni.kit.app.get_app_interface()

from .bot import RBot
from .plantsarea import PlantsArea
from .obstaclesarea import ObstaclesArea

ANGLE_CAMERAS_DOWN = 25


class RArena(XFormPrim):
    def __init__(self, bot1: RBot = None, bot2: RBot = None):
        super().__init__("/World/Arena", "arena", translation=[0, 0, 100])
        self.bot1 = bot1
        self.bot2 = bot2
        self.lights = []

    def populate(self, no_plants: bool = False, no_panels: bool = True):
        size_arena = (2000, 3000)
        wall_width = 10
        wall_height = 200
        prim_path = f"/World/Arena/FloorContainer/floor"
        add_reference_to_stage(
            usd_path="/home/ekter/Documents/to_convert/ground_robocup.usd",
            prim_path=prim_path,
        )
        self.floor_container = XFormPrim(prim_path+"/Cube")
        self.floor_container.set_local_pose(
            translation=[0, 0, -0.5],
            orientation=[0.5, 0.5, 0.5, 0.5],
        )
        # self.plant_body = XFormPrim(prim_path)
        # self.plant_body.set_local_pose(translation=np.array([0,0,-wall_height]),orientation=np.array([0.86, -.5*0.707, -.5*0.707, -.5*0.707]),)
        # self.floor = FixedCuboid(
        #     "/World/Arena/floor",
        #     "floor",
        #     scale=np.array([size_arena[0], size_arena[1], 1]),
        # )
        self.wall_back = FixedCuboid(
            "/World/Arena/wall_back",
            "wall_back",
            translation=np.array([0, -size_arena[1] / 2, wall_height / 2]),
            scale=np.array([size_arena[0], wall_width, wall_height]),
        )
        self.wall_front = FixedCuboid(
            "/World/Arena/wall_front",
            "wall_front",
            translation=np.array([0, size_arena[1] / 2, wall_height / 2]),
            scale=np.array([size_arena[0], wall_width, wall_height]),
        )
        self.wall_left = FixedCuboid(
            "/World/Arena/wall_left",
            "wall_left",
            translation=np.array([-size_arena[0] / 2, 0, wall_height / 2]),
            scale=np.array([wall_width, size_arena[1], wall_height]),
        )
        self.wall_right = FixedCuboid(
            "/World/Arena/wall_right",
            "wall_right",
            translation=np.array([size_arena[0] / 2, 0, wall_height / 2]),
            scale=np.array([wall_width, size_arena[1], wall_height]),
        )
        # self.beg_front_mid = VisualCuboid(
        #     "/World/Arena/Hitboxes/beg_front_mid",
        #     "beg_front_mid",
        #     translation=np.array([]),
        # )
        self.plants_1 = PlantsArea(
            "/World/Arena/plant_area_1",
            "plant_area_1",
            translation=np.array([600, 0, 0]),
            scale=np.array([150, 150, 150]),
        )
        self.plants_2 = PlantsArea(
            "/World/Arena/plant_area_2",
            "plant_area_2",
            translation=np.array([300, 500, 0]),
            scale=np.array([150, 150, 150]),
        )
        self.plants_3 = PlantsArea(
            "/World/Arena/plant_area_3",
            "plant_area_3",
            translation=np.array([-300, 500, 0]),
            scale=np.array([150, 150, 150]),
        )
        self.plants_4 = PlantsArea(
            "/World/Arena/plant_area_4",
            "plant_area_4",
            translation=np.array([-600, 0, 0]),
            scale=np.array([150, 150, 150]),
        )
        self.plants_5 = PlantsArea(
            "/World/Arena/plant_area_5",
            "plant_area_5",
            translation=np.array([-300, -500, 0]),
            scale=np.array([150, 150, 150]),
        )
        self.plants_6 = PlantsArea(
            "/World/Arena/plant_area_6",
            "plant_area_6",
            translation=np.array([300, -500, 0]),
            scale=np.array([150, 150, 150]),
        )
        self.obstacles = ObstaclesArea(
            "/World/Arena/obstacles",
            "obstacles",
        )
        self.body = XFormPrim(
            "/World/Arena/body",
            "body",
            translation=np.array([0, 0, 0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
        )
        prim_path = "/World/Arena/body/robot"
        add_reference_to_stage(
            usd_path="/home/ekter/Documents/Omniverse/polybot/exts/polybot.riviera/polybot/riviera/models/robot-v3.usd",
            prim_path=prim_path,
        )
        self.floor_container = XFormPrim(prim_path+"/Cube")
        self.floor_container.set_local_pose(
            translation=[0, 0, -0.5],
            orientation=[0.5, 0.5, 0.5, 0.5],
        )

        self.camera_front = Camera(
            "/World/Arena/body/camera_front",
            "camera_front",
            translation=np.array([0, 0, 285]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, ANGLE_CAMERAS_DOWN, 0]), degrees=True),
            frequency=20,
            resolution=(640, 480),
        )
        self.camera_front.set_visibility(True)
        self.camera_front.set_focal_length(0.8)
        self.camera_front.initialize()

        self.camera_right = Camera(
            "/World/Arena/body/camera_right",
            "camera_right",
            translation=np.array([0, 0, 285]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, ANGLE_CAMERAS_DOWN, 90]), degrees=True),
            frequency=20,
            resolution=(640, 480),
        )
        self.camera_right.set_visibility(True)
        self.camera_right.set_focal_length(0.8)
        self.camera_right.initialize()

        self.camera_back = Camera(
            "/World/Arena/body/camera_back",
            "camera_back",
            translation=np.array([0, 0, 285]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, ANGLE_CAMERAS_DOWN, 180]), degrees=True),
            frequency=20,
            resolution=(640, 480),
        )
        self.camera_back.set_visibility(True)
        self.camera_back.set_focal_length(0.8)
        self.camera_back.initialize()

        self.camera_left = Camera(
            "/World/Arena/body/camera_left",
            "camera_left",
            translation=np.array([0, 0, 285]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, ANGLE_CAMERAS_DOWN, 270]), degrees=True),
            frequency=20,
            resolution=(640, 480),
        )
        self.camera_left.set_visibility(True)
        self.camera_left.set_focal_length(0.8)
        self.camera_left.initialize()
        for light in self.lights:
            prim_utils.delete_prim(light)
        for _ in range(random.randint(1, 3)):
            self.lights.append(prim_utils.create_prim(
                            f"/World/Arena/Lights/Light_{random.randint(0, 4567382)}",
                            "DistantLight",
                            orientation=np.array(
                                [0.707, random.random()/4 - 0.25, random.random()/4 - 0.25, 0.707]
                                ),
                            attributes={
                                "inputs:intensity": (int(random.expovariate(1/2e3))),
                                "inputs:color": tuple(random.uniform(0, 1) for _ in range(3)),
                            }
                        ).GetPath())
        self.plant_areas = [self.plants_1, self.plants_2, self.plants_3,
                            self.plants_4, self.plants_5, self.plants_6]

        # for i in range(2):
        #     print(f"update n {i}")
        #     app.update()

    def toggle_visibility_(self):
        for plant_area in self.plant_areas:
            plant_area.toggle_visibility()

    def set_facing_dir(self, angle_degrees: float):
        self.body.set_local_pose(orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, angle_degrees]), degrees=True))

    def set_body_pos(self, pos: np.ndarray):
        self.body.set_local_pose(translation=pos)

    def randomize_plants(self):
        for plant_area in self.plant_areas:
            for plant in plant_area.get_plants():
                plant.set_world_pose(position=np.random.random(3)*np.array([1800, 2800, 0])-np.array([900, 1400, -100]))
        self.set_body_pos(np.random.random(3)*np.array([1800, 2800, 0])-np.array([900, 1400, 0]))
        self.set_facing_dir(np.random.random()*360)

    def randomize_obstacles(self):
        for obstacle in self.obstacles.get_obstacles():
            obstacle.set_world_pose(position=np.random.random(3)*np.array([1800, 2800, 0])-np.array([900, 1400, -100]))             # TODO set obstacle to ground
        self.set_body_pos(np.random.random(3)*np.array([1800, 2800, 0])-np.array([900, 1400, 0]))
        self.set_facing_dir(np.random.random()*360)

    def get_plants_poses(self):
        res = []
        for plant_area in self.plant_areas:
            for plant in plant_area.get_plants():
                res.append(plant.get_world_pose())
        return res

    def get_plants_in_camera_ref(self, camera: Camera = None):
        if camera is None:
            camera = self.camera_front
        res = []
        for plant_area in self.plant_areas:
            for plant in plant_area.get_plants():
                pos_b = (plant.get_world_pose()[0]-camera.get_world_pose()[0])*np.array([1, 1, 0])
                r = np.linalg.norm(pos_b)
                phi_b = math.atan2(pos_b[1], pos_b[0])
                phi = (phi_b - rot_utils.quats_to_euler_angles(camera.get_world_pose()[1])[2])
                res.append((r, phi))
        return res                  #TODO normalize and %math.pi*2

    def get_obstacles_in_camera_ref(self, camera: Camera = None):
        if camera is None:
            camera = self.camera_front
        res = []
        for obstacle in self.obstacles.get_obstacles():
            for point in obstacle.get_contour_rectangle_corners():
                pos_b = (point.get_world_pose()[0]-camera.get_world_pose()[0])*np.array([1, 1, 0])
                r = np.linalg.norm(pos_b)
                phi_b = math.atan2(pos_b[1], pos_b[0])
                phi = (phi_b - rot_utils.quats_to_euler_angles(camera.get_world_pose()[1])[2])
                res.append((r, phi))

    def get_cameras(self):
        return [self.camera_front, self.camera_right, self.camera_back, self.camera_left]

    def photo_to(self, img_path: str, camera: Camera = None) -> None:
        # def _get_sensor_data(sensor, dtye):
        #     width = sdi.get_sensor_width(sensor)
        #     height = sdi.get_sensor_height(sensor)
        #     row_size = sdi.get_sensor_row_size(sensor)
        #     return sdi.get_sensor_host_float_texture_array(sensor, width, height, row_size)
        # data = _get_sensor_data(_syntheticdata.SensorType.Rgb, "uint32")
        # image = np.frombuffer(data, dtype=np.uint8).reshape(*data.shape, -1)
        if camera is None:
            camera = self.camera_front
        img = camera.get_current_frame()["rgba"][:, :, :3]
        print(img.shape)
        cv2.imwrite(img_path, img)
        # print(imageio.v3.imread(img_path).shape)
