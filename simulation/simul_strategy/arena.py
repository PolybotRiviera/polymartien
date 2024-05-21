import random

from PySide6.QtCore import Slot, Signal, Qt, QRect
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter

from coords import Coord
from area import Area, RectangleArea
from plants_area import PlantsArea
from beggining_area import BegginingArea

class IArena(QWidget):

    def __init__(self, bot1: "Bot", bot2: "Bot") -> None:
        QWidget.__init__(self)

        self.bots: list[Bot] = [bot1, bot2]

        self.total_size: QRect = QRect(0, 0, 3000, 2000)
        self.entire_area: RectangleArea = RectangleArea(Coord(self.total_size.bottomLeft()), Coord(self.total_size.topLeft()))

        self.beggining_blue1 = BegginingArea(Coord(0, 0), Coord(450, 450), "blue")
        self.beggining_blue2 = BegginingArea(Coord(3000-450, 1000-450/2), Coord(450, 450), "blue")
        self.beggining_blue_res = BegginingArea(Coord(0, 2000-450), Coord(450, 450), "blue")
        self.beggining_yellow1 = BegginingArea(Coord(3000-450, 0), Coord(450, 450), "yellow")
        self.beggining_yellow2 = BegginingArea(Coord(0, 1000-450/2), Coord(450, 450), "yellow")
        self.beggining_yellow_res = BegginingArea(Coord(3000-450, 2000-450), Coord(450, 450), "yellow")

        self.beggining_areas: list[Area] = [self.beggining_blue1, self.beggining_blue2, self.beggining_blue_res, self.beggining_yellow1, self.beggining_yellow2, self.beggining_yellow_res]

        self.plants_1 = PlantsArea(Coord(1500, 1600), 150, 6)
        self.plants_2 = PlantsArea(Coord(2000, 1300), 150, 6)
        self.plants_3 = PlantsArea(Coord(2000, 700), 150, 6)
        self.plants_4 = PlantsArea(Coord(1500, 400), 150, 6)
        self.plants_5 = PlantsArea(Coord(1000, 700), 150, 6)
        self.plants_6 = PlantsArea(Coord(1000, 1300), 150, 6)

        self.plants: list[PlantsArea] = [self.plants_1, self.plants_2, self.plants_3, self.plants_4, self.plants_5, self.plants_6]

    def paintEvent(self, event=None) -> None:
        painter = QPainter(self)
        painter.setWindow(self.total_size)
        painter.setViewport(QRect(0, 0, self.width(), self.height()))
        self.entire_area.draw(painter)
        for area in self.beggining_areas:
            area.draw(painter, blue=self.bots[0], yellow=self.bots[1])
        for plant in self.plants:
            plant.draw(painter, blue=self.bots[0], yellow=self.bots[1])

        for bot in self.bots:
            bot.draw(painter)


    def update_bots(self) -> None:
        for bot in self.bots:
            bot.act(self)
        self.update()




class Bot:
    def __init__(self, team: str):
        self.team: str = team
        self.pos: Coord = Coord(0, 0)
        self.hitbox = RectangleArea(self.pos-Coord(150,150), Coord(300, 300))
        self.plants: int = 0
        self.max_plants: int = random.randint(2, 10)
        self.speed: float = 10
        self.strategy = random.randint(0, 2)
        self.obj = "plant"

    def move(self, new_pos: Coord) -> None:
        # print(new_pos)
        self.pos = new_pos
        self.hitbox = RectangleArea(self.pos-Coord(150,150), Coord(300, 300))

    def draw(self, painter: QPainter) -> None:
        painter.setBrush(Qt.transparent)
        painter.drawEllipse(self.pos.abs-150, self.pos.ord-150, 300, 300)
        painter.drawText(self.pos.abs, self.pos.ord, str(self))
        painter.drawRect(self.pos.abs-150, self.pos.ord-150, 300, 300)

    def closest_plant(self, arena:IArena) -> PlantsArea | None:
        plant = min(arena.plants, key=lambda plant: ((self.hitbox.dist_to(plant)) if plant.get_plants_number()>0 else 100000))
        if plant.get_plants_number()>0:
            return plant
        else:
            return None

    def closest_beggining(self, arena:IArena, color:str) -> BegginingArea:
        return min(arena.beggining_areas, key=lambda area: ((area.center-self.pos).size() if area.color==color else 100000))

    def act(self, arena:IArena) -> None:
        if self.strategy==2:
            if self.plants == 0:
                self.obj = "plant"
            if self.plants == self.max_plants:
                plant=self.closest_plant(arena)
                if plant is None or plant.get_plants_number()==0:
                    self.obj = "beggining"
                else:
                    self.obj = "plant"
            if self.obj == "plant":
                goal = self.closest_plant(arena)
            else:
                goal = self.closest_beggining(arena, self.team)

        if self.strategy==1:
            if self.plants < self.max_plants:
                for _ in range(10):
                    goal = self.closest_plant(arena)
                    if goal is None:
                        goal = self.closest_beggining(arena, self.team)
                    if self in goal:
                        if self.plants < self.max_plants and goal.remove_plant():
                            self.plants += 1
                        else:
                            goal = self.closest_beggining(arena, self.team)
            else:
                goal = self.closest_beggining(arena, self.team)
        if self.strategy==0:
            if self.plants < self.max_plants:
                goal = self.closest_plant(arena)
            else:
                goal = self.closest_beggining(arena, self.team)
        if goal is None:
            goal = self.closest_beggining(arena, self.team)
        if self in goal:
            if isinstance(goal, PlantsArea):
                if self.plants<self.max_plants  and goal.remove_plant():
                    self.plants += 1
            elif isinstance(goal, BegginingArea):
                if self.plants>0 and goal.add_plant():
                    self.plants -= 1
        else:
            self.move(self.pos + (goal.center-self.pos).normalized()*self.speed)

        # if self.team == "blue":
        #     self.move(self.pos + Coord(1, 0))
        # else:
        #     self.move(self.pos + Coord(-1, 0))

    def __str__(self) -> str:
        return f"Bot[{self.team}] with {self.plants} plants"