from PySide6.QtCore import Slot, Signal, Qt, QCoreApplication, QRectF
from PySide6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QMainWindow,
    QMenu,
    QMenuBar,
    QVBoxLayout,
    QWidget,
    QTableView,
    QProgressDialog,
)
from PySide6.QtGui import QPainter, QPaintEvent
from coords import Coord
from area import Area, CircleArea, RectangleArea
from bot import Bot

class IArena(QWidget):

    def __init__(self, bot1: Bot, bot2: Bot) -> None:
        QWidget.__init__(self)

        self.bots: list[Bot] = [bot1, bot2]

        self.entire_area: RectangleArea = RectangleArea(Coord(0, 0), Coord(3000, 2000))

        self.beggining_blue1 = RectangleArea(Coord(0, 0), Coord(450, 450))
        self.beggining_blue2 = RectangleArea(Coord(3000-450, 1000-450/2), Coord(450, 450))
        self.beggining_blue_res = RectangleArea(Coord(0, 2000-450), Coord(450, 450))
        self.beggining_yellow1 = RectangleArea(Coord(3000-450, 0), Coord(450, 450))
        self.beggining_yellow2 = RectangleArea(Coord(0, 1000-450/2), Coord(450, 450))
        self.beggining_yellow_res = RectangleArea(Coord(3000-450, 2000-450), Coord(450, 450))

        self.beggining_areas: list[Area] = [self.beggining_blue1, self.beggining_blue2, self.beggining_blue_res, self.beggining_yellow1, self.beggining_yellow2, self.beggining_yellow_res]

        self.plants_1 = CircleArea(Coord(1500, 1600), 150)
        self.plants_2 = CircleArea(Coord(2000, 1300), 150)
        self.plants_3 = CircleArea(Coord(2000, 700), 150)
        self.plants_4 = CircleArea(Coord(1500, 400), 150)
        self.plants_5 = CircleArea(Coord(1000, 700), 150)
        self.plants_6 = CircleArea(Coord(1000, 1300), 150)

        self.plants: list[Area] = [self.plants_1, self.plants_2, self.plants_3, self.plants_4, self.plants_5, self.plants_6]

        # ...


    def paintEvent(self, event=None) -> None:
        painter = QPainter(self)
        self.entire_area.draw(painter)
        for area in self.beggining_areas:
            area.draw(painter)
        for area in self.plants:
            area.draw(painter)

        for bot in self.bots:
            bot.draw(painter)


    def update_bots(self) -> None:
        for bot in self.bots:
            bot.act()
        self.update()