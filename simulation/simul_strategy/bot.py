from coords import Coord
from area import RectangleArea
from PySide6.QtGui import QPainter


class Bot:
    def __init__(self, team: str):
        self.team: str = team
        self.pos: Coord = Coord(0, 0)
        self.hitbox = RectangleArea(self.pos-Coord(150,150), Coord(300, 300))

    def move(self, new_pos: Coord) -> None:
        self.pos = new_pos

    def draw(self, painter: QPainter) -> None:
        painter.drawEllipse(self.pos.abs-150, self.pos.ord-150, 300, 300)
        painter.drawText(self.pos.abs, self.pos.ord, self.team)
        painter.drawRect(self.pos.abs-150, self.pos.ord-150, 300, 300)

    def act(self) -> None:
        if self.team == "blue":
            self.move(self.pos + Coord(1, 0))
        else:
            self.move(self.pos + Coord(-1, 0))
