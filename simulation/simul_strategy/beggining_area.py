from PySide6.QtGui import QPainter, QColor
from coords import Coord
from area import RectangleArea

class BegginingArea(RectangleArea):
    def __init__(self, top_left: Coord, size: float, color:str) -> None:
        super().__init__(top_left, size)
        self.plants_number: int = 0
        self.color: str = color

    def remove_plant(self) -> bool:
        if not self.is_empty():
            self.plants_number -= 1
            return True
        return False

    def add_plant(self) -> bool:
        self.plants_number += 1
        return True

    def is_empty(self) -> bool:
        return self.plants_number == 0

    def get_plants_number(self) -> int:
        return self.plants_number

    def __str__(self) -> str:
        return f"Begg area at {str(self.top_left)} with : plants[{str(self.plants_number)}]"

    def draw(self, painter: QPainter, **kwargs) -> None:
        painter.setBrush(QColor(self.color))
        super().draw(painter, **kwargs)
        painter.drawText(self.top_left.abs, self.top_left.ord+30, str(self))
