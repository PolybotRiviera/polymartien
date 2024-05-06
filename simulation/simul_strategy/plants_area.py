from PySide6.QtGui import QPainter
from coords import Coord
from area import CircleArea

class PlantsArea(CircleArea):
    def __init__(self, center: Coord, radius: float, plants_number: int):
        super().__init__(center, radius)
        self.plants_number: int = plants_number

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
        return f"Plants[{str(self.plants_number)}] at {str(self.center)}"

    def draw(self, painter: QPainter, **kwargs) -> None:
        super().draw(painter, **kwargs)
        painter.drawText(self.center.abs, self.center.ord, str(self))
