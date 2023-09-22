from typing import Union

from PySide6.QtCore import QRectF
from PySide6.QtGui import QPainter

from coords import Coord


class Area():
    def __contains__(self, item: Union[Coord, "Area", "Bot"]) -> bool:
        pass

    def draw(self, painter: QPainter, *args, **kwargs) -> None:
        self.method(painter, self.rect, *args, **kwargs)


class CircleArea(Area):
    def __init__(self, center: Coord, radius: float):
        self.center: Coord = center
        self.radius: float = radius
        self.points: list[Coord] = [center]
        self.rect = QRectF(center.abs - radius, center.ord - radius, radius * 2, radius * 2)
        self.method = QPainter.drawEllipse

    def __contains__(self, item: Union[Coord, Area, "Bot"]) -> bool:
        if isinstance(item, Coord):
            return (item - self.center).__len__() <= self.radius
        if isinstance(item, RectangleArea):
            return all(point in self for point in item.points)
        if isinstance(item, CircleArea):
            return (item.center - self.center).__len__() + item.radius <= self.radius
        if isinstance(item, Bot):
            return item.hitbox in self
        else:
            raise TypeError(f"Cannot check if {item}, type {type(item)}, is in {self}")


class RectangleArea(Area):
    def __init__(self, top_left: Coord, size: Coord):
        self.top_left: Coord = top_left
        self.bottom_right: Coord = top_left + size
        self.points: list[Coord] = [top_left, top_left + Coord(size.abs, 0), top_left + Coord(0, size.ord), top_left + size]
        self.rect = QRectF(top_left.abs, top_left.ord, size.abs, size.ord)
        self.method = QPainter.drawRect

    def __contains__(self, item: Union[Coord, Area, "Bot"]) -> bool:
        if isinstance(item, Coord):
            return (
            self.top_left.abs <= item.abs <= self.bottom_right.abs
            and self.top_left.ord <= item.ord <= self.bottom_right.ord
        )
        if isinstance(item, RectangleArea):
            return all(point in self for point in item.points)
        if isinstance(item, CircleArea):
            return all(point in item for point in self.points)
