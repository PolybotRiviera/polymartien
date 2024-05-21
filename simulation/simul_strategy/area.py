from typing import Union
import math

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QPainter, QColor

from coords import Coord


class Area():
    def __init__(self, points:list[Coord], rect: QRectF, method = QPainter.drawRect) -> None:
        self.points: list[Coord] = points
        self.rect: QRectF = rect
        rect.top()
        self.method = method


    def __contains__(self, item: Union[Coord, "Area", "Bot"]) -> bool:
        pass

    def draw(self, painter: QPainter, **kwargs) -> None:
        color = "alpha"
        for item in kwargs:
            if kwargs[item] in self:
                color: str = item
        painter.setBrush(QColor(color))
        self.method(painter, self.rect)
        painter.setBrush(Qt.GlobalColor.transparent)


class CircleArea(Area):
    def __init__(self, center: Coord, radius: float):
        self.center: Coord = center
        self.radius: float = radius
        Area.__init__(self,
            [center+Coord(
                math.sin(angle/180*math.pi),
                math.cos(angle/180*math.pi)
            )*radius for angle in range(0, 360)],
            QRectF(center.abs - radius, center.ord - radius, radius * 2, radius * 2),
            QPainter.drawEllipse)

    def __contains__(self, item: Union[Coord, Area]) -> bool:
        if isinstance(item, Coord):
            return (item - self.center).size() <= self.radius
        if isinstance(item, RectangleArea):
            return any(point in self for point in item.points) or any(point in item for point in self.points)
        if isinstance(item, CircleArea):
            return (item.center - self.center).size() + item.radius <= self.radius
        return item.hitbox in self

    def dist_to(self, item: Union["RectangleArea", "CircleArea", Coord]) -> float:
        if isinstance(item, Coord):
            return max((item - self.center).size() - self.radius,0)
        if isinstance(item, RectangleArea):
            return min(self.dist_to(point) for point in item.points)
        if isinstance(item, CircleArea):
            return max((item.center - self.center).size() - self.radius - item.radius, 0)
        raise TypeError(f"dist_to() argument must be Coord, RectangleArea or CircleArea, not {type(item)}")


class RectangleArea(Area):
    def __init__(self, top_left: Coord, size: Coord):
        self.top_left: Coord = top_left
        self.bottom_right: Coord = top_left + size
        self.center: Coord = top_left + size / 2
        Area.__init__(self, [top_left, top_left + Coord(size.abs, 0), top_left + Coord(0, size.ord), top_left + size], QRectF(top_left.abs, top_left.ord, size.abs, size.ord), QPainter.drawRect)

    def __contains__(self, item: Union[Coord, Area]) -> bool:
        if isinstance(item, Coord):
            return (
            self.top_left.abs <= item.abs <= self.bottom_right.abs
            and self.top_left.ord <= item.ord <= self.bottom_right.ord
        )
        if isinstance(item, RectangleArea):
            return any(point in self for point in item.points)
        if isinstance(item, CircleArea):
            return any(point in item for point in self.points)
        return item.hitbox in self

    def dist_to(self, item: Union["RectangleArea", "CircleArea", Coord]) -> float:
        if isinstance(item, Coord):
            return min((point-item).size() for point in self.points)
        if isinstance(item, RectangleArea):
            return min(self.top_left - self.bottom_right, self.bottom_right - self.top_left, self.bottom_right.coin1(self.top_left) - self.bottom_right.coin2(self.top_left), self.bottom_right.coin2(self.top_left) - self.bottom_right.coin1(self.top_left), key = lambda coord: coord.size())
        if isinstance(item, CircleArea):
            return item.dist_to(self)
        raise TypeError(f"dist_to() argument must be Coord, RectangleArea or CircleArea, not {type(item)}")
