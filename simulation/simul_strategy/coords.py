import math
from typing import Generator, Any, Literal, Union


class Coord():
    """Vec2D object, created by rectangular or polar coordinates(with float coords in normal
    condition, but if broken, can have floats.(used for the moving anims)"""

    def __init__(self, abscisse: Union[float, Any], ordonnee: Union[float, None] = None) -> None:
        if ordonnee is None:
            self.abs: float = abscisse.x()
            self.ord: float = abscisse.y()
        else:
            self.abs: float = abscisse
            self.ord: float = ordonnee

    def __repr__(self) -> str:
        return "<" + str(self.abs) + "," + str(self.ord) + ">"

    def __eq__(self, other: "Coord") -> bool:
        return (
            self.abs == other.abs and self.ord == other.ord
            if isinstance(other, Coord)
            else self.size() == other
        )

    def __ne__(self, other: "Coord") -> bool:
        return not self == other

    def __add__(self, other: "Coord") -> "Coord":
        if isinstance(other, Coord):
            return Coord(self.abs + other.abs, self.ord + other.ord)
        return Coord(self.abs + other, self.ord + other)

    def __neg__(self) -> "Coord":
        return Coord(-self.abs, -self.ord)

    def __mul__(self, other: "Coord") -> "Coord":
        if isinstance(other, Coord):
            return Coord(self.abs * other.abs, self.ord * other.ord)
        return Coord(self.abs * other, self.ord * other)

    def __sub__(self, other: "Coord") -> "Coord":
        if isinstance(other, Coord):
            return Coord(self.abs - other.abs, self.ord - other.ord)
        return Coord(self.abs - other, self.ord - other)

    def __abs__(self) -> "Coord":
        return Coord(abs(self.abs), abs(self.ord))

    def __floordiv__(self, other: "Coord") -> "Coord":
        if isinstance(other, Coord):
            return Coord(self.abs / other.abs, self.ord / other.ord)
        return Coord(self.abs / other, self.ord / other)

    def __truediv__(self, other: "Coord") -> "Coord":
        if isinstance(other, Coord):
            return Coord(self.abs / other.abs, self.ord / other.ord)
        return Coord(self.abs / other, self.ord / other)

    def size(self) -> float:
        return math.sqrt(self.abs ** 2 + self.ord ** 2)

    def __lt__(self, other: "Coord") -> bool:
        if isinstance(other, Coord):
            return self.size() < other.size()
        return self.size() < other

    def __gt__(self, other: "Coord") -> bool:
        if isinstance(other, Coord):
            return self.size() > other.size()
        return self.size() > other

    def __le__(self, other: "Coord") -> bool:
        if isinstance(other, Coord):
            return self.size() <= other.size()
        return self.size() <= other

    def __ge__(self, other: "Coord") -> bool:
        if isinstance(other, Coord):
            return self.size() >= other.size()
        return self.size() >= other

    def __iter__(self) -> Generator[float, Any, None]:
        for i in (self.abs, self.ord):
            yield i

    def __hash__(self) -> float:
        return hash((self.abs, self.ord))

    def getangle(self) -> float:
        """returns the angle of the vector (self.abs, self.ord)"""
        return math.atan2(self.ord, self.abs)

    def ind(self) -> tuple:
        """returns a tuple (abs,ord). Used for """
        return self.abs, self.ord

    def distance(self, other: "Coord") -> float:
        "Diagonal distance between two points"
        return (self - other).size()

    def dirtrig(self):
        "Direction from the center to a points"
        if self == Coord(0, 0):
            return Coord(0, 0)
        cos = self.abs / self.size()
        if cos > 1 / math.sqrt(2) - 0.1:
            return Coord(-1, 0)
        if cos < -1 / math.sqrt(2) + 0.1:
            return Coord(1, 0)
        if self.ord > 0:
            return Coord(0, -1)
        return Coord(0, 1)

    def normalized(self):
        return self / self.size()

    def cosinus(self, other: "Coord") -> float:
        """returns the cosine of a coord (adj/hyp)"""
        return (self - other).abs / (self - other).size()

    def direction(self, other: "Coord") -> "Coord":
        "Direction from a point to another"
        return (self - other).dirtrig()

    def inverse(self):
        "Inverse of a Coord(swapping x and y)"
        return Coord(self.ord, self.abs)

    def coin1(self, other: "Coord"):
        "First combinaison of two Coords"
        return Coord(self.abs, other.ord)

    def coin2(self, other: "Coord"):
        "Second combinaison of two Coords"
        return Coord(other.abs, self.ord)

    def middle(self, other: "Coord"):
        "Middle of two Coords"
        return (self + other) // 2

    def facing(self) -> Literal[1, 2, 3, 0]:
        "Function to choose the correct direction of an image"
        direction = self.dirtrig()
        if direction == Coord(0, 1):
            return 1
        if direction == Coord(1, 0):
            return 2
        if direction == Coord(-1, 0):
            return 3
        return 0
