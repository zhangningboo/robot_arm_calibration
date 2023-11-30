from enum import Enum


class Mode(Enum):
    Auto = 0
    Hand = 1


class BlockMode(Enum):
    Yes = 0
    No = 1


class CollisionMode(Enum):
    Level = 0
    Percentage = 1


class YesOrNo(Enum):
    Yes = 1
    No = 0


class CollisionStrategy(Enum):
    StopAndThrowError = 0
    IgnoreAndKeepRunning = 1


class CoordinateRelType(Enum):
    AbsoluteBaseCoordinate = 0
    RelBaseCoordinate = 1
    RelToolCoordinate = 2


