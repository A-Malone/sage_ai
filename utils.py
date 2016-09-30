import math
from PythonClientAPI.libs.Game.Enums import *
from functools import total_ordering

class DirectionUtils(object):
    direction_move_map = {
        Direction.LEFT : Move.FACE_LEFT,
        Direction.RIGHT : Move.FACE_RIGHT,
        Direction.UP : Move.FACE_UP,
        Direction.DOWN : Move.FACE_DOWN
    }
    direction_delta_map = {
        Direction.LEFT : (-1, 0),
        Direction.RIGHT : (1, 0),
        Direction.UP : (0, -1),
        Direction.DOWN : (0, 1)
    }
    direction_angle_map = {
        Direction.LEFT : math.pi,
        Direction.RIGHT : 0,
        Direction.UP : math.pi / 2,
        Direction.DOWN : 3 * math.pi / 2
    }

class State(object):
    def __init__(self, x, y, orientation, move):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.move = move

    def satisfies(self, end):
        success =  self.x == end.x
        success &= self.y == end.y
        success &= (end.orientation == None or self.orientation == end.orientation)
        success &= (end.move == None or self.move == end.move)
        return success

    def __hash__(self):        
        return self.x.__hash__() ^ self.y.__hash__() ^ self.orientation.value.__hash__() ^ self.move.value.__hash__()

    def __eq__(self, other):
        return other and self.x == other.x and self.y == other.y and self.orientation == other.orientation and self.move == other.move

    def __str__(self):
        return "<State: {} {} {} {}>".format(self.x, self.y, self.orientation, self.move)

    def __repr__(self):
        return self.__str__()
