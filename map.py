#from PythonClientAPI.libs.Game.Gameboard import Gameboard
#from PythonClientAPI.libs.Game.MapOutOfBoundsException import *
import numpy as np
from .utils import *

class CostMap(object):

    def __init__(self):
        super(CostMap, self).__init__()
        self.gameboard = None
        self.turret_cost_cache = {}

    def initialize(self, gameboard):
        self.gameboard = gameboard

    def update_costs(self, gameboard):
        self.gameboard = gameboard

        self.width = gameboard.width
        self.height = gameboard.height

        self.cost_tensor = [self.generate_cost_map(0)]

    def generate_bullet_cost_map(self, turn):
        pass

    def generate_turret_cost_map(self, turn):
        cost_map = np.zeros((self.width, self.height), dtype=np.int)

        future_turn = self.gameboard.current_turn + turn

        for turret in self.gameboard.turrets:
            # Determine whether turret is firing this turn
            firing_period = turret.fire_time + turret.cooldown_time
            cycle_position = (future_turn-1) % firing_period

            # If turret is firing, update the cost map
            if (cycle_position < turret.fire_time):
                for d in DirectionUtils.direction_delta_map.values():
                    for i in range(4):
                        px, py = self.wrap_coordinates(turret.x + d[0]*i, turret.y + d[1]*i)
                        if (self.gameboard.wall_at_tile[px][py]):
                            break
                        cost_map[px][py] += 10
        return cost_map

    def generate_cost_map(self, turn):
        return self.turret_cost_cache.setdefault(self.gameboard.current_turn + turn, self.generate_turret_cost_map(turn))

    def cost(self, x, y, turn):
        cost_map = None
        if (turn < len(self.cost_tensor)):
            cost_map = self.cost_tensor[turn]
        elif(len(self.cost_tensor) == turn):
            cost_map = self.generate_cost_map(turn)
            self.cost_tensor.append(cost_map)
        else:
            print(turn, len(self.cost_tensor))
        return cost_map[x][y]

    def is_valid(self, x,y):
        return x >= 0 and x < self.width and y >= 0 and y < self.height

    def wrap_coordinates(self, x, y):
        return (x%self.width, y%self.height)

    def is_valid_state(self, state):
        return self.is_valid(state.x, state.y) and self.can_move_to(state)

    def can_move_to(self, state):
        return not self.gameboard.wall_at_tile[state.x][state.y] and not self.gameboard.turret_at_tile[state.x][state.y]
