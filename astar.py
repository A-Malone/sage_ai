from queue import Queue, PriorityQueue
import sys

from .utils import *


# Modified manhattan distance
def manhattan_rampage(state1, state2):
    dx = (state2.x - state1.x)
    dy = (state2.y - state1.y)

    start_angle =  DirectionUtils.direction_angle_map[state1.orientation]
    turn_angle = math.atan(dy/dx) + math.pi if dy < 0 else 0
    end_angle = DirectionUtils.direction_angle_map[state1.orientation]

    turn_cost = 0

    start_turn = abs(start_angle - turn_angle)
    if (start_turn > math.pi):
        start_turn = 2*math.pi - start_turn
    turn_cost += math.floor(start_turn / (math.pi / 2))

    end_turn = abs(turn_angle - end_angle)
    if (end_turn > math.pi):
        end_turn = 2*math.pi - start_turn
    turn_cost += math.floor((end_turn % (2*math.pi)) / (math.pi / 2))

    return abs(dx) + abs(dy) + turn_cost

# Cartesian distance
def cartesian(state1, state2):
    return math.sqrt((state2.x - state1.x)**2 + (state2.y - state1.y)**2)

# Path cost
def path_cost(path, cost_map):
    cost = 0
    for state in path:
        cost += step_cost(None, state, cost_map)
    return cost

# Cost of a single step
def step_cost(first, second, cost_map, turn):
    return 1 + cost_map.cost(second.x, second.y, turn)

def reconstruct_path(came_from, state_data_map, end):
    state = end
    path = []
    while (state in came_from):
        new_state = came_from[state]
        # Take no moves into consideration
        if(state_data_map[state].turn - state_data_map[new_state].turn > 1):
            print("Waited")
        for i in range(0, state_data_map[state].turn - state_data_map[new_state].turn):
            path.append(state)
        state = new_state

    return path[::-1]

class Path(object):
    """docstring for Path"""
    def __init__(self, path_list, cost):
        super(Path, self).__init__()
        self.path_list = path_list
        self.cost = cost

@total_ordering
class AStarData(object):
    def __init__(self, state, g, h, turn):
        self.state = state
        self.g = g
        self.h = h
        self.turn = turn

    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f() < other.f()

class AStar(object):
    """docstring for AStar"""

    def __init__(self):
        super(AStar, self).__init__()

        self.open_set = set()
        self.closed_set = set()

        # Stores astardata references
        self.open_queue = PriorityQueue()

        self.came_from = {}

        self.state_data_map = {}

    def get_neighbours(self, state, cost_map):
        # Add in turning
        neighbours = [State(state.x, state.y, Direction(o), DirectionUtils.direction_move_map[Direction(o)]) for o in range(1, len(Direction.__members__)+1) if o != state.orientation.value]

        # Add in no move
        neighbours.append(State(state.x, state.y, state.orientation, Move.NONE))

        # Add in going straight forward
        delta = DirectionUtils.direction_delta_map[state.orientation]

        x, y = cost_map.wrap_coordinates(state.x + delta[0], state.y + delta[1])
        move_state = State(x, y, state.orientation, Move.FORWARD)
        if (cost_map.is_valid_state(move_state)):
            neighbours.append(move_state)

        return neighbours

    def get_or_create_data(self, state):
        data = self.state_data_map.get(state, None)
        if (data == None):
            data = AStarData(state, 0, 0, 0)
            self.state_data_map[state] = data

        return data

    def heuristic(self, state1, state2):
        return cartesian(state1, state2)

    def get_path(self, start, end, cost_map):
        start_data = AStarData(start, 0, self.heuristic(start, end), 0)
        self.state_data_map[start] = start_data
        self.open_queue.put(start_data)

        self.open_set.add(start)

        path = None

        while (not self.open_queue.empty()):
            # Get the first item in the min-heap
            current_data = self.open_queue.get();
            current = current_data.state
            self.open_set.remove(current);

            # Short-circuit for the end
            if (current.satisfies(end)):
                path = Path(reconstruct_path(self.came_from, self.state_data_map, current), current_data.g)
                break

            self.closed_set.add(current);

            for neighbour in self.get_neighbours(current, cost_map):

                # If it's in the closed set skip
                if (neighbour in self.closed_set):
                    continue

                # The distance from start to goal passing through current and the neighbour.
                neighbour_data = self.get_or_create_data(neighbour)

                tentative_g_score = current_data.g + step_cost(current, neighbour, cost_map, current_data.turn + 1)
                if (neighbour in self.open_set and tentative_g_score >= neighbour_data.g):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.came_from[neighbour] = current;

                neighbour_data.g = tentative_g_score
                neighbour_data.h = self.heuristic(neighbour, end)
                neighbour_data.turn = current_data.turn + 1

                if (not neighbour in self.open_set):
                    self.open_set.add(neighbour)
                    self.open_queue.put(neighbour_data)

        if (path == None):
            print("No solution")

        return path
