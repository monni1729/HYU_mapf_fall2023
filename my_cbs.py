from math import fabs
from itertools import combinations
from copy import deepcopy
from a_star import AStar
from heapq import heappop, heappush
import time


class Location:
    def __init__(self, x, y):
        self.x, self.y = x, y

    def __eq__(self, other_loc):
        if isinstance(other_loc, type(self)):
            if self.x == other_loc.x and self.y == other_loc.y:
                return True
            else:
                return False
        else:
            raise TypeError

    def __str__(self):
        return str((self.x, self.y))


class State:
    def __init__(self, time_grid, loc):
        self.time = time_grid
        self.loc = loc

    def __eq__(self, other_state):
        if isinstance(other_state, type(self)):
            if self.time == other_state.time and self.loc == other_state.loc:
                return True
            else:
                return False
        else:
            raise TypeError

    def __hash__(self):
        return hash(f'{self.time}, {self.loc.x}, {self.loc.y}')

    def __str__(self):
        return f'{self.time}, {self.loc.x}, {self.loc.y}'

    def is_equal_location(self, other_state):
        if isinstance(other_state, type(self)):
            if self.loc == other_state.loc:
                return True
            else:
                return False
        else:
            raise TypeError


class Conflict:
    def __init__(self, conf_time, conf_type, a1, a2, loc_1, loc_2):
        self.conf_time = conf_time
        self.conf_type = conf_type
        self.agent_1 = a1
        self.agent_2 = a2
        self.loc_1 = loc_1
        self.loc_2 = loc_2


class VertexConstraint:
    def __init__(self, time, loc):
        self.time = time
        self.loc = loc

    def __eq__(self, other_vertex):
        if isinstance(other_vertex, type(self)):
            if self.time == other_vertex.time and self.loc == other_vertex.loc:
                return True
            else:
                return False
        else:
            raise TypeError

    def __hash__(self):
        return hash(f'{self.time}, {self.loc.x}, {self.loc.y}')


class EdgeConstraint:
    def __init__(self, time, loc_1, loc_2):
        self.time = time
        self.loc_1 = loc_1
        self.loc_2 = loc_2

    def __eq__(self, other_edge):
        if isinstance(other_edge, type(self)):
            if self.time == other_edge.time:
                if self.loc_1 == other_edge.loc_1:
                    if self.loc_2 == other_edge.loc_2:
                        return True
            return False
        else:
            raise TypeError

    def __hash__(self):
        return hash(f'{self.time}, {self.loc_1}, {self.loc_2}')


class Constraints:
    def __init__(self, v_con=None, e_con=None):
        self.vertex_cons = set()
        self.edge_cons = set()
        if v_con:
            self.vertex_cons = {v_con}
        if e_con:
            self.edge_cons = {e_con}

    def add_constraint(self, other_cons):
        if isinstance(other_cons, type(self)):
            self.vertex_cons.update(other_cons.vertex_cons)
            self.edge_cons.update(other_cons.edge_cons)
        else:
            raise TypeError


class Environment:
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension
        self.obstacles = obstacles
        self.agents = agents
        self.agent_dict = {}
        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}
        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []
        if self.is_state_valid(State(state.time + 1, state.loc)):
            neighbors.append(State(state.time + 1, state.loc))

        ns = [State(state.time + 1, Location(state.loc.x, state.loc.y + 1)),  # North
              State(state.time + 1, Location(state.loc.x, state.loc.y - 1)),  # South
              State(state.time + 1, Location(state.loc.x - 1, state.loc.y)),  # West
              State(state.time + 1, Location(state.loc.x + 1, state.loc.y))]  # East

        for n in ns:
            if self.is_state_valid(n):
                if self.is_transition_valid(state, n):
                    neighbors.append(n)

        return neighbors

    def get_first_conflict(self, solution):
        max_t = max([len(each_plan) for each_plan in solution.values()])
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_location(state_2):
                    return Conflict(t, 'VERTEX', agent_1, agent_2, state_1.loc, state_2.loc)

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_1_next = self.get_state(agent_1, solution, t + 1)
                state_2 = self.get_state(agent_2, solution, t)
                state_2_next = self.get_state(agent_2, solution, t + 1)

                if state_1.is_equal_location(state_2_next):
                    if state_1_next.is_equal_location(state_2):
                        return Conflict(t, 'EDGE', agent_1, agent_2, state_1.loc, state_1_next.loc)

        return False

    @staticmethod
    def create_constraints_from_conflict(conflict):
        constraint_dict = {}
        if conflict.conf_type == 'VERTEX':
            v_con = VertexConstraint(conflict.conf_time, conflict.loc_1)
            constraint = Constraints(v_con=v_con)
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.conf_type == 'EDGE':
            e_con1 = EdgeConstraint(conflict.conf_time, conflict.loc_1, conflict.loc_2)
            e_con2 = EdgeConstraint(conflict.conf_time, conflict.loc_2, conflict.loc_1)

            constraint_dict[conflict.agent_1] = Constraints(e_con=e_con1)
            constraint_dict[conflict.agent_2] = Constraints(e_con=e_con2)

        return constraint_dict

    @staticmethod
    def get_state(agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def is_state_valid(self, state):
        if 0 <= state.loc.x < self.dimension[0]:
            if 0 <= state.loc.y < self.dimension[1]:
                if VertexConstraint(state.time, state.loc) not in self.constraints.vertex_cons:
                    if (state.loc.x, state.loc.y) not in self.obstacles:
                        return True

        return False

    def is_transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.loc, state_2.loc) not in self.constraints.edge_cons

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.loc.x - goal.loc.x) + fabs(state.loc.y - goal.loc.y)

    def is_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_location(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)

            if local_solution:
                solution.update({agent: local_solution})
            else:
                return False

        return solution


def compute_cost(solution):
    return sum([len(path) for path in solution.values()])


class HighLevelNode:
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other_node):
        if isinstance(other_node, type(self)):
            if self.solution == other_node.solution:
                if self.cost == other_node.cost:
                    return True
            return False
        else:
            raise TypeError

    def __hash__(self):
        return self.cost

    def __lt__(self, other):
        return self.cost < other.cost


class CBS:
    def __init__(self, env):
        self.env = env
        self.open_set = []
        self.closed_set = set()

    def search(self) -> dict:
        initial_node = HighLevelNode()
        for agent in self.env.agent_dict.keys():
            initial_node.constraint_dict[agent] = Constraints()
        initial_node.solution = self.env.compute_solution()
        initial_node.cost = compute_cost(initial_node.solution)
        if not initial_node.solution:
            return dict()

        self.open_set.append(initial_node)
        while self.open_set:
            # print(len(self.open_set), len(self.closed_set))
            curr = heappop(self.open_set)
            self.closed_set.update({curr})

            self.env.constraint_dict = curr.constraint_dict
            conflict_dict = self.env.get_first_conflict(curr.solution)

            if not conflict_dict:
                print("Solution found")
                return curr.solution

            else:
                constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

                for agent in constraint_dict.keys():
                    new_node = deepcopy(curr)
                    new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                    self.env.constraint_dict = new_node.constraint_dict
                    new_node.solution = self.env.compute_solution()
                    if new_node.solution:
                        new_node.cost = compute_cost(new_node.solution)
                        if new_node not in self.closed_set:
                            heappush(self.open_set, new_node)

        print("No solution")
        return dict()


def main():
    dimension = [5, 5]
    obstacles = [(0, 0), (0, 2), (0, 4),
                 (2, 0), (2, 2), (2, 4),
                 (4, 0), (4, 2), (4, 4), ]
    agents = [{'start': [0, 1], 'goal': [4, 1], 'name': 'agent_0'},
              {'start': [0, 3], 'goal': [4, 3], 'name': 'agent_1'},
              {'start': [4, 1], 'goal': [0, 1], 'name': 'agent_2'},
              {'start': [4, 3], 'goal': [0, 3], 'name': 'agent_3'},
              {'start': [1, 0], 'goal': [1, 4], 'name': 'agent_4'},
              {'start': [3, 0], 'goal': [3, 4], 'name': 'agent_5'}]

    env = Environment(dimension, agents, obstacles)

    cbs = CBS(env)
    solution = cbs.search()
    if solution:
        for agent, path in solution.items():
            print([{'t': state.time, 'x': state.loc.x, 'y': state.loc.y} for state in path])

        print(f'Cost: {compute_cost(solution)}')
    else:
        print("Solution not found")
        return


if __name__ == "__main__":
    t1 = time.time()
    main()
    print('Computation time: ', time.time() - t1)
