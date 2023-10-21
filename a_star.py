class AStar:
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.heuristic = env.admissible_heuristic
        self.is_goal = env.is_goal
        self.get_neighbors = env.get_neighbors

    def search(self, agent_name):
        initial_state = self.agent_dict[agent_name]["start"]

        closed_set = set()
        open_set = {initial_state}

        came_from = dict()
        g_score = {initial_state: 0}
        f_score = {initial_state: self.heuristic(initial_state, agent_name)}

        while open_set:
            temp_dict = {open_item: f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            curr = min(temp_dict, key=temp_dict.get)

            if self.is_goal(curr, agent_name):
                return reconstruct_path(came_from, curr)

            else:
                open_set.difference_update({curr})
                closed_set.update({curr})
                for neighbor in self.get_neighbors(curr):
                    if neighbor not in closed_set:
                        tentative_g_score = g_score.setdefault(curr, float("inf")) + 1

                        if neighbor not in open_set:
                            open_set.update({neighbor})
                            came_from[neighbor] = curr
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, agent_name)

        return False


def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from.keys():
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]
