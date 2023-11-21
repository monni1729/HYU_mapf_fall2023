from my_cbs import Environment, compute_cost, CBS
import matplotlib.pyplot as plt
import numpy as np
import time


def vis(obstacles, solution):
    ccc = ['b', 'r', 'g', 'c', 'm', 'y']
    max_t = max([len(s) for s in solution.values()])
    positions = np.zeros((len(solution), (max_t - 1) * 10 + 1, 2), dtype=float)

    for i, s in enumerate(solution.values()):
        for j in range(len(s) - 1):
            for k in range(10):
                positions[i, j * 10 + k, 0] = (s[j].loc.x * (10 - k) + s[j + 1].loc.x * k) / 10
                positions[i, j * 10 + k, 1] = (s[j].loc.y * (10 - k) + s[j + 1].loc.y * k) / 10

        for j in range(len(s) - 1, max_t - 1):
            for k in range(10):
                positions[i, j * 10 + k, 0] = s[-1].loc.x
                positions[i, j * 10 + k, 1] = s[-1].loc.y

        positions[i, -1, 0] = s[-1].loc.x
        positions[i, -1, 1] = s[-1].loc.y

    plt.figure(figsize=(6, 6))
    board = plt.axes(xlim=(-0.5, 4.5), ylim=(-0.5, 4.5))

    for f in range((max_t - 1) * 10 + 1):
        circles_rects = []
        for i in range(len(solution)):
            circles_rects.append(plt.Circle((positions[i, f, 0], positions[i, f, 1]),
                                            0.25, facecolor=ccc[i], fill=True, edgecolor='black'))
        for xx, yy in obstacles:
            circles_rects.append(plt.Rectangle((xx - 0.5, yy - 0.5), 1, 1,
                                               linewidth=0, edgecolor=None, facecolor='black'))

        for cr in circles_rects:
            board.add_patch(cr)

        plt.draw()

        plt.pause(0.1)
        # plt.savefig(fname='./fig_' + str(f).zfill(2) + '.png', dpi=200)
        for cr in circles_rects:
            cr.remove()

    time.sleep(1)


def main():
    dimension = [5, 5]
    obstacles = [(0, 0), (0, 2), (0, 4),
                 (2, 0), (2, 2), (2, 4),
                 (4, 0), (4, 2), (4, 4)]

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
        for k, v in solution.items():
            print(k, end='\t')
            for vv in v:
                print(f't{vv.time}->{vv.loc}', end=' ')
            print()
        print(f'Cost: {compute_cost(solution)}')
        vis(obstacles, solution)
    else:
        print("Solution not found")
        return


if __name__ == '__main__':
    main()
