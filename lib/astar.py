#
#
#   A-Star
#   This module contains functions and objects reused throughout the simulation
#
#
import sys
from shapely.geometry import Polygon, LineString

polygons = []

#
#   Action Function
#


def reachable_vertices_by_state(current_state, goal_state, closed_set):
    reachable_vertices = []
    # print("Check reachable nodes from the current node: " + str(current_state))
    for polygon in polygons:
        for vertex in polygon.exterior.coords:

            # if we have it, move on
            if vertex in reachable_vertices:
                continue

            # ignore closed nodes
            if vertex in closed_set:
                continue

            # let's not test ourselves
            if current_state == vertex:
                test_line_valid = False
                # print("Don't need to test the current state...")
                continue
            else:
                test_line_valid = True

            test_line = LineString((current_state, vertex))

            # print("Testing if " + str(vertex) + " is reachable...")
            obstacle_names = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
            i = 0
            for obstacle in polygons:
                if test_line.crosses(obstacle):
                    # print("Obstacle '" + obstacle_names[i] + "' is in the way of point " + str(vertex))
                    test_line_valid = False
                    break
                if test_line.within(obstacle):
                    # print("Skip non-adjacent vertices on the same obstacle '" + obstacle_names[i] + "' :" + str(vertex))
                    test_line_valid = False
                    break
                i += 1

            if test_line_valid:
                # print(str(test_line) + " is reachable!")
                reachable_vertices.append(vertex)

    # Test for goal state
    line_to_goal = LineString((current_state, goal_state))

    line_to_goal_blocked = False
    for obstacle in polygons:
        if line_to_goal.crosses(obstacle) or line_to_goal.within(obstacle):
            line_to_goal_blocked = True
            break

    if not line_to_goal_blocked:
        reachable_vertices.append(goal_state)

    return reachable_vertices


def construct_best_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)

    total_path.reverse()
    return total_path


def heuristic_function(state, goal):
    return line_length(state, goal)


def line_length(node1, node2):
    line = LineString((node1, node2))
    return line.length


def run(start, goal):
    closed_set = []
    open_set = [start]
    came_from = dict()

    g_score = {start: 0}
    f_score = {start: heuristic_function(start, goal)}

    current_state = None

    while len(open_set) > 0:

        previous_state = current_state

        f_score_min = float('inf')  # infinity
        for open_state in open_set:
            if f_score[open_state] < f_score_min:
                current_state = open_state
                f_score_min = f_score[open_state]

        # if current_state is not previous_state:
        #     print("Current State updated: " + str(current_state))

        if current_state == goal:
            return construct_best_path(came_from, goal)

        open_set.remove(current_state)
        closed_set.append(current_state)
        # print(str(current_state) + " closed!")

        reachable_states = reachable_vertices_by_state(current_state, goal, closed_set)
        # print(reachable_states)

        # print("Current Path: " + str(construct_best_path(came_from, current_state)))
        # print("State\t\t\tg_score\t\tdistance_to_state\t\theuristic\t\ttotal g(n)\t\ttotal f(n)")
        for state in reachable_states:

            # add cost of next potential state to current cost
            temp_g_score = g_score[current_state] + line_length(current_state, state)
            heuristic = heuristic_function(state, goal)

            # print(str(state).ljust(12) + "\t" + "{0:.3f}".format(g_score[current_state]) + "\t\t" +
            #       "{0:.3f}".format(line_length(current_state, state)) + "\t\t\t\t\t" + "{0:.3f}".format(heuristic) +
            #       "\t\t\t" + "{0:.3f}".format(temp_g_score) + "\t\t\t" + "{0:.3f}".format(temp_g_score + heuristic))

            if state not in open_set:
                open_set.append(state)
            elif temp_g_score >= g_score[state]:
                continue

            came_from[state] = current_state
            g_score[state] = temp_g_score
            f_score[state] = g_score[state] + heuristic

    return False
