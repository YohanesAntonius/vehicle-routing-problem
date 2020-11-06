
"""Vehicles Routing Problem (VRP)."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt
import numpy as np
import math

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['locations'] = [
        (22, 3.05), (49.88, 50.56), (3.22, 49.62), (36.95, 5.26), (6.65, 51.99), 
        (6.16, 11.98), (29.69, 38.47), (8.1, 22.27), (21.78, 7.63), (50.66, 1.76), 
        (4.07, 27.76), (46.49, 30.79), (52.56, 2.69), (2.72, 20.35), (10.42, 21.75), 
        (53.87, 12.16), (12.53, 40.09), (53.08, 37.08), (55.35, 29.15), (32.58, 44.77),
        (36.98, 27.46), (51.36, 18.56), (39.47, 34.63), (0.14, 47.6), (55.45, 33.63), (47.57, 14.25)
    ]
    data['num_vehicles'] = 5
    data['depot'] = 0
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1]))))
    return distances

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    max_route_distance = 0
    solutions = {}
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        vehicle_solution = []
        while not routing.IsEnd(index):
            vehicle_solution.append(manager.IndexToNode(index))
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
        solutions[vehicle_id] = vehicle_solution
    print('Maximum of the route distances: {}m'.format(max_route_distance))
    return solutions

"""Solve the CVRP problem."""
# Instantiate the data problem.
data = create_data_model()

# Create the routing index manager.
manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                       data['num_vehicles'], data['depot'])

# Create Routing Model.
routing = pywrapcp.RoutingModel(manager)

distance_matrix = compute_euclidean_distance_matrix(data['locations'])


# Create and register a transit callback.
def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add Distance constraint.
dimension_name = 'Distance'
routing.AddDimension(
    transit_callback_index,
    0,  # no slack
    3000,  # vehicle maximum travel distance
    True,  # start cumul to zero
    dimension_name)
distance_dimension = routing.GetDimensionOrDie(dimension_name)
distance_dimension.SetGlobalSpanCostCoefficient(100)

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

# Solve the problem.
solution = routing.SolveWithParameters(search_parameters)

# Print solution on console.
if solution:
    solution_dict = print_solution(data, manager, routing, solution)

X = np.array([x[0] for x in data['locations']])
Y = np.array([x[1] for x in data['locations']])

f, bx = plt.subplots(figsize = [7,5])

bx.plot(X, Y, 'ko', markersize=3)
bx.plot(X[0], Y[0], 'yo', markersize=20)

for i, txt in enumerate(data['locations']):
    bx.text(X[i], Y[i], f"{i}")

vehicle_colors = ["b","g","o", "m", "c"]
for vehicle in solution_dict:
    bx.plot(X[solution_dict[vehicle] + [0]], Y[solution_dict[vehicle] + [0]], f'{vehicle_colors[vehicle]}--')
    
bx.set_title("Tugas 5 Sistem Optimasi")
# plt.axis([-6, 60, -5, 60])
plt.show()