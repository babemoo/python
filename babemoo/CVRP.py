#CVRP
"""Capacitated Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from haversine import haversine
import time
start_time = time.time()



###########################
# Problem Data Definition #
###########################
class Vehicle():
    """Stores the property of a vehicle"""
    def __init__(self):
        """Initializes the vehicle properties"""
        self._capacity = 15

    @property
    def capacity(self):
        """Gets vehicle capacity"""
        return self._capacity


class DataProblem():
    """Stores the data for the problem"""
    def __init__(self):
        """Initializes the data for the problem"""
        self._vehicle = Vehicle()
        self._num_vehicles = 15



        # Locations in block unit
        locations = \
                [
                    (14.373754, 100.836407),  # depot
                    # # # ชลบุรี # # #
                    (13.385257, 100.989043),  # บจก.บางทรายค้าวัสดุ (1)
                    (13.357187, 100.996355),  # บจ.นลินรัตน์การเกษตร (2)
                    (13.116732, 100.916528),  # หจก.อ่าวอุดมค้าไม้ (3)
                    (12.836261, 100.912753),  # บ.บ้านอำเภอเทรดดิ้ง จก. (4)
                    (12.679031, 100.939723),  # หจก.รังศิริเคหะภัณฑ์ (5)
                    (12.965603, 100.910155),  # บ.ทองทิตต์เจริญ (1985) จก. (6)
                    (13.355352, 100.999196),  # บ.ชลบุรีเกียรติเจริญ จก. (7)
                    (13.163742, 100.931482),  # หจก.รัตนประดิษฐ์ค้าไม้ศรีราชา (8)
                    # # # จันทบุรี # # #
                    (12.460677, 102.229842),  # หจก.ดำน้ำหยด (9)
                    (12.61544, 102.123146),  # บจ.เจริญเคหะ โฮมมาร์ท (10)
                    (12.72115, 101.972848),  # สหชัยท่อน้ำ (11)
                    (13.091482, 102.253094),  # หจก.บุญทวีวัสดุ (12)
                    # # # ระยอง # # #
                    (12.720792, 101.066595),  # หจก.แสงกระจ่าง(สำนักงาน เทศบาล เมืองบ้านฉาง) (13)
                    (12.726166, 101.058159),  # บ.บ้านฉางสามเจริญ จก. (14)
                    (12.681149, 101.282734),  # บ.ระยองเคหะภัณฑ์ 1989 จก. (15)
                    # # # ฉะเชิงเทรา # # #
                    (13.74471, 101.357831),  # หจก.พนมซิเมนต์ (16)
                    (13.720463, 101.206644),  # บ.พูนศิริวัฒนา จก. (17)
                    (13.536871, 100.964694),  # บจก.สุทธิพงศ์ค้าวัสดุก่อสร้าง (072) (18)
                    (13.694448, 101.063641),  # บ.ชัยเจริญแปดริ้วค้าวัสดุ จก. (19)
                    (13.727118, 101.208758),  # หจก.เพิ่มทรัพย์ค้าวัสดุก่อสร้าง (20)
                    (13.84993, 101.052112),  # หจก. ก.ทวีทรัพย์ค้าวัสดุ (21)
                    (13.743739, 101.34692),  # บ.เจริญโสภณ จก. (22)
                    (13.68269, 101.06574),  # บ.ทองทวีซีเมนต์ จก. (23)
                    # # # ปราจีนบุรี # # #
                    (13.983836, 101.774994),  # หจก.ส.วัสดุก่อสร้าง (24)

                    # # # สระแก้ว # # #
                    (13.826301, 102.062271)  # บ.เอกภัณฑ์ซีเมนต์ จก. (25)
                ]
        # locations in meters using the city block dimension

        self._locations = [(
            loc[0] ,
            loc[1] ) for loc in locations]

        self._depot = 0

        self._demands = \
            [0, # depot

             3, 5, 1, 1, 2,
             2, 3, 3, 1, 2,
             4, 2, 1, 1, 3,
             2, 2, 3, 1, 1,
             3, 1, 1, 2, 2

             ]

    @property
    def vehicle(self):
        """Gets a vehicle"""
        return self._vehicle

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        """Gets locations"""
        return self._locations

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.locations)

    @property
    def depot(self):
        """Gets depot location index"""
        return self._depot

    @property
    def demands(self):
        """Gets demands at each location"""
        return self._demands

#######################
# Problem Constraints #
#######################
def manhattan_distance(position_1, position_2):
    """Computes the Manhattan distance between two points"""



    return haversine(position_1,position_2)

class CreateDistanceEvaluator(object): # pylint: disable=too-few-public-methods
    """Creates callback to return distance between points."""
    def __init__(self, data):
        """Initializes the distance matrix."""
        self._distances = {}

        # precompute distance between location to have distance callback in O(1)
        for from_node in xrange(data.num_locations):
            self._distances[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._distances[from_node][to_node] = 0
                else:
                    self._distances[from_node][to_node] = (
                        manhattan_distance(
                            data.locations[from_node],
                            data.locations[to_node]))

    def distance_evaluator(self, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return self._distances[from_node][to_node]

class CreateDemandEvaluator(object): # pylint: disable=too-few-public-methods
    """Creates callback to get demands at each location."""
    def __init__(self, data):
        """Initializes the demand array."""
        self._demands = data.demands

    def demand_evaluator(self, from_node, to_node):
        """Returns the demand of the current node"""
        del to_node
        return self._demands[from_node]

def add_capacity_constraints(routing, data, demand_evaluator):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimension(
        demand_evaluator,
        0, # null capacity slack
        data.vehicle.capacity, # vehicle maximum capacity
        True, # start cumul to zero
        capacity)

###########
# Printer #
###########
class ConsolePrinter():
    """Print solution to console"""
    def __init__(self, data, routing, assignment):
        """Initializes the printer"""
        self._data = data
        self._routing = routing
        self._assignment = assignment

    @property
    def data(self):
        """Gets problem data"""
        return self._data

    @property
    def routing(self):
        """Gets routing model"""
        return self._routing

    @property
    def assignment(self):
        """Gets routing model"""
        return self._assignment

    def print(self):
        """Prints assignment on console"""
        # Inspect solution.
        total_dist = 0
        for vehicle_id in xrange(self.data.num_vehicles):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index)))
                route_dist += manhattan_distance(
                    self.data.locations[node_index],
                    self.data.locations[next_node_index])
                route_load += self.data.demands[node_index]
                plan_output += ' {0}  -> '.format(node_index, route_load)
                index = self.assignment.Value(self.routing.NextVar(index))

            node_index = self.routing.IndexToNode(index)
            total_dist += route_dist
            plan_output += ' {0}\n'.format(node_index, route_load)
            plan_output += 'Distance of the route: {0}km\n'.format(route_dist)
            plan_output += 'Load of the route: {0}\n'.format(route_load)
            print(plan_output)
        print('Total Distance of all routes: {0}km'.format(total_dist))

########
# Main #
########
def main():
    """Entry point of the program"""
    # Instantiate the data problem.
    data = DataProblem()

    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    # Define weight of each edge
    distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    # Add Capacity constraint
    demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
    add_capacity_constraints(routing, data, demand_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    printer = ConsolePrinter(data, routing, assignment)
    printer.print()

if __name__ == '__main__':
  main()
  print("time is %s seconds" % (time.time() - start_time))