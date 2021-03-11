#!/usr/bin/env python

import itertools
import random
import simpy
from yaml import safe_load
from topological_navigation.route_search2 import TopologicalRouteSearch2
from topological_navigation.tmap_utils import *
from math import hypot, ceil
from functools import partial, wraps

RANDOM_SEED = 42
GAS_STATION_SIZE = 200     # liters
THRESHOLD = 20             # Threshold for calling the tank truck (in %)
FUEL_TANK_SIZE = 40        # liters
FUEL_TANK_LEVEL = [5, 25]  # Min/max levels of fuel tanks (in liters)
REFUELING_SPEED = 2        # liters / second
TANK_TRUCK_TIME = 30       # Seconds it takes the tank truck to arrive
T_INTER = [1, 30]          # Create a car every [min, max] seconds
SIM_TIME = 100             # Simulation time in seconds


def patch_resource(resource, pre=None, post=None):
    """Patch *resource* so that it calls the callable *pre* before each
     put/get/request/release operation and the callable *post* after each
     operation.  The only argument to these functions is the resource
     instance.

    """

    def get_wrapper(func):
        # Generate a wrapper for put/get/request/release
        @wraps(func)
        def wrapper(*args, **kwargs):
            # This is the actual wrapper
            # Call "pre" callback
            if pre:
                pre(resource)

            # Perform actual operation
            ret = func(*args, **kwargs)

            # Call "post" callback
            if post:
                post(resource)

            return ret

        return wrapper

    # Replace the original operations with our wrapper
    for name in ['put', 'get', 'request', 'release']:
        if hasattr(resource, name):
            setattr(resource, name, get_wrapper(getattr(resource, name)))


class TopoMap():
    def __init__(self, topo_map_file, env=None):
        with open(topo_map_file, "r") as f:
            self.tmap2 = safe_load(f)
        self._route_planner = TopologicalRouteSearch2(self.tmap2)
        self.env = env

        self._nodes = sorted([n['node']['name'] for n in self.tmap2['nodes']])
        self._nodes_dict = {n['node']['name']: n['node'] for n in self.tmap2['nodes']}
        self._node_res = {}

        self._node_log = {}

        if self.env:
            for n in self._nodes:
                self._node_res[n] = simpy.Container(self.env, capacity=1, init=0)  # each node can hold one robot max
                self._node_log[n] = []
                patch_resource(self._node_res[n], post=partial(self.log_resource, n))

    def log_resource(self, node, resource):
        """This is our monitoring callback."""
        item = (
            resource._env.now,  # The current simulation time
            resource.level
        )
        self._node_log[node].append(item)

    def get_nodes(self):
        return self._nodes

    def distance(self, nodeA, nodeB):
        a = self._nodes_dict[nodeA]
        b = self._nodes_dict[nodeB]
        return hypot(
            a['pose']['position']['x'] - b['pose']['position']['x'],
            a['pose']['position']['y'] - b['pose']['position']['y']
        )

    def request_node(self, node):
        """
        Put one robot into the _node_res, i.e., the node resource is occupied by the robot. Note: the robot starts requesting the next route node when the robot reaches the current node
        """
        if self.env:
            return self._node_res[node].put(1)

    def release_node(self, node):
        """
        Get the robot out of the _node_res, i.e., the node resource is released, the robot leaves this node
        """
        if self.env:
            if self._node_res[node].level > 0:
                return self._node_res[node].get(1)
            else:
                return self.env.timeout(0)

    def get_route(self, origin, target):
        return self._route_planner.search_route(origin, target)

    def node_occupied(self, node):
        return self._node_res[n].count > 0

    def monitor(self, freq=2):
        """
        Get the topological map nodes that occupied by the robot currently
        """
        occupied_nodes = []
        for n in self._node_res:
            if self._node_res[n].level > 0:
                occupied_nodes.append(n)
        print("*% 4d:   nodes %s are occupied" % (self.env.now, sorted(occupied_nodes)))


class Robot():
    def __init__(self, name, tmap, initial_node='A'):
        self._current_node = initial_node
        self._tmap = tmap
        self._env = tmap.env
        self._name = name
        self._speed_m_s = .3
        self._active_process = None
        if self._tmap.env:
            self._tmap.request_node(initial_node)
            # self._tmap._node_res[initial_node].count = 1
            # print(self._current_node)
            # self._tmap.env.process(self._goto(initial_node))

    def _goto(self, target):
        route = self._tmap.get_route(self._current_node, target)
        if self._tmap.env:
            if route is not None:
                r = route.source[1:]
            else:
                r = []
            r.append(target)
            if target == self._current_node:
                print('% 5d: %s is already at target %s' % (
                    self._tmap.env.now, self._name,
                    target
                ))
                return

            print('% 5d: %s going from node %s to node %s via route %s' % (
                self._tmap.env.now, self._name,
                self._current_node, target, r))
            interrupted = False
            for n in r:
                #  getting the distance between current and target:
                d = self._tmap.distance(self._current_node, n)
                time_to_travel = int(ceil(d / self._speed_m_s))
                print('% 5d:  %s traversing route from node %s to node %s (distance: %f, time: %d)' % (
                    self._tmap.env.now, self._name, self._current_node, n, d, time_to_travel))
                request = None
                try:
                    start_wait = self._env.now
                    yield self._tmap.request_node(n)
                    if self._env.now - start_wait > 0:
                        print('$$$$ % 5d:  %s has lock on %s after %d' % (
                        self._env.now, self._name, n, self._env.now - start_wait))
                except:
                    print('% 5d: %s INTERRUPTED while waiting to gain access to go from node %s going to node %s' % (
                        self._tmap.env.now, self._name,
                        self._current_node, n
                    ))
                    interrupted = True
                    break
                try:
                    time_to_travel = ceil(d / (2 * self._speed_m_s))
                    yield self._tmap.env.timeout(time_to_travel)
                    yield self._tmap.release_node(self._current_node)
                    # The robot is reaching at the half way between the current node and next node
                    print('% 5d:  %s ---> %s reaching half way ---> %s' % (self._tmap.env.now, self._current_node, self._name, n))
                    self._current_node = n
                    remain_time_to_travel = ceil(d / self._speed_m_s) - time_to_travel
                    yield self._tmap.env.timeout(remain_time_to_travel)
                    print('% 5d:  %s reached node %s' % (self._tmap.env.now, self._name, n))
                    yield self._tmap.env.timeout(0)  # Why yield here? TODO
                except simpy.Interrupt:
                    print('% 5d: %s INTERRUPTED while travelling from node %s going to node %s' % (
                        self._tmap.env.now, self._name,
                        self._current_node, n
                    ))
                    print('% 5d: @@@ %s release previously acquired target node %s' % (self._env.now, self._name, n))
                    yield self._tmap.release_node(n)
                    interrupted = True
                    break
                # if self._current_node != target:
            if interrupted:
                print('% 5d: %s ABORTED at %s' % (self._tmap.env.now, self._name, self._current_node))
            else:
                print('% 5d: %s COMPLETED at %s' % (self._tmap.env.now, self._name, self._current_node))

    def goto(self, target):
        if self._tmap.env:
            if self._active_process:
                if self._active_process.is_alive:
                    self._active_process.interrupt()
                    self._active_process = None
            self._active_process = self._tmap.env.process(self._goto(target))
        else:
            self._current_node = target


def car(name, env, gas_station, fuel_pump):
    """A car arrives at the gas station for refueling.

    It requests one of the gas station's fuel pumps and tries to get the
    desired amount of gas from it. If the stations reservoir is
    depleted, the car has to wait for the tank truck to arrive.

    """
    fuel_tank_level = random.randint(*FUEL_TANK_LEVEL)
    print('%s queuing at gas station at %.1f' % (name, env.now))
    with gas_station.request() as req:
        start = env.now
        # Request one of the gas pumps
        yield req
        print('%s serving at gas station at %.1f' % (name, env.now))

        # Get the required amount of fuel
        liters_required = FUEL_TANK_SIZE - fuel_tank_level
        yield fuel_pump.get(liters_required)

        # The "actual" refueling process takes some time
        yield env.timeout(liters_required / REFUELING_SPEED)

        print('%s finished refueling in %.1f seconds.' % (name,
                                                          env.now - start))


def gas_station_control(env, fuel_pump):
    """Periodically check the level of the *fuel_pump* and call the tank
    truck if the level falls below a threshold."""
    while True:
        print('  fuel level at %f' % fuel_pump.level)
        if fuel_pump.level / fuel_pump.capacity * 100 < THRESHOLD:
            # We need to call the tank truck now!
            print('Calling tank truck at %d' % env.now)
            # Wait for the tank truck to arrive and refuel the station
            yield env.process(tank_truck(env, fuel_pump))

        yield env.timeout(10)  # Check every 10 seconds


def tank_truck(env, fuel_pump):
    """Arrives at the gas station after a certain delay and refuels it."""
    yield env.timeout(TANK_TRUCK_TIME)
    print('Tank truck arriving at time %d' % env.now)
    amount = fuel_pump.capacity - fuel_pump.level
    print('Tank truck refuelling %.1f liters.' % amount)
    yield fuel_pump.put(amount)


def car_generator(env, gas_station, fuel_pump):
    """Generate new cars that arrive at the gas station."""
    for i in itertools.count():
        yield env.timeout(random.randint(*T_INTER))
        env.process(car('Car %d' % i, env, gas_station, fuel_pump))


if __name__ == '__main__':
    # Setup and start the simulation
    print('Gas Station refuelling')
    random.seed(RANDOM_SEED)

    # Create environment and start processes
    env = simpy.Environment()
    gas_station = simpy.Resource(env, 2)
    fuel_pump = simpy.Container(env, GAS_STATION_SIZE, init=GAS_STATION_SIZE)
    env.process(gas_station_control(env, fuel_pump))
    env.process(car_generator(env, gas_station, fuel_pump))

    # Execute!
    env.run(until=SIM_TIME)
