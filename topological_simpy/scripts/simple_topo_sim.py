#!/usr/bin/env python

from topological_simpy.robot import Robot, TopoMap
import simpy
import itertools
import csv
import os

from random import randint, choice, seed
from datetime import datetime

seed(10)

env = simpy.Environment()
tmap = TopoMap('/home/zuyuan/catkin_ws/src/topological_navigation/topological_navigation/maps/riseholme.tmap2', env)

# r1 = Robot('thorvald', tmap, 'A')
# r1.goto('A')
# r2 = Robot('thorvald2', tmap, 'D')
# r2.goto('A')

#robot_homes = ['dock_0', 'dock_1', 'dock_2']
robot_homes = ['WayPoint131', 'WayPoint111', 'WayPoint66', 'WayPoint94']
target_nodes = ['WayPoint66', 'WayPoint142', 'WayPoint102', 'WayPoint78']
robots = [
    # Robot('Hurga', tmap, 'A'),
    # Robot('Foo', tmap, 'B'),
    Robot('Hurga', tmap, robot_homes[0]),
    Robot('Foo', tmap, robot_homes[1]),
    Robot('Foo2', tmap, robot_homes[2]),
    Robot('Thor', tmap, robot_homes[3])
]
nodes = tmap.get_nodes()
# print(nodes)


def goal_generator(_env, _robots, _nodes, max_interval=50):
    for i in itertools.count():
        delay_time = randint(_env.now + 1, _env.now + max_interval)  # TODO decrease delay_time
        #delay_time = randint(10, max_interval)  # TODO decrease delay_time
        print('.% 4d:    Generating goal after %6d seconds at %6d s' % (env.now, delay_time, env.now + delay_time))
        yield _env.timeout(delay_time)
        rob = choice(_robots)
        n = choice(_nodes)
        print('+% 4d:    new goal for robot %10s: %s' % (_env.now, rob._name, n))
        rob.goto(n)


def goal_generator2(_env, _robots, _target_nodes):
    for i in range(len(_robots)):
        yield _env.timeout(5)
        rob = _robots[i]
        print('+% 4d:    new goal for robot %10s: %s' % (_env.now, rob._name, _target_nodes[i]))
        rob.goto(_target_nodes[i])


#env.process(goal_generator(env, robots, nodes))
env.process(goal_generator2(env, robots, target_nodes))

until = 3000
while env.peek() < until:
    tmap.monitor()
    # for r in robots:
    #     print("R% 3d:     STATUS: % 10s: %s (%s)" % (
    #         env.now,
    #         r._name,
    #         r._current_node,
    #         'ACTIVE' if r._active_process and r._active_process.is_alive else 'IDLE'
    #     ))
    env.step()
# print(tmap._node_log)

# write the node_log to file
now = datetime.now()

file_name = 'node_log_' + now.isoformat() + '.csv'
try:
    os.mkdir("./data")
except OSError as e:
    print("Directory exists")

with open('../data/' + file_name, 'w') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in tmap.node_log.items():
        writer.writerow([key, value])
    writer.writerow('')
    for r in robots:
        for key, value in r.cost.items():
            writer.writerow([key, value])
        writer.writerow('')
# env.run(until=3600)
