#!/usr/bin/env python

from topological_simpy.robot import Robot, TopoMap
import simpy
import itertools
import csv

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
robot_homes = ['WayPoint131', 'WayPoint111', 'WayPoint66']
target_nodes = ['WayPoint66', 'WayPoint142', 'WayPoint102']
robots = [
    # Robot('Hurga', tmap, 'A'),
    # Robot('Foo', tmap, 'B'),
    Robot('Hurga', tmap, robot_homes[0]),
    Robot('Foo', tmap, robot_homes[1]),
    Robot('Foo2', tmap, robot_homes[2])
]

nodes = tmap.get_nodes()

print(nodes)


def goal_generator(env, robots, nodes, max_interval=200):
    for i in itertools.count():
        #delay_time = randint(env.now + 1, env.now + max_interval)  # TODO decrease delay_time
        delay_time = randint(100, max_interval)  # TODO decrease delay_time
        print('.% 4d:    Generating goal after %6d seconds at %6d s' % (env.now, delay_time, env.now + delay_time))
        yield env.timeout(delay_time)
        r = choice(robots)
        n = choice(nodes)
        print('+% 4d:    new goal for robot %10s: %s' % (env.now, r._name, n))
        r.goto(n)


def goal_generator2(env, robots, target_nodes):
    for i in range(3):
        yield env.timeout(5)
        r = robots[i]
        print('+% 4d:    new goal for robot %10s: %s' % (env.now, r._name, target_nodes[i]))
        r.goto(target_nodes[i])



#env.process(goal_generator(env, robots, nodes))
env.process(goal_generator2(env, robots, target_nodes))

until = 3600
while env.peek() < until:
    tmap.monitor()
    for r in robots:
        print("R% 3d:     STATUS: % 10s: %s (%s)" % (
            env.now,
            r._name,
            r._current_node,
            'ACTIVE' if r._active_process and r._active_process.is_alive else 'IDLE'
        ))
    env.step()
print(tmap._node_log)

# write the node_log to file
now = datetime.now()
node_log_file_name = 'node_log_' + now.isoformat() + '.csv'
with open(node_log_file_name, 'w') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in tmap._node_log.items():
        writer.writerow([key, value])
    writer.writerow('')
    for r in robots:
        for key, value in r._cost.items():
            writer.writerow([key, value])
        writer.writerow('')
# env.run(until=3600)
