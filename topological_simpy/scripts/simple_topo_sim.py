#!/usr/bin/env python

from topological_simpy.robot import Robot, TopoMap
import simpy
import itertools
import csv, codecs, cStringIO

from random import randint, choice, seed
from datetime import datetime

seed(10)

env = simpy.Environment()
# tmap = TopoMap('/home/mhanheide/workspace/topological_navigation_ws/src/topological_navigation/topological_navigation/maps/test.tmap2', env)
tmap = TopoMap('/home/zuyuan/catkin_ws/src/topological_navigation/topological_navigation/maps/riseholme.tmap2', env)

# r1 = Robot('thorvald', tmap, 'A')
# r1.goto('A')
# r2 = Robot('thorvald2', tmap, 'D')
# r2.goto('A')

robots = [
    # Robot('Hurga', tmap, 'A'),
    # Robot('Foo', tmap, 'B'),
    Robot('Hurga', tmap, 'dock_0'),
    Robot('Foo', tmap, 'dock_1'),
    Robot('Foo2', tmap, 'dock_2')
]

nodes = tmap.get_nodes()

print(nodes)


def goal_generator(env, robots, nodes, max_interval=50):
    for i in itertools.count():
        delay_time = randint(env.now + 1, env.now + max_interval)  # TODO decrease delay_time
        print('.% 4d:    Generating goal after %6d seconds at %6d s' % (env.now, delay_time, env.now + delay_time))
        yield env.timeout(delay_time)
        r = choice(robots)
        n = choice(nodes)
        print('+% 4d:    new goal for robot %10s: %s' % (env.now, r._name, n))
        r.goto(n)

def goal_generator2(env, robots, nodes, max_interval=10):
    for i in range(3):
        #delay_time = randint(env.now + 1, env.now + max_interval)  # TODO decrease delay_time
       # print('.% 4d:    Generating goal after %6d seconds at %6d s' % (env.now, delay_time, env.now + delay_time))
        yield env.timeout(5)
        r = robots[i]
        n = nodes[3+i]
        print('+% 4d:    new goal for robot %10s: %s' % (env.now, r._name, n))
        r.goto(n)



#env.process(goal_generator(env, robots, nodes))
env.process(goal_generator2(env, robots, nodes))

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
# env.run(until=3600)
