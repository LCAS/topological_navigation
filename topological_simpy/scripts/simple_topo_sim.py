#!/usr/bin/env python

from topological_simpy.robot import Robot, TopoMap
import simpy
import itertools


from random import randint, choice

env = simpy.Environment()
#tmap = TopoMap('/home/mhanheide/workspace/topological_navigation_ws/src/topological_navigation/topological_navigation/maps/test.tmap2', env)
tmap = TopoMap('/home/zuyuan/catkin_ws/src/topological_navigation/topological_navigation/maps/riseholme.tmap2', env)

#r1 = Robot('thorvald', tmap, 'A')
#r1.goto('A')
#r2 = Robot('thorvald2', tmap, 'D')
#r2.goto('A')

robots = [
    #Robot('Hurga', tmap, 'A'),
    #Robot('Foo', tmap, 'B'),
    Robot('Hurga', tmap, 'dock_0'),
    Robot('Foo', tmap, 'dock_1'),
    Robot('Foo2', tmap, 'dock_2')
]

nodes = tmap.get_nodes()

print(nodes)

def goal_generator(env, robots, nodes, max_interval=50):
    for i in itertools.count():
        yield env.timeout(randint(env.now+1, env.now+max_interval))
        r = choice(robots)
        n = choice(nodes)
        print('new goal for robot %s: %s' % (r._name, n))
        r.goto(n)

env.process(goal_generator(env, robots, nodes))

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
#env.run(until=3600)
