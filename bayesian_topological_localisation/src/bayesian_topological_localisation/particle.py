import numpy as np

class Particle():

    def __init__(self, node, vel, life, timestamp_secs, n_steps_vel=40):
        self.node = node
        self.vel = np.array(vel)
        self.life = life
        self.last_time_secs = timestamp_secs
        self.n_steps_vel = n_steps_vel

    def __copy__(self):
        newone = Particle(self.node, np.copy(self.vel),
                          self.life, self.last_time_secs, self.n_steps_vel)
        # newone.__dict__.update(self.__dict__)
        return newone
    
    def __str__(self):
        return "({}, {}, {})".format(self.node, self.vel, self.life)