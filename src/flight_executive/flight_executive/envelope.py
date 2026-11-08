"""Map-frame body containment independent of planner trajectory generation."""
import math


class Envelope:
    def __init__(self, lower, upper, heading=0., origin=(0., 0., 0.), margin=.4):
        if (len(lower) != 3 or len(upper) != 3 or len(origin) != 3 or
                not all(math.isfinite(v) for v in (*lower, *upper, *origin, heading, margin)) or
                margin < 0 or any(b-a <= 2*margin for a,b in zip(lower,upper))):
            raise ValueError('Invalid map boundary/alignment')
        self.lower, self.upper, self.origin = lower, upper, origin
        self.heading, self.margin = heading, margin

    def contains(self, position, ground_contact=False, ground_tolerance=0.0):
        dx,dy,dz = (v-o for v,o in zip(position,self.origin))
        c,s = math.cos(self.heading), math.sin(self.heading)
        p = (c*dx+s*dy, s*dx-c*dy, -dz)
        return all(math.isfinite(v) and
                   (self.lower[i]-ground_tolerance if ground_contact and i==2 else self.lower[i]+self.margin)-1e-6 <= v <=
                   self.upper[i]-self.margin+1e-6 for i,v in enumerate(p))
