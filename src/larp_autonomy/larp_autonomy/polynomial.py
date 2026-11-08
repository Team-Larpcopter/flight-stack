"""Bounded, fail-closed decoder for SUPER's stopping polynomial contract.

Collision certification belongs to the mapper/planner. This module checks the
wire representation and samples its exact polynomial, never a sparse path.
"""
from dataclasses import dataclass
import math


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass(frozen=True)
class Piece:
    duration: float
    rows: tuple

    @classmethod
    def decode(cls, message, dimensions):
        degree=int(message.degree);duration=float(message.duration)
        if degree<0 or not math.isfinite(duration) or duration<=0:
            raise ValueError('invalid piece degree or duration')
        coefficients=tuple(float(x) for x in message.coefficients)
        if len(coefficients)!=dimensions*(degree+1):
            raise ValueError('invalid coefficient matrix')
        return cls(duration,tuple(coefficients[r*(degree+1):(r+1)*(degree+1)] for r in range(dimensions)))

    def sample(self, t, derivative=0):
        if not math.isfinite(t) or not 0<=t<=self.duration+1e-8:raise ValueError('sample outside piece')
        result=[]
        for row in self.rows:
            row=list(row)
            for _ in range(derivative):row=[v*(len(row)-i-1) for i,v in enumerate(row[:-1])]
            value=0.
            for coefficient in row:value=value*t+coefficient
            result.append(value)
        return tuple(result)


def chain(pieces, t, derivative=0):
    for piece in pieces:
        if t<=piece.duration+1e-9:return piece.sample(min(t,piece.duration),derivative)
        t-=piece.duration
    raise ValueError('sample after trajectory end')


def decode_chain(messages, dimensions):
    if not messages:raise ValueError('invalid piece count')
    pieces=tuple(Piece.decode(m,dimensions) for m in messages)
    return pieces


@dataclass(frozen=True)
class Trajectory:
    start: float
    certificate: float
    goal_id: int
    sequence: int
    position: tuple
    yaw: tuple

    @classmethod
    def decode(cls,message,now,goal_id,epoch=1,max_age=.3):
        certificate=seconds(message.header.stamp);start=seconds(message.start_time)
        if (message.status!=message.VALID or message.header.frame_id!='map' or message.localization_epoch!=epoch
            or message.goal_id!=goal_id or not math.isfinite(now) or (max_age is not None and not -.005<=now-certificate<=max_age)
            or start<=0 or (max_age is not None and start-now>.3)):raise ValueError('invalid or stale trajectory certificate')
        position=decode_chain(message.position,3);yaw=decode_chain(message.yaw,1)
        duration=sum(p.duration for p in position)
        if abs(duration-sum(p.duration for p in yaw))>.001:raise ValueError('yaw/position duration mismatch')
        # A freshly recertified terminal stop may be held after its end time.
        # Certificate freshness still expires even when the polynomial stops.
        return cls(start,certificate,int(goal_id),int(message.sequence),position,yaw)

    @property
    def duration(self):return sum(p.duration for p in self.position)

    def sample(self, now):
        # A stopped terminal state remains defined; extrapolation does not.
        t=max(0.,min(now-self.start,self.duration))
        if now >= self.start+self.duration:
            return (chain(self.position,t),(0.,0.,0.),(0.,0.,0.),chain(self.yaw,t)[0],0.)
        return (chain(self.position,t),chain(self.position,t,1),chain(self.position,t,2),
                chain(self.yaw,t)[0],chain(self.yaw,t,1)[0])
