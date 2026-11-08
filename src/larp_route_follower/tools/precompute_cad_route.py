#!/usr/bin/env python3
"""Offline CAD route generator. Requires numpy/scipy only on the build host.

Occupied AND missing cells obstruct the swept sphere. Required tunnel/window
passages are explicit; the artifact is geometry, never permission to fly through
an obstacle detected at runtime. No dependency on this script on the aircraft.
"""
import argparse
import heapq
import itertools
import json
import math
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt


def generate(cad, output):
    res = float(Path(cad).read_text().splitlines()[0].split()[1])
    rows = np.loadtxt(cad, skiprows=1)
    ids = np.floor(rows[:, :3] / res + 1e-6).astype(int)
    origin = ids.min(axis=0) - 1
    shape = ids.max(axis=0) - origin + 2
    free = np.zeros(shape, dtype=bool)
    free[tuple((ids - origin).T)] = rows[:, 3] == 0
    # Distance to obstacle centres minus voxel circumradius conservatively
    # bounds distance to the complete occupied voxel (including diagonal moves).
    clearance = distance_transform_edt(free) * res - math.sqrt(3) * res / 2
    lower = np.array([-4.1624, -5.5626, 0.]) + .4
    upper = np.array([1.9336, .5334, 1.8]) - .4
    def valid(p):
        ix = np.floor(np.asarray(p)/res).astype(int)-origin
        return (np.all(p >= lower) and np.all(p <= upper) and
                np.all(ix >= 0) and np.all(ix < shape) and clearance[tuple(ix)] >= .34)
    def edge(a,b):
        return all(valid(a+(b-a)*t) for t in np.linspace(0,1,max(2,math.ceil(np.linalg.norm(b-a)/.02)+1)))
    sin, cos = math.sin(math.radians(20)), math.cos(math.radians(20))
    wx = (90.6+31.75*sin-120)*.0254
    wy = (.72+31.75*cos-120)*.0254
    centre = np.array([wy-1.1144, -(wx+2.5146), .6948])
    normal = np.array([sin,cos,0.])
    checkpoints = [np.array(p) for p in [(0,0,1),(.8,-4.85,1),(-1.8,-4.85,1)]]
    checkpoints += [centre-.5*normal,centre+.5*normal,np.array([-2.82128,-.77724,1.])]
    # 10 cm search lattice; collision geometry remains the complete 5 cm CAD.
    step=.1
    moves=list(itertools.product((-1,0,1),repeat=3));moves.remove((0,0,0))
    def search(start,goal):
        seed=tuple(np.round(start/step).astype(int))
        point=lambda i:np.array(i)*step
        if not edge(start,point(seed)) or not valid(goal):
            raise RuntimeError(f'Checkpoint lacks body clearance: {start} -> {goal}')
        dist={seed:0.};parent={};heap=[(float(np.linalg.norm(point(seed)-goal)),seed)]
        closed=set()
        while heap:
            _,cur=heapq.heappop(heap)
            if cur in closed:continue
            closed.add(cur);p=point(cur)
            if np.linalg.norm(p-goal)<.16 and edge(p,goal):
                path=[goal,p]
                while cur in parent:cur=parent[cur];path.append(point(cur))
                return [start]+list(reversed(path)),len(closed)
            for move in moves:
                nxt=tuple(a+b for a,b in zip(cur,move))
                if nxt in closed:continue
                q=point(nxt);cost=dist[cur]+float(np.linalg.norm(q-p))
                if cost>=dist.get(nxt,float('inf')) or not edge(p,q):continue
                dist[nxt]=cost;parent[nxt]=cur
                heapq.heappush(heap,(cost+float(np.linalg.norm(q-goal)),nxt))
        raise RuntimeError('No CAD route between required passages')
    route=[];gates=[];expansions=0
    for start,goal in zip(checkpoints,checkpoints[1:]):
        raw,count=search(start,goal);expansions+=count
        # Greedy line-of-sight simplification, never across a required passage.
        compact=[raw[0]];i=0
        while i<len(raw)-1:
            j=len(raw)-1
            while j>i+1 and not edge(raw[i],raw[j]):j-=1
            compact.append(raw[j]);i=j
        for p in compact:
            if not route or np.linalg.norm(p-route[-1])>1e-6:route.append(p)
        gates.append(len(route)-1)
    # Dense samples make ordered progress/rejoining independent of long edges.
    dense=[route[0]];dense_gates=[]
    for i,(a,b) in enumerate(zip(route,route[1:]),1):
        n=max(1,math.ceil(np.linalg.norm(b-a)/.1))
        dense.extend(a+(b-a)*k/n for k in range(1,n+1))
        if i in gates:dense_gates.append(len(dense)-1)
    with Path(output).open('w') as f:
        f.write('# CAD master route, map frame metres. Start 0 0 1; shifted final goal.\n')
        f.write('# gates '+ ' '.join(map(str,dense_gates))+'\n')
        for p in dense:f.write(' '.join(f'{x:.6f}' for x in p)+'\n')
    print(json.dumps({'points':len(dense),'length_m':sum(np.linalg.norm(b-a) for a,b in zip(dense,dense[1:])),
                      'expanded':expansions,'gates':dense_gates,'output':str(output)}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('cad');p.add_argument('output');a=p.parse_args()
    generate(a.cad,a.output)
