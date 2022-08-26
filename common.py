from typing import Tuple
import numpy as np
import heapq as hq
import networkx as nx
import json
from scipy.optimize import linear_sum_assignment
from collections import OrderedDict
from typing import Any, Dict, List, Tuple


def manhattan_distance(v1:Tuple[int,int], v2:Tuple[int,int]):
    """_summary_

    Args:
        v1 (Tuple[int,int]): _description_
        v2 (Tuple[int,int]): _description_

    Returns:
        _type_: _description_
    """
    return abs(v1[0]-v2[0])+abs(v1[1]-v2[1])

class Agent(object):
    def __init__(self,loc=None,goal=None,id=None):
        self.loc=loc
        self.future_loc=loc
        self.priority=None
        self.path=[loc]
        self.plan=[]
        self.goal=goal
        self.id=id
        self.tmp_goal=None
        self.state=None
    
    def get_next_v(self):
        if len(self.plan)==0:
            return self.loc
        else:
            return self.plan[0]

def fill_paths(paths):
    pass

def compress_paths(paths):
    for p in paths:
        while len(p)>=2 and p[-1]==p[-2]:
            p.pop()


def evaluate_paths(paths,num_retrieval=None):
    makespan=0
    makespanLB=0
    sod=0
    if num_retrieval is None:
        num_retrieval=len(paths)
    sor=0
    compress_paths(paths)
    i=0
    for p in paths:
        makespan=max(makespan,len(p))
        makespanLB=max(makespanLB,manhattan_distance(p[0],p[-1]))
        sodi=0
        last_v=p[0]
        for u in p:
            if last_v!=u:
                sodi=sodi+1
            last_v=u
        sod=sod+sodi
        if i<num_retrieval:
            sor=sor+len(p)
        i=i+1
    return makespan,makespanLB,sod,sor


def save_data_as_csv(file_name,data1,data2):
    f = open(file_name, "w")
    for x, y in zip(data1,data2):
        f.write(str(x)+","+str(y))
        f.write("\n")
    
        
    



