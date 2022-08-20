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
        self.priority=None
        self.path=[]
        self.plan=[]
        self.goal=goal
        self.id=id
        self.tmp_goal=None








