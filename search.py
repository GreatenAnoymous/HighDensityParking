from common import *
from queue import PriorityQueue


def AStarSearch(graph:nx.graph,start:Tuple,goal:Tuple)->List:
    """_summary_

    Args:
        graph (nx.graph): _description_
        start (Tuple): _description_
        goal (Tuple): _description_
        rt_table (Tuple): _description_

    Returns:
        List: _description_
    """
    frontier = PriorityQueue()
    frontier.put(start, 0)
    parent = {start: None}
    g_value = {start: 0}
    # Start an A* search
    while not frontier.empty():
        current_v = frontier.get()
        # If goal reached, return path length
        if current_v == goal:
            path=[]
            timesteps = 0
            while current_v != start:
                path.append(current_v)
                current_v = parent[current_v]
                
                timesteps += 1
            path.append(start)
            path.reverse()
            return path
        # Expand the current node
        for next_v in graph[current_v]:
            new_cost = g_value[current_v] + 1
            if next_v not in g_value or new_cost < g_value[next_v]:
                g_value[next_v] = new_cost
                priority = new_cost + manhattan_distance(next_v, goal)
                frontier.put(next_v, priority)
                parent[next_v] = current_v
    # Goal not reached, raise exception
    raise ValueError

class AStarNode(object):
    def __init__(self,loc,f,t,parent=None):
        self.loc=loc
        self.f=f
        self.t=t
        self.parent=parent

    
    def __cmp__(self,other):
        if self.f!=other.f:
            return self.f<other.f
        else:
            return self.t>other.t

    def __lt__(self,other):
        if self.f!=other.f:
            return self.f<other.f
        else:
            return self.t>other.t

class TimedSpaceAstar(object):
    def __init__(self,graph:nx.Graph,start,goal,v_table,e_table,max_time):
        self.graph=graph
        self.start=start
        self.goal=goal
        self.v_table=v_table
        self.e_table=e_table
        self.max_time=max_time

    def search(self):
        openList=PriorityQueue()
        root=AStarNode(self.start,manhattan_distance(self.start,self.goal),0,None)
        openList.put(root)
        closed=set()
        print("searching",self.start,self.goal)
        while not openList.empty():
            best=openList.get()
            print(best.loc,"t=",best.t,"f=",best.f,self.goal,best.loc)
            if best.loc==self.goal and best.t>=self.max_time:
                print("found solutions")
                path=[]
                current_v=best
                while current_v is not None:
                    path.append(current_v.loc)
                    # print(current_v.loc,current_v.parent.loc)
                    current_v=current_v.parent
                    
                path.reverse()
                return path
            #Expand the node
            closed.add((best.loc,best.t))
            # nbrs=self.graph.neighbors(best.loc)
            nbrs=[n for n in self.graph.neighbors(best.loc)]
            
            nbrs.append(best.loc)
            for nbr in nbrs:
                t1=best.t+1
                if (nbr,t1) in closed:
                    continue
               
                vertexObs=(nbr,min(t1,self.max_time-1))
                if vertexObs in self.v_table:
                    continue
                
                if best.loc<nbr:
                    edgeObs=(best.loc,nbr,t1)
                else:
                    edgeObs=(nbr,best.loc,t1)
                if t1<self.max_time and edgeObs in self.e_table:
                    continue
                childNode=AStarNode(nbr,t1+manhattan_distance(nbr,self.goal),t1,best)
                openList.put(childNode)
        print("Failed to find a solution")
        return None




class TimedSpaceAstar(object):
    def __init__(self,graph:nx.Graph,start,goal,v_table,e_table,max_time):
        self.graph=graph
        self.start=start
        self.goal=goal
        self.v_table=v_table
        self.e_table=e_table
        self.max_time=max_time

    def search(self):
        openList=PriorityQueue()
        root=AStarNode(self.start,manhattan_distance(self.start,self.goal),0,None)
        openList.put(root)
        closed=set()
        print("searching",self.start,self.goal)
        while not openList.empty():
            best=openList.get()
            print(best.loc,"t=",best.t,"f=",best.f,self.goal,best.loc)
            if best.loc==self.goal and best.t>=self.max_time:
                print("found solutions")
                path=[]
                current_v=best
                while current_v is not None:
                    path.append(current_v.loc)
                    # print(current_v.loc,current_v.parent.loc)
                    current_v=current_v.parent
                    
                path.reverse()
                return path
            #Expand the node
            closed.add((best.loc,best.t))
            # nbrs=self.graph.neighbors(best.loc)
            nbrs=[n for n in self.graph.neighbors(best.loc)]
            
            nbrs.append(best.loc)
            for nbr in nbrs:
                t1=best.t+1
                if (nbr,t1) in closed:
                    continue
               
                vertexObs=(nbr,min(t1,self.max_time-1))
                if vertexObs in self.v_table:
                    continue
                
                if best.loc<nbr:
                    edgeObs=(best.loc,nbr,t1)
                else:
                    edgeObs=(nbr,best.loc,t1)
                if t1<self.max_time and edgeObs in self.e_table:
                    continue
                childNode=AStarNode(nbr,t1+manhattan_distance(nbr,self.goal),t1,best)
                openList.put(childNode)
        print("Failed to find a solution")
        return None
    

class TimedSpaceAstarPlus(object):
    def __init__(self,graph:nx.Graph,start,goal,v_table,e_table,start_time=0):
        self.graph=graph
        self.start=start
        self.goal=goal
        self.v_table=v_table
        self.e_table=e_table
        self.start_time=start_time
        # self.max_time=max_time

    def search(self):
        openList=PriorityQueue()
        root=AStarNode(self.start,manhattan_distance(self.start,self.goal),self.start_time,None)
        openList.put(root)
        closed=set()
        # print("searching",self.start,self.goal)
        while not openList.empty():
            best=openList.get()
            print(best.loc,"t=",best.t,"f=",best.f,self.goal,best.loc)
            if best.loc==self.goal:
                print("found solutions")
                path=[]
                current_v=best
                while current_v is not None:
                    path.append(current_v.loc)
                    # print(current_v.loc,current_v.parent.loc)
                    current_v=current_v.parent
                    
                path.reverse()
                return path
            #Expand the node
            closed.add((best.loc,best.t))
            # nbrs=self.graph.neighbors(best.loc)
            nbrs=[n for n in self.graph.neighbors(best.loc)]
            
            nbrs.append(best.loc)
            for nbr in nbrs:
                t1=best.t+1
                if (nbr,t1) in closed:
                    continue
               
                
                if t1 in self.v_table and nbr in self.v_table[t1]:
                    continue
                
                if best.loc<nbr:
                    edgeObs=(best.loc,nbr)
                else:
                    edgeObs=(nbr,best.loc)
                if t1 in self.e_table and edgeObs in self.e_table[t1]:
                    continue
                childNode=AStarNode(nbr,t1+manhattan_distance(nbr,self.goal),t1,best)
                openList.put(childNode)
        print("Failed to find a solution")
   
        return None