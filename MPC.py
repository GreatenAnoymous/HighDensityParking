from types import TracebackType
from common import *
from .search import AStarSearch
from .oneshot import OneShotInstance



class Vertex(object):
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.visiting_agents=[]
    

class MPC(object):
    def __init__(self,original_paths) -> None:
        self.original_paths=original_paths
        self.vertices=dict()
        self.plans=[[] for i in range(len(self.original_paths))]
        self.location_id=dict()
    
        # self.id_location=dict()
        self.moved=dict()

    def init_schedule(self):
        makespan=len(self.original_paths[0])
        num_robots=len(self.original_paths)
        for t in range(makespan):
            for i in range(num_robots):
                v=self.original_paths[i][t]
                if v not in self.vertices:
                    self.vertices[v]=Vertex(v[0],v[1])
                vertex=self.vertices[v]
                if len(vertex.visiting_agents)==0:
                    vertex.visiting_agents.append(i)
                elif vertex.visiting_agents[-1]!=i:
                    vertex.visiting_agents.append(i)
        for i in range(num_robots):
            self.location_id[self.original_paths[i][0]]=i

    def remove_waiting_for_one_path(self,path):
        i=0
        new_path=[]
        if len(path)==0:
            return
        last_v=path[0]
        new_path.append(last_v)
        while i<len(path):
            v=path[i]
            if v!=last_v:
                last_v=v
                new_path.append(v)
            i=i+1
        return new_path

    def remove_waiting_status(self):
        for i in range(len(self.original_paths)):
            self.original_paths[i]=self.remove_waiting_for_one_path(self.original_paths[i])
        
    def mpc(self):
        while self.check_arrive_goals()==False:
            self.moved.clear()
            for agent in range(len(self.original_paths)):
                self.move(agent)

    def forward_agent(self,agent,curr_v,next_v):
        self.location_id.pop(curr_v)
        self.location_id[next_v]=agent
        self.original_paths[agent].pop(0)
        self.vertices[next_v].visiting_agents.pop(0)
        self.moved[agent]=True
        self.plans[agent].append(next_v)

    def wait_agent(self,agent,curr_v):
        self.moved[agent]=False
        self.plans[agent].append(curr_v)


    def check_perpendicular(self,v1,v2,v3):
        x1,y1=v1
        x2,y2=v2
        x3,y3=v3
        if (x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)==0:
            return True
        return False

    def check_arrive_goals(self):
        for p in self.original_paths:
            if len(p)!=0:
                return False
        return True

    def move(self,agent):
        if agent in self.moved:
            return self.moved[agent]
        if len(self.original_paths[agent])==0:
            return True
        next_v=self.original_paths[agent][0]
        curr_v=self.plans[agent][-1]
        if agent==self.vertices[next_v].visiting_agents[0]:
            if next_v not in self.location_id:
                self.forward_agent(agent,curr_v,next_v)
                return True
            else:
                aj=self.location_id[next_v]
                vj=self.original_paths[aj][0]
                flag=self.move(aj)
                if flag==False:
                    self.moved[agent]=False
                    return False
                elif self.check_perpendicular(curr_v,next_v,vj)==False:
                    self.forward_agent(agent,curr_v,next_v)
                    return True
                else:
                    self.moved[agent]=False
                    return False
                    


class BCPRSolver(object):
    def __init__(self,instance:OneShotInstance) -> None:
        self.instance=instance
        self.agents=[]
        self.clock=0
        self.location_id=dict()
        self.parking_agents=[]
        self.retrieving_agents=[]
        # self.entangled_agents=set()
        self.outOfBoundAgents=set()
        self.moved=dict()

        for agent in self.instance.retrieval_agents:
            self.location_id[agent.loc]=agent.id
            self.agents.append(agent)
            self.retrieving_agents.append(agent)
        # assert(len(self.instance.parking_agents)==0)
        for agent in self.instance.already_parked_agents:
            self.location_id[agent.loc]=agent.id
            self.agents.append(agent)
        print("already parked cars num=",len(self.instance.already_parked_agents))
        for agent in self.instance.parking_agents:
            self.location_id[agent.loc]=agent.id
            self.agents.append(agent)
            self.parking_agents.append(agent)
        self.agents.sort(key=lambda a:a.id)
      
        print("cars need to be parked num=",len(self.instance.parking_agents))

    def fill_paths(self,makespan=-1):
        # makespan=0
        if makespan<0:
            for agent in self.agents:
                makespan=max(len(agent.path),makespan)
        while len(agent.path)<makespan:
            agent.path.append(agent.path[-1])
        


    def ParkingMp(self,agent:Agent):
        
        pass

    def MoveBack(self,agent:int,reserved:set,dx=None):
        xa,ya=self.agents[agent].loc
        if xa==0:
            dx=1
        elif xa==self.instance.xmax-1:
            dx=-1
        next_v=(xa+dx,ya)
        if next_v not in self.location_id and next_v not in reserved:
            self.forward(agent,(xa,ya),next_v)
        elif next_v in self.location_id:
            aj=self.location_id[next_v]
            self.MoveBack(aj,reserved,dx)

        
        

    def RetrieveMp(self,agent:int):
        ai=self.agents[agent]
        self.clock=self.clock+1
        reserved=set()
        while ai.loc!=ai.goal:
            self.moved.clear()
            reserved.clear()
            if ai.loc[1]<self.instance.ymax-2:
                next_v=(ai.loc[0],ai.loc[1]+1)
                next_next_v=(ai.loc[0],ai.loc[1]+2)
                if next_v not in self.location_id:
                    reserved.add(ai.loc)
                    self.forward(agent.id,ai.loc,next_v)
                    reserved.add(next_v)
                else:
                    aj=self.location_id(next_v)
                    self.YieldMp(aj,reserved,dx=None)
                    self.moved[ai.id]=False
                if next_next_v in self.location_id:
                    ajj=self.location_id(next_next_v)
                    self.YieldMp(ajj,reserved,dx=None)
                    self.moved[ai.id]=False
            else:
                x,y=ai.loc
                xg,yg=ai.goal
                if x<xg:
                    dx=1
                else:
                    dx=-1
                while x!=xg:
                    x=x+dx
                    ai.path.append((x,y))
                ai.path.append((xg,yg))
                ai.loc=ai.goal
            for a in self.outOfBoundAgents:
                self.MoveBack(a,reserved)
      

    def forward(self,agent:int,curr:Tuple[int,int],next:Tuple[int,int]):
        self.agents[agent].loc=next
        self.agents[agent].path.append(next)
        self.location_id.pop(curr)
        self.location_id[next]=agent
        self.moved[agent]=True
        if next[0]==0 or next[0]==self.instance.xmax-1:
            self.outOfBoundAgents.add(agent)

        
        
    
    def YieldMp(self,agent:int,reserved:set,dx=None):
        if agent in self.moved:
            return self.moved[agent]
        xa,ya=self.agents[agent].loc
        if (dx is None or dx==1):
            dx=1
            next_v=(xa+dx,ya)
            if next_v not in self.location_id:
                self.forward(agent,(xa,ya),next_v)
            
                return True
            else:
                aj=self.location_id[next_v]
                if self.YieldMp(aj,reserved,dx)==True:
                    self.forward(agent,(xa,ya),next_v)
                    return True
                else:
                    self.moved[agent]=False
                    return False
        elif (dx is None or dx==-1):
            dx=-1
            next_v=(xa+dx,ya)
            if next_v not in self.location_id:
                self.forward(agent,(xa,ya),next_v)
                return True
            else:
                aj=self.location_id[next_v]
                if self.YieldMp(aj,reserved,dx)==True:
                    self.forward(agent,(xa,ya),next_v)
                    return True
                else:
                    self.moved[agent]=False
                    return False


        
        


    
    
                

        


    
        