from pickle import FALSE
from tkinter.tix import MAX
from turtle import right
from types import TracebackType
from common import *
from search import AStarSearch
from oneshot import OneShotInstance

MAX_HORIZON=500

class Vertex(object):
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.visiting_agents=[]
    

class MPC(object):
    def __init__(self,original_paths) -> None:
        self.original_paths=original_paths
        self.vertices=dict()
        self.plans=[[self.original_paths[i][0]] for i in range(len(self.original_paths))]
        self.location_id=dict()
        self.old_location_id=dict()
        self.v_obs=set()
        self.e_obs=set()
        # self.id_location=dict()
        self.moved=dict()

    def get_next_v(self,agent):
        if agent in self.moved:
            return self.plans[agent][-1]
        else:
            return self.original_paths[agent][0]

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
            starti=self.original_paths[i][0]
            self.original_paths[i].pop(0)
    
            self.vertices[starti].visiting_agents.pop(0)
        
    def mpcSolve(self):
        self.init_schedule()
        self.remove_waiting_status()
        k=0
        while self.check_arrive_goals()==False:
            self.moved.clear()
            self.v_obs.clear()
            self.old_location_id=self.location_id.copy()
            k=k+1
            print(k,'*****************')
            if k>MAX_HORIZON:
                break
            for agent in range(len(self.original_paths)):
                self.move(agent)
            print()
        sol=dict()
    
        
        sol["paths"]=self.plans
        
        with open("./demo/mpc.json","w") as fp:
            json.dump(sol,fp)

    def forward_agent(self,agent,curr_v,next_v):
        if self.location_id[curr_v]==agent:
            self.location_id.pop(curr_v)
        self.location_id[next_v]=agent
   
        self.original_paths[agent].pop(0)
        self.vertices[next_v].visiting_agents.pop(0)
        self.moved[agent]=True
        self.plans[agent].append(next_v)
        self.v_obs.add(next_v)

    def wait_agent(self,agent,curr_v):
        self.moved[agent]=False
        self.plans[agent].append(curr_v)
        self.v_obs.add(curr_v)


    def check_perpendicular(self,v1,v2,v3):
        x1,y1=v1
        x2,y2=v2
        x3,y3=v3
        if (x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)!=0:
            print("perpendicular following conflicts")
            return True
        return False

    def check_arrive_goals(self):
        for p in self.original_paths:
            if len(p)!=0:
                return False
        return True

    def move(self,agent):
      
        # print("moving agent",agent)
        if agent in self.moved:
            return self.moved[agent]
        if len(self.original_paths[agent])==0:
         
            self.moved[agent]=False
            return False
        next_v=self.original_paths[agent][0]
        curr_v=self.plans[agent][-1]
        if agent==self.vertices[next_v].visiting_agents[0]:
            if next_v not in self.old_location_id:
                if  next_v not in self.v_obs:
                    self.forward_agent(agent,curr_v,next_v)
                    return True
                else:
                    self.wait_agent(agent,curr_v)
                    return False
            else:
                aj=self.old_location_id[next_v]
                # print(aj,agent,self.original_paths[aj])
                # if len(self.original_paths[aj])!=0:
                print("debug",aj,agent,self.original_paths[aj],self.original_paths[agent])
                # vj=self.original_paths[aj][0]
                vj=self.get_next_v(aj)
                flag=self.move(aj)
                if flag==False:
                    self.wait_agent(agent,curr_v)
                    return False
                else:
                 
                    if  self.check_perpendicular(curr_v,next_v,vj)==False and next_v not in self.v_obs:
                        self.forward_agent(agent,curr_v,next_v)
                        return True
                    else:
                        self.wait_agent(agent,curr_v)
                        return False
        else:
            print(agent,curr_v,next_v,self.vertices[next_v].visiting_agents)
            print("not the correct ordering ")
            self.wait_agent(agent,curr_v)
            return False
                    


class BCPRSolver(object):
    def __init__(self,instance:OneShotInstance) -> None:
        self.instance=instance
        self.agents=[]
        self.clock=1
        self.location_id=dict()
        self.parking_agents=[]
        self.retrieving_agents=[]
        # self.entangled_agents=set()
        self.outOfBoundAgents=set()
        self.moved=dict()

        self.instance.retrieval_agents.sort(key=lambda agent:-agent.loc[1])

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
        for agent in self.agents:
            agent.path.append(agent.loc)
        self.agents.sort(key=lambda a:a.id)
      
        print("cars need to be parked num=",len(self.instance.parking_agents))

    def fill_paths(self,makespan=-1):
        # makespan=0
        if makespan<0:
            for agent in self.agents:
                makespan=max(len(agent.path),makespan)
        for agent in self.agents:
            while len(agent.path)<makespan:
                agent.path.append(agent.path[-1])
    
    def save_as_json(self,file_name):
        sol=dict()
        paths=[]
        for agent in self.agents:
            paths.append(agent.path)
        sol["paths"]=paths
        
        with open(file_name,"w") as fp:
            json.dump(sol,fp)

    def solve(self):
        for agent in self.retrieving_agents:
            self.RetrieveMp(agent.id)
        print("solved")
        self.save_as_json("./demo/single.json")
        

    def ParkingMp(self,agent:Agent):
        
        pass

    def MoveBack(self,agent:int,reserved:set,dx=None):
        xa,ya=self.agents[agent].loc
        if dx is None:
            if xa==0:
                dx=1
            elif xa==self.instance.xmax-1:
                dx=-1
            else:
                self.outOfBoundAgents.remove(agent)
                return False
        print("debug",(xa,ya),dx,agent in self.outOfBoundAgents,"agent=",agent)
        next_v=(xa+dx,ya)
        print("?????")
        if next_v not in self.location_id and next_v not in reserved:
            self.forward(agent,(xa,ya),next_v)
        elif next_v in self.location_id:
            aj=self.location_id[next_v]
            self.MoveBack(aj,reserved,dx)

        
        

    def RetrieveMp(self,agent:int):
        ai=self.agents[agent]
        
        reserved=set()
        while ai.loc!=ai.goal:
            
            if self.clock>MAX_HORIZON:
                break
            self.moved.clear()
            reserved.clear()
            if ai.loc[1]<self.instance.ymax-2:
                self.clock=self.clock+1
                next_v=(ai.loc[0],ai.loc[1]+1)
                next_next_v=(ai.loc[0],ai.loc[1]+2)
                if next_v not in self.location_id:
                    reserved.add(ai.loc)
                    self.forward(ai.id,ai.loc,next_v)
                    reserved.add(next_v)
                else:
                    aj=self.location_id[next_v]
                    self.YieldMp(aj,reserved)
                    self.moved[ai.id]=False
                if next_next_v[1]<self.instance.ymax-1 and next_next_v in self.location_id:
                    ajj=self.location_id[next_next_v]
                    # print("yielding for ajj",ajj,next_next_v,self.agents[ajj].loc)
                    self.YieldMp(ajj,reserved)
                    
                    self.moved[ai.id]=False
                
            else:
                x,y=ai.loc
                self.location_id.pop((x,y))
                xg,yg=ai.goal
                if x<xg:
                    dx=1
                else:
                    dx=-1
                while x!=xg:
                    self.clock=self.clock+1
                    x=x+dx
                    ai.path.append((x,y))
                self.clock=self.clock+1
                ai.path.append((xg,yg))
                ai.loc=ai.goal
                
                self.location_id[ai.loc]=ai.id
            # for a in self.outOfBoundAgents.copy():
            #     if a in self.moved:
            #         continue
            #     self.MoveBack(a,reserved)
                # if self.agents[a].loc[0]>0 and self.agents[a].loc[0]<self.instance.xmax-1:
                #     self.outOfBoundAgents.remove(a)
                
            self.fill_paths(self.clock)
            print("timestep=",self.clock)

      

    def forward(self,agent:int,curr:Tuple[int,int],next:Tuple[int,int]):
        self.agents[agent].loc=next
        self.agents[agent].path.append(next)
        self.location_id.pop(curr)
        print(curr,"is popoed for agent",agent,"move to ",next)
        self.location_id[next]=agent
        self.moved[agent]=True
        if next[0]==0 or next[0]==self.instance.xmax-1:
            print("agent out of bounds, agent=",agent ,next)
            self.outOfBoundAgents.add(agent)

        
        
    
    def YieldMp(self,agent:int,reserved:set,direction=None):
        if agent in self.moved:
            return self.moved[agent]
        xa,ya=self.agents[agent].loc
        # assert((xa,ya) in self.location_id)
        if xa>self.instance.xmax/2:
            first_direc="right"
            fdx=1
            second_direc="left"
            sdx=-1
        else:
            first_direc="left"
            second_direc="right"
            fdx=-1
            sdx=1
        if (direction is None or direction==first_direc):
            
            next_v=(xa+fdx,ya)
            if next_v not in self.location_id and (next_v[0]>=0 and next_v[0]<self.instance.xmax):
                self.forward(agent,(xa,ya),next_v)
                return True
            elif next_v in self.location_id:
                assert(next_v[0]>=0 and next_v[0]<self.instance.xmax)
                aj=self.location_id[next_v]
                if self.YieldMp(aj,reserved,first_direc)==True:
                    self.forward(agent,(xa,ya),next_v)
                    return True
            # print("will this be called?")
              
        if (direction is None or direction==second_direc):
          
          
            next_v=(xa+sdx,ya)
            if next_v not in self.location_id and (next_v[0]>=0 and next_v[0]<self.instance.xmax):
                self.forward(agent,(xa,ya),next_v)
                return True
            elif next_v in self.location_id:
                aj=self.location_id[next_v]
                if self.YieldMp(aj,reserved,second_direc)==True:
                    self.forward(agent,(xa,ya),next_v)
                    return True

        self.moved[agent]=False
        return False


        
        
if __name__=="__main__":
    problem=OneShotInstance("./demo/demo_only_retrieval.json")
    solver=BCPRSolver(problem)
    solver.solve()
    paths=[]
    for a in  solver.agents:
        paths.append(a.path)
    mpc=MPC(paths)
    mpc.mpcSolve()



    
    
                

        


    
        