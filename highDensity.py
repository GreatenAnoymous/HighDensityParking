from turtle import right

from astar import AStar
from sympy import false
from common import *
from oneshot import *
from search import AStarSearch, TimedSpaceAstar

WELL_FORMED=1
HIGH_DENSITY=0
HORIZON=500
MAX_AGENT_ID=200


class highDensityRetrieval(object):
    def __init__(self,instance:OneShotInstance):
        self.agents=[]
        self.instance=instance
        self.clock=0
        self.graph=nx.grid_graph(dim=[instance.ymax,instance.xmax])
        self.v_table=set()
        self.e_table=set()
        self.max_constraint_time=0

        self.recorded_paths=[]
        self.location_id=dict()
        self.id_location=dict()
        self.retrival_agents=dict()     #agent_id and  its goal
        for agent in self.instance.retrieval_agents:
            self.retrival_agents[agent.id]=agent.goal
            self.location_id[agent.loc]=agent.id
            self.id_location[agent.id]=agent.loc
        assert(len(self.instance.parking_agents)==0)
        for agent in self.instance.already_parked_agents:
            self.location_id[agent.loc]=agent.id
            self.id_location[agent.id]=agent.loc
        self.plans=dict()
        self.planned=set()
        # print(self.id_location)
        # exit(0)

    def reached_goals(self):
        for agent,v in self.retrival_agents.items():
            if self.id_location[agent]!=v:
                return False
        return True

    def retrieval_path(self):
        num_agents=len(self.instance.retrieval_agents)+len(self.instance.already_parked_agents)
        paths=[[] for i in range(num_agents)]
        for agent in self.instance.retrieval_agents:
            paths[agent.id].append(agent.loc)
        for agent in self.instance.already_parked_agents:
            paths[agent.id].append(agent.loc)
        for config in self.recorded_paths:
            for p in paths:
                p.append(p[-1])
            for i, v in config.items():
              
                paths[i][-1]=v
           
        sol=dict()
        sol["paths"]=paths
        with open("./full_paths.json","w") as fp:
            json.dump(sol,fp)
        



    def run(self):
        while self.clock<=HORIZON:
            self.clock=self.clock+1
            self.mainloop()
          
            if self.reached_goals():
                print(self.clock)
                print("all agents arrive at goals")
                break
        self.retrieval_path()

    def mainloop(self):
        self.planned.clear()
    
        for agent in self.retrival_agents:
            self.move_agent(agent)
        config=dict()
        for agent, path in self.plans.items():
            if len(path)==0:
                continue
            self.location_id[path[0]]=agent
            self.id_location[agent]=path[0]
            config[agent]=path[0]
            path.pop(0)
        print(config)
        self.recorded_paths.append(config)
        
        
    
    def move_agent(self,agent,direction=None):
        if agent is None:
            return True
        if agent <0:
            return False
        if agent in self.planned: 
            return False
        if agent in self.retrival_agents:
            v=self.id_location[agent]
            if v[1]<=self.instance.ymax-3:
                next_v=(v[0],v[1]+1)
                next_next_v=(v[0],v[1]+2)
                if next_next_v in self.location_id:
                    block_agent=self.location_id[next_next_v]
                    self.move_agent(block_agent)
                if next_v in self.location_id:
                    block_agent=self.location_id[next_v]
                    self.move_agent(block_agent)
                    self.planned.add(agent)
                    self.plans[agent]=[v]
                
                    return False
                else:
                    self.planned.add(agent)
                
                    self.location_id.pop(v)
                    print(agent,"moved to",next_v)
                    self.plans[agent]=[next_v]
                    return True
            elif len(self.plans[agent])==0 and v!=self.retrival_agents[agent]:
                if v in self.location_id:
                    self.location_id.pop(v)
                path=AStarSearch(self.graph,v,self.retrival_agents[agent])
              
                self.plans[agent]=path[1:]
                self.planned.add(agent)
                print("plan new paths",path)
                return True

            else:
                return False
                
        else:
            v=self.id_location[agent]
            left_agent=None
            right_agent=None
            try:
                left_agent=self.location_id[(v[0]-1,v[1])]
            except:
                if v[0]-1<0:
                    left_agent=-1
            try:
                right_agent=self.location_id[(v[0]+1,v[1])]
            except:
                if v[0]+1>=self.instance.xmax:
                    right_agent=-1
            
            if (direction is None or direction=="left") and self.move_agent(left_agent,"left")==True:
                self.plans[agent]=[(v[0]-1,v[1])]
                self.location_id.pop(v)
                self.planned.add(agent)
                print(agent,"at",v,"unlabeled moved to ",(v[0]-1,v[1]),left_agent)
                return True
            elif (direction is None or direction=="right") and self.move_agent(right_agent,"right")==True:
                self.plans[agent]=[(v[0]+1,v[1])]
                self.location_id.pop(v)
                self.planned.add(agent)
                print(agent,"at",v,"unlabeled moved to ",(v[0]+1,v[1]),right_agent)
                return True
            else:
                print(agent,"cannot move")
                return False
            



if __name__=="__main__":
    problem=OneShotInstance("./demo/demo_only_retrieval.json")
    solver=highDensityRetrieval(problem)
    solver.run()
    