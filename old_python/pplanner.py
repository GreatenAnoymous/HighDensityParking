from struct import unpack
from common import *

from oneshot import *
from search import *

# for well formed instances
class pplaner(object):
    def __init__(self,instance:OneShotInstance):
        self.agents=[]
        self.instance=instance
        print(instance.xmax,instance.ymax)
        self.graph=nx.grid_graph(dim=[instance.ymax,instance.xmax])
        self.v_table=set()
        self.e_table=set()
        self.max_constraint_time=0
        for agent in instance.already_parked_agents:
     
            self.graph.remove_node(agent.loc)
        self.parked_agents=[]

        # for agent in instance.retrieval_agents:
        #     self.agents.append(agent)

        # for agent in instance.parking_agents:
        #     self.agents.append(agent)

    def solve(self):
        unoccupied_slots=self.instance.wellformed_slots
        for agent in self.instance.retrieval_agents:
            # unoccupied_slots.remove(agent.loc)
            self.plan_for_agent(agent)
            self.update_table(agent)

        #check  available slots
        for agent in self.instance.already_parked_agents:
            unoccupied_slots.remove(agent.loc)

        #plan for parking agents
        for agent in self.instance.parking_agents:
            self.greedy_select(agent,unoccupied_slots)
            self.plan_for_agent(agent)
            self.update_table(agent)
            



    def greedy_select(self,agent,unoccupied_slots):
        min_dist=900000000000
        for slot in unoccupied_slots:
            dist=manhattan_distance(agent.loc,slot)
            if dist<min_dist:
                agent.goal=slot
                min_dist=dist
        assert(agent.goal is not None)
        unoccupied_slots.remove(agent.goal)

    
    def plan_for_agent(self,agent):
        assert(agent.goal is not None)
        searcher=TimedSpaceAstar(self.graph,agent.loc,agent.goal,self.v_table,self.e_table,self.max_constraint_time)
        agent.path=searcher.search()
    

    def update_table(self,agent):
        if self.max_constraint_time<len(agent.path):
            self.max_constraint_time=len(agent.path)
            for ai in self.parked_agents:
                li=len(ai.path)
                while li<self.max_constraint_time:
                    self.v_table.add((ai.path[-1],li-1))
                    li=li+1
            
        self.parked_agents.append(agent)
        for t in range(len(agent.path)-1):
            self.v_table.add((agent.path[t],t))
            if t+1<len(agent.path) and agent.path[t]!=agent.path[t+1]:

                self.e_table.add(self.get_edge_obstacle(agent.path[t],agent.path[t+1],t+1))
                following_obs=self.get_following_edge(agent.path[t],agent.path[t+1],t+1)
                for obs in following_obs:
                    self.e_table.add(obs)

        if len(agent.path)<self.max_constraint_time:
            li=len(agent.path)
            while li<self.max_constraint_time:
                self.v_table.add((agent.path[-1],li-1))
                li=li+1

    def get_edge_obstacle(self,v1,v2,t):
        if v1<v2:
            return (v1,v2,t)
        else:
            return (v2,v1,t)

    def get_following_edge(self,v1,v2,t):
        obstacles=[]
        for n in self.graph.neighbors(v1):
            if n==v2 or (n[0]-v1[0])*(v1[1]-v2[1])-(n[1]-v1[1])*(v1[0]-v2[0])==0:
                continue
            obstacles.append(self.get_edge_obstacle(n,v1,t))
        for n in self.graph.neighbors(v2):
            if n==v1 or (n[0]-v2[0])*(v1[1]-v2[1])-(n[1]-v2[1])*(v1[0]-v2[0])==0:
                continue
            obstacles.append(self.get_edge_obstacle(n,v2,t))
        return obstacles

    

    def save_paths_as_json(self,file_name):
        data_dict=dict()
        paths=[]
        for agent in self.instance.retrieval_agents:
            paths.append(agent.path)
        for agent in self.instance.parking_agents:
            paths.append(agent.path)
        for agent in self.instance.already_parked_agents:
            paths.append([agent.loc])
        data_dict["paths"]=paths
        with open(file_name,"w") as fp:
            json.dump(data_dict,fp)


if __name__=="__main__":
    problem=OneShotInstance("./demo/demo_well_formed.json")
    solver=pplaner(problem)
    solver.solve()
    solver.save_paths_as_json("./demo/well_formed_paths.json")