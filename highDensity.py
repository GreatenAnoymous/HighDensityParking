from tokenize import Triple
from turtle import right


from common import *
from oneshot import *
from search import AStarSearch, TimedSpaceAstar,TimedSpaceAstarPlus

WELL_FORMED=1
HIGH_DENSITY=0
HORIZON=50
MAX_AGENT_ID=200


class highDensityRetrieval(object):
    def __init__(self,instance:OneShotInstance):
        self.agents=[]
        self.instance=instance
        self.clock=0
        self.graph=nx.grid_graph(dim=[instance.ymax,instance.xmax])
        self.v_table=dict()
        self.e_table=dict()
        self.max_constraint_time=0
        self.row_count=dict()
        self.column_count=dict()
        self.recorded_paths=[]
        self.used_empty_slots=set()
        self.reserved_slots=set()
        self.location_id=dict()
        self.old_location_id=dict()
        # self.old_location_id=dict()
        self.id_location=dict()
        self.agents_out_of_slots=dict()
        self.parking_agents=dict()
        self.retrival_agents=dict()     #agent_id and  its goal
        for agent in self.instance.retrieval_agents:
            self.retrival_agents[agent.id]=agent.goal
            self.location_id[agent.loc]=agent.id
            self.id_location[agent.id]=agent.loc
        # assert(len(self.instance.parking_agents)==0)
        for agent in self.instance.already_parked_agents:
            self.location_id[agent.loc]=agent.id
            self.id_location[agent.id]=agent.loc
        print("already parked cars num=",len(self.instance.already_parked_agents))
        for agent in self.instance.parking_agents:
            self.location_id[agent.loc]=agent.id
            self.id_location[agent.id]=agent.loc
            self.parking_agents[agent.id]=agent.loc
        print("cars need to be parked num=",len(self.instance.parking_agents))
        self.plans=dict()
     
        self.planned=set()
        # print(self.id_location)
        # exit(0)

   
    def count_num_robots_at_row(self,row):
        num=0
        for i in range(0,self.instance.xmax):
            if (i,row) in self.old_location_id:
                num=num+1
        return num

    #add vertex obstacle
    def add_v_obstacle(self,t:int,v:Tuple[int,int]):
        """_summary_

        Args:
            t (int): _description_
            v (Tuple[int,int]): _description_
        """
        if t not in self.v_table:
            self.v_table[t]=set()
        self.v_table[t].add(v)

    def get_edge_obstacle(self,v1:Tuple[int,int],v2:Tuple[int,int]):
        if v1<v2:
            e_obs=(v1,v2)
        else:
            e_obs=(v2,v1)
        return e_obs

    def reserve_path(self,path):
        t=self.clock
        last_v=path[0]
        for v in path:
            self.add_v_obstacle(t,v)
            if v!=last_v:
                self.add_e_obstacle(t,last_v,v)
            t=t+1
            last_v=v

    #add edge obstacle
    def add_e_obstacle(self,t:int,v1:Tuple[int,int],v2:Tuple[int,int]):
        """_summary_

        Args:
            t (int): _description_
            v1 (Tuple[int,int]): _description_
            v2 (Tuple[int,int]): _description_
        """
        if t not in self.e_table:
            self.e_table[t]=set()
       
        self.e_table[t].add(self.get_edge_obstacle(v1,v2))
        for n in self.graph.neighbors(v1):
            if n==v2 or (n[0]-v1[0])*(v1[1]-v2[1])-(n[1]-v1[1])*(v1[0]-v2[0])==0:
                continue
            self.e_table[t].add(self.get_edge_obstacle(n,v1))
            # obstacles.append(self.get_edge_obstacle(n,v1,t))
        for n in self.graph.neighbors(v2):
            if n==v1 or (n[0]-v2[0])*(v1[1]-v2[1])-(n[1]-v2[1])*(v1[0]-v2[0])==0:
                continue
            self.e_table[t].add(self.get_edge_obstacle(n,v2))
            # obstacles.append(self.get_edge_obstacle(n,v2,t))

        


    # find an empty slot at the same cplumn
    def find_empty_slot_at_column(self,current_pos):
        xc,yc=current_pos
        for y in range(self.instance.ymax-3,-1,-1):
            if (xc,y) not in self.location_id and (xc,y) not in self.reserved_slots and (xc,y)  not in self.used_empty_slots:
                self.used_empty_slots.add((xc,y))
                return (xc,y)
        return None

    # find the an empty slot for parking
    def find_empty_slot(self,current_pos):
        xc,yc=current_pos
        for y in range(self.instance.ymax-3,-1,-1):
            if self.count_num_robots_at_row(y)>=self.instance.xmax-2:
                continue
            open=[(xc,y)]
            visited=set()
            while len(open)!=0:
                curr=open.pop(0)
                visited.add(curr)
                if curr not in self.location_id and curr not in self.reserved_slots and curr not in self.used_empty_slots:
                    self.used_empty_slots.add(curr)
                    return curr
                right=(curr[0]+1,curr[1])
                if right[0]<self.instance.xmax-1 and right not in visited:
                    open.append(right)
                left=(curr[0]-1,curr[1])
                if left[0]>=1 and left not in visited:
                    open.append(left)
        return None

    #bring an escort to column 
    def bring_escort_to_column(self,escort:Tuple[int,int],desired_column:int):
        xe,ye=escort
        
        if xe==desired_column:
            return
        if xe<desired_column:
            dx=1
        elif xe>desired_column:
            dx=-1
        curr=xe
        # curr=xe+dx
        while curr!=desired_column:
            #parking first
            print((curr,ye),desired_column,"bring to escort")
            # print((curr,xe))
            if (curr+dx,ye) not in self.location_id:
                print((curr+dx,ye), "not in dict")
                return 
            robot=self.location_id[(curr+dx,ye)]
            if robot in self.planned:
                print(robot,"already planned line 162")
                return 
            next_v=(curr,ye)
            if self.clock+1 in self.v_table and next_v in self.v_table[self.clock+1]:
                print("collisions",(curr+dx,ye),"to",next_v,"already occupied")
                return
            self.forward_agent(robot,(curr+dx,ye),next_v)
            curr=curr+dx
            print("curr=",curr)
            # self.plans[robot]=[()]
            # self.planned.add(robot)



    def bring_escort_to_port(self,escort:Tuple[int,int],curr:Tuple[int,int]):
        xe,ye=escort
        yc=self.instance.ymax-3
        # assert(xe==curr[0])
        ys=ye
        while ys<yc:
            ys=ys+1
            # if (xe,ys) not in self.location_id:
            #     print(escort,(xe,ys),(xe,ys) in self.used_empty_slots)
            # assert((xe,ys) in self.location_id)
            if (xe,ys) not in self.location_id:
                return False
            robot=self.location_id[(xe,ys)]
            if robot in self.planned:
                return False
            next_v=(xe,ys-1)
            if (self.clock+1) in self.v_table and next_v in self.v_table[self.clock+1]:
                return False
            self.forward_agent(robot,(xe,ys),next_v)

        self.reserved_slots.add((xe,yc))
        return True


    def plan_for_parking_agent(self,agent):
        curr=self.id_location[agent]
        if curr in self.instance.all_slots:
            self.parking_agents.pop(agent)
            if curr in self.reserved_slots:
                self.reserved_slots.remove(curr)
            return
        escort=self.find_empty_slot_at_column(curr)
        if escort is not None:
            print("the escort is", escort,"for agent ",agent,"at",curr)
            flag=self.bring_escort_to_port(escort,curr)
            if flag==True:
                solver=TimedSpaceAstarPlus(self.graph,curr,(curr[0],self.instance.ymax-3),self.v_table,self.e_table)
                path=solver.search()
            
                self.plans[agent]=path[1:]
                self.reserve_path(path)
            else:
                return
            #plan using timespace A *
        elif escort is None:
            escort=self.find_empty_slot(curr)
            if escort is not None:
                print("the escort X is", escort,"for agent ",agent,"at",curr)
                self.bring_escort_to_column(escort,curr[0])
            




    def reached_goals(self):
        for agent,v in self.retrival_agents.items():
            if self.id_location[agent]!=v:
                return False
        for agent,start in self.parking_agents.items():
            v=self.id_location[agent]
            if v not in self.instance.all_slots:
                return False
            
        return True

    def retrieval_path(self):
        num_agents=len(self.instance.retrieval_agents)+len(self.instance.already_parked_agents)+len(self.instance.parking_agents)
        paths=[[] for i in range(num_agents)]
        for agent in self.instance.retrieval_agents:
            paths[agent.id].append(agent.loc)
        for agent in self.instance.already_parked_agents:
            paths[agent.id].append(agent.loc)
        for agent in self.instance.parking_agents:
            paths[agent.id].append(agent.loc)
        for config in self.recorded_paths:
            for p in paths:
                p.append(p[-1])
            for i, v in config.items():
                # print(i,v)
                paths[i][-1]=v
           
        sol=dict()
        sol["paths"]=paths
        file_name="./demo/parking_retrieval_paths.json"
        with open(file_name,"w") as fp:
            json.dump(sol,fp)
        
    def parking_primitive(self):
        pass


    def run(self):
        while self.clock<=HORIZON:
            self.clock=self.clock+1
            self.mainloop()
            if self.reached_goals():
                # print(self.clock)
                # print("all agents arrive at goals")
                break
            print("*******************************\n")
        self.retrieval_path()

    def mainloop(self):
        self.planned.clear()
     
        self.used_empty_slots.clear()
        for agent in self.parking_agents.copy():
            if  agent not in self.plans or len(self.plans[agent])==0:
                self.plan_for_parking_agent(agent)

        for agent in self.retrival_agents:
            print("moving agent",agent)
            flag=self.move_agent(agent)
            # if flag==False:
            #     print("moving agent",agent,"failed")
        config=dict()
        for agent, path in self.plans.items():
            if len(path)==0:
                continue
            last_v=self.id_location[agent]
            # if last_v in self.location_id:
            #     self.location_id.pop(last_v)
            #     print("poped",last_v,agent,path[0])
            self.location_id[path[0]]=agent
            self.id_location[agent]=path[0]
            # if path[0] is None:
            #     print(path,agent)
            #     assert(path[0] is not None)
            config[agent]=path[0]
            path.pop(0)
        # print(config)
        self.recorded_paths.append(config)
        
    def wait_agent(self,agent,curr=None):
        self.planned.add(agent)
        self.plans[agent]=[curr]
        self.add_v_obstacle(self.clock+1,curr)
        
    def forward_agent(self,agent,curr_v,next_v):
        self.planned.add(agent)
        self.plans[agent]=[next_v]
        # assert(next_v is not None)
        self.add_e_obstacle(self.clock+1,curr_v,next_v)
        self.add_v_obstacle(self.clock+1,next_v)
        if curr_v in self.location_id:
            self.location_id.pop(curr_v)
    
    def move_agent(self,agent,direction=None):
        
        if agent is None:
            return True
        
        if agent <0:
            return False
        if agent in self.planned: 
            print("agent ",agent," is planned")
            return False
        print("function move agent is called for",agent,self.id_location[agent])
        if agent in self.retrival_agents:
            v=self.id_location[agent]
            if v[1]<=self.instance.ymax-3:
                next_v=(v[0],v[1]+1)
                next_next_v=(v[0],v[1]+2)
                
                if next_v in self.location_id:
                    print("front for agent",agent," is blocked by",self.location_id[next_v],"at",next_v,self.id_location[self.location_id[next_v]])
                    block_agent=self.location_id[next_v]
                    self.move_agent(block_agent)
                    self.wait_agent(agent,v)
                    if next_next_v in self.location_id:
                        block_agent=self.location_id[next_next_v]
                        print("moving next next block agent",block_agent,next_next_v,self.id_location[block_agent])
                        self.move_agent(block_agent)
                
                    return False
                elif (self.clock+1) in self.e_table and self.get_edge_obstacle(v,next_v) in self.e_table[self.clock+1]:
                    self.planned.add(agent)
                    self.plans[agent]=[v]
                    # assert(v is not None)
                    self.wait_agent(agent,v)
                    # if next_next_v in self.location_id:
                    #     block_agent=self.location_id[next_next_v]
                    #     print("moving next next block agent",block_agent,next_next_v)
                    #     self.move_agent(block_agent)
                    return False
                    # self.forward_agent(agent,v,next_v)
                
                else:
                    self.planned.add(agent)
                
                    # self.location_id.pop(v)
                    # print(agent,"moved to",next_v)
                    self.forward_agent(agent,v,next_v)
                    # self.plans[agent]=[next_v]
                    # self.add_v_obstacle(self.clock,next_v)
                    # self.add_e_obstacle(self.clock,v,next_v)
                    return True
            elif len(self.plans[agent])==0 and v!=self.retrival_agents[agent]:
                if v in self.location_id:
                    self.location_id.pop(v)
                # path=AStarSearch(self.graph,v,self.retrival_agents[agent])
                solver=TimedSpaceAstarPlus(self.graph,v,self.retrival_agents[agent],self.v_table,self.e_table,self.clock)
                path=solver.search()
                self.reserve_path(path)
                self.plans[agent]=path[1:]
                self.planned.add(agent)
                
                return True
            elif v==self.retrival_agents[agent]:
                return True
            elif len(self.plans[agent])!=0:
                if v in self.location_id:
                    self.location_id.pop(v)
                return True

            else:
                print("retrieval agent ",agent,"not move")
                return False
                
        else:
            print("!!!moving agent  ",agent)
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
                next_v=(v[0]-1,v[1])
                e_obs=dict()
                v_obs=dict()
                if self.clock+1 in self.e_table:
                    e_obs=self.e_table[self.clock+1]
                    v_obs=self.v_table[self.clock+1]
                if self.get_edge_obstacle(v,next_v) not in e_obs and next_v not in v_obs and next_v not in self.reserved_slots:
                    self.forward_agent(agent,v,next_v)
                    # self.plans[agent]=[]
                    # self.location_id.pop(v)
                    # self.planned.add(agent)
                    print(agent,"at",v,"left moved to ",(v[0]-1,v[1]),left_agent)
                    if agent==40:
                        print((v[0],v[1]) in self.location_id)
                        print(self.id_location[21])
                    return True
            if (direction is None or direction=="right")  and self.move_agent(right_agent,"right")==True:
                next_v=(v[0]+1,v[1])
                e_obs=dict()
                v_obs=dict()
                if self.clock+1 in self.e_table:
                    e_obs=self.e_table[self.clock+1]
                    v_obs=self.v_table[self.clock+1]
                if self.get_edge_obstacle(v,next_v) not in e_obs and next_v not in v_obs and next_v not in self.reserved_slots:
                    self.forward_agent(agent,v,next_v)
                    # self.plans[agent]=[(v[0]+1,v[1])]
                    # self.location_id.pop(v)
                    # self.planned.add(agent)
                    print(agent,"at",v,"right moved to ",(v[0]+1,v[1]),right_agent)
                    return True
            print("agent",agent," wait at its current vertex",v)
            # self.planned.add(agent)
            self.wait_agent(agent,v)
           
            return False
            



if __name__=="__main__":
    # problem=OneShotInstance("./demo/demo_only_retrieval.json")
    # problem=OneShotInstance("./demo/demo_parking.json")
    problem=OneShotInstance("./demo//demo_parking_retrieval.json")
    # problem=OneShotInstance("./demo//demo_retrieve_one_car.json")
    solver=highDensityRetrieval(problem)
    solver.run()
    