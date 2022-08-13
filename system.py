from queue import PriorityQueue
from common import *

WELL_FORMED=1
HIGH_DENSITY=0
HORIZON=500
MAX_AGENT_ID=200

class System(object):
    def __init__(self) -> None:
        self.xmax=None
        self.ymax=None
        self.all_slots=[]
        self.wellformed_slots=[]
        self.non_wellformed_slots=[]
        self.io_ports=[]
        self.occupancy_map=dict()
        self.removed_robots=[]
        self.clock=0
        self.reservation_table=dict()
        self.density=0
        self.mode=WELL_FORMED
        self.io_ports_occupied=set()
        self.current_agents=dict()
        self.future_plans=dict()
        self.record_paths=[]
        self.avaiable_id=list(range(0,MAX_AGENT_ID))
        self.e_table=set()
        self.v_table=set()
        self.current_configuration=dict()
        hq.heapify(self.avaiable_id)

  


    def run(self):
        while self.clock<=HORIZON:
            self.clock=self.clock+1
            self.mainloop()

    def mainloop(self):
        pass
    

    def get_new_orders(self):
        """_summary_
        check if there is new car needed to park or be retrieved
        """
        pass


    def get_new_car_to_park(self):
        """_summary_
        """
        probability=0.3
        for i in range(len(self.io_ports)):
            eta=np.random.random()
            if eta<probability and i not in self.io_ports_occupied:
                agent_id=hq.heappop(self.avaiable_id)
                self.current_agents[agent_id]=self.io_ports[i]
                
    def execute(self):
        for ai in self.future_plans:
            if len(self.future_plans[ai])!=0:
                current_position=self.future_plans[ai].pop(0)
                self.current_agents[ai]=current_position
    
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
    
    def get_reservation_table(self):
        self.v_table.clear()
        self.e_table.clear()
        max_time=0
        for ai in self.future_plans:
            max_time=max(max_time,len(self.future_plans[ai]))

        for ai in self.future_plans:
            for t in range(max_time):
                ti=min(t,len(self.future_plans[ai])-1)
                self.v_table.add((self.future_plans[ai][ti],ti))

            for t in range(0,len(self.future_plans[ai])-1):
                v1=self.future_plans[ai][t]
                v2=self.future_plans[ai][t+1]
                e_obs=self.get_edge_obstacle(v1,v2,t+1)
                self.e_table.add(e_obs)
                following_obs=self.get_following_edge(v1,v2,t+1)
                for e_obs in following_obs:
                    self.e_table.add(e_obs)
        

    def replan_paths(self):
        pass







                
                
                

