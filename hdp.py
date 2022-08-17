from common import *



class HDP(object):
    def __init__(self):

        pass

    def __init__(self,json_name) -> None:
        f1=open(json_name)
        system_info=json.load(f1)
        self.xmax=system_info["xmax"]
        self.ymax=system_info["ymax"]

        self.clock=0
        self.grid=None
        self.v_table=dict()
        self.e_table=dict()
        self.recordedPaths=[]
        self.currentAgents=[]
        self.id_location=dict()
        self.location_id=dict()
        self.agents=[]
        self.emptySlots=set()
        # self.y_boarder=self.
        # self.id_vertex_map=dict()
        # self.vertex_id_map=dict()

    def getVertex(self,id):
        x=id/self.ymax
        y=id%self.xmax
        return (x,y)

    def getId(self,vertex):
        return vertex[0]*self.ymax+vertex[1]

    def fill_paths(self):
        makespan=0
        for agent in self.agents:
            makespan=max(len(agent.path),makespan)
        while len(agent.path)<makespan:
            agent.path.append(agent.path[-1])


    def parkingForOneCar(self,i):
        agent=self.agents[i]
        ex,ey=self.greedyGetEmptySlot(agent.loc)


    def greedyGetEmptySlot(self,location:Tuple)->Tuple[int,int]:
        """_summary_
        Args:
            location (Tuple):  the location of the car that should be parked

        Returns:
            Tuple[int,int]: the greedy closest empty parking slot
        """

        min_dist=99999
        best_v=None
        for v in self.emptySlots:
            dist_x=manhattan_distance(v,location)
            if dist_x<min_dist:
                best_v=v
        return best_v


        
        

    

    def retrieveOneCar(self,i):
        pass


    def MPC(self,paths):
        pass


