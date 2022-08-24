from cgi import test
from os import system
from common import *

WELL_FORMED=1
HIGH_DENSITY=0

class OneShotInstance(object):
    def __init__(self):
        self.retrieval_agents=[]
        self.parking_agents=[]
        self.already_parked_agents=[]
        self.xmax=None
        self.ymax=None
        self.all_slots=[]
        self.wellformed_slots=[]
        self.non_wellformed_slots=[]
        self.io_ports=[]
        
    def __init__(self,file_name):
        f1=open(file_name)
        system_info=json.load(f1)
        self.retrieval_agents=[]
        k=0
        for start,goal in zip(system_info["retrieval_starts"],system_info["retrieval_goals"]):
            agent=Agent(tuple(start),tuple(goal),k)
            k=k+1
            self.retrieval_agents.append(agent)
        self.xmax=system_info["xmax"]
        self.ymax=system_info["ymax"]
        self.all_slots=[tuple(v) for v in system_info["ParkingSlots"]]
        self.io_ports=[tuple(v) for v in system_info["IOports"]]
        try:
            self.wellformed_slots=[tuple(v) for v in system_info["WellformedSlots"]]
        except:
            pass
        self.parking_agents=[]
        self.already_parked_agents=[]
        
        for start in system_info["other_starts"]:
            agent=Agent(tuple(start),None,k)
            k=k+1
            self.already_parked_agents.append(agent)
        
        for start in system_info["parking_starts"]:
            agent=Agent(tuple(start),None,k)
            k=k+1
            self.parking_agents.append(agent)
                
            



        
if __name__=="__main__":
    test_instance=OneShotInstance("./test_wellformed_instance.json")
    print(test_instance.xmax,test_instance.ymax)
