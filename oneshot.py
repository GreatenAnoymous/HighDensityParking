from os import system
from common import *

WELL_FORMED=1
HIGH_DENSITY=0

class OneShotInstance(object):

    def save_as_json(self,file_name):
        data_dict=dict()
        data_dict["xmax"]=self.xmax
        data_dict["ymax"]=self.ymax
        data_dict["num_retrieval"]=len(self.retrieval_agents)
        data_dict["num_parking"]=len(self.parking_agents)
        data_dict["other_starts"]=len(self.already_parked_agents)
        data_dict["ParkingSlots"]=self.all_slots
        data_dict["IOports"]=self.io_ports
        data_dict["retrieval_starts"]=[agent.loc for agent in self.retrieval_agents]
        data_dict["retrieval_goals"]=[agent.goal for agent in self.retrieval_agents]
        data_dict["parking_starts"]=[agent.loc for agent in self.parking_agents]
        data_dict["other_starts"]=[agent.loc for agent in self.already_parked_agents]
        with open(file_name,"w") as fp:
            json.dump(data_dict,fp)
        
        
    def __init__(self,file_name=None):
        if file_name is None:
            self.retrieval_agents=[]
            self.parking_agents=[]
            self.already_parked_agents=[]
            self.xmax=None
            self.ymax=None
            self.all_slots=[]
            self.wellformed_slots=[]
            self.non_wellformed_slots=[]
            self.io_ports=[]
        else:
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
