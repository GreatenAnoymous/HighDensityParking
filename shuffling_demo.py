from MPC import MPC,Vertex
from common import *
import time


class shuffle_demo(object):
    def __init__(self,xmax,ymax):
        self.xmax=xmax
        self.ymax=ymax
        self.io_ports=[(x,self.ymax-1) for x in range(1,xmax-1)]
        self.agents=[]
        self.col_agents=dict()
        self.create_random_config()
    

    
    def create_random_config(self):
        slots=[(x,y) for x in range(1,self.xmax-1) for y in range(0,self.ymax-2)]
        np.random.shuffle(slots)
        ranking=list(range(len(slots)))
        np.random.shuffle(ranking)
        k=0
        for slot in slots:
            ai=Agent(loc=slot,goal=None,id=k)
            ai.priority=ai.id
            k=k+1
            # ai.priority=ranking.pop()
            self.agents.append(ai)
            if ai.loc[0] not in self.col_agents:
                self.col_agents[ai.loc[0]]=[]
            self.col_agents[ai.loc[0]].append(ai)

    def save_system_info(self,file_name):
        data_dict=dict()
        data_dict["xmax"]=self.xmax
        data_dict["ymax"]=self.ymax
        data_dict["num_retrieval"]=0
        data_dict["num_parking"]=0
        data_dict["IOports"]=self.io_ports
        data_dict["retrieval_starts"]=[]
        data_dict["retrieval_goals"]=[]
        data_dict["ParkingSlots"]=[(x,y) for x in range(1,self.xmax-1) for y in range(0,self.ymax-2)]
        with open(file_name,"w") as fp:
            json.dump(data_dict,fp)
        
    def save_paths_as_json(self,file_name):
        paths=[]
        for agent in self.agents:
            paths.append(agent.path)
        data_dict=dict()
        data_dict["paths"]=paths
        with open(file_name,"w") as fp:
            json.dump(data_dict,fp)
            

    def rearrange(self):
        for col in range(1,self.xmax-1):
            self.shuffle(col)
        
    
    # shuffle the agents in coulun
    def shuffle(self,column):
        self.prepare(column)
        self.sorting(column)
        self.move_back(column)
    
    def prepare(self,column):
        for agent in self.agents:
      
            if agent.loc[0]<column:
                agent.path.append((agent.loc[0]-1,agent.loc[1]))
                agent.loc=agent.path[-1]
            elif agent.loc[0]==column:
                agent.path.append(agent.loc)
            elif agent.loc[0]>column:
                agent.path.append((agent.loc[0]+1,agent.loc[1]))
                agent.loc=agent.path[-1]
        # self.fill_paths()
                
    def sorting(self,column):
        agents_at_c=self.col_agents[column]
        agents_at_c.sort(key=lambda a: a.priority)
        for k in range(len(agents_at_c)):
            ak=agents_at_c[k]
            xc,yc=agents_at_c[k].loc
            if k<yc:
                ak.path.append((xc-1,yc))
                while yc!=k:
                    yc=yc-1
                    ak.path.append((xc-1,yc))
                ak.path.append((xc,k))
                ak.loc=(xc,k)
            elif k>yc:
                assert(yc<self.ymax-1)
                ak.path.append((xc+1,yc))
                while yc!=k:
                    yc=yc+1
                    ak.path.append((xc+1,yc))
                ak.path.append((xc,k))
                ak.loc=(xc,k)
        self.fill_paths()
            
    def move_back(self,column):
        for agent in self.agents:

            if agent.loc[0]<column:
                agent.path.append((agent.loc[0]+1,agent.loc[1]))
                agent.loc=agent.path[-1]
            elif agent.loc[0]==column:
                agent.path.append(agent.loc)
            elif agent.loc[0]>column:
                agent.path.append((agent.loc[0]-1,agent.loc[1]))
                agent.loc=agent.path[-1]
        
    def fill_paths(self,makespan=-1):
        # makespan=0
        if makespan<0:
            for agent in self.agents:
                makespan=max(len(agent.path),makespan)
        for agent in self.agents:
            while len(agent.path)<makespan:
                agent.path.append(agent.path[-1])   
    
    def get_paths(self):
        # for i in range(len(self.agents)):
        #     print(i,self.agents[i].path)
        # print()
        # print("=====================") 
        
        paths=[]
        for agent in self.agents:
            paths.append(agent.path)
        return paths
    
    def evaluateRT(self):
        paths=self.get_paths()
        makespan,makespanLB,sod,sor=evaluate_paths(paths)
        print("makespan",makespan)
        print("sod",sod/len(paths))
        print("sor",sor/len(paths))
    
    def save_shuffled_results(self,file_name):
        data_dict=dict()
        agents=[]
        for agent in self.agents:
            agent_dict=dict()
            print(agent.id)
            agent_dict["id"]=agent.id
            agent_dict["loc"]=agent.loc
            agent_dict["priority"]=agent.priority
            agents.append(agent_dict)
        data_dict["agents"]=agents
        with open(file_name,"w") as fp:
            json.dump(data_dict,fp)
        
            
        
if __name__=="__main__":
    shuffler=shuffle_demo(22,22)
    t0=time.time()
    shuffler.rearrange()
    t1=time.time()-t0
    shuffler.evaluateRT()
    # print("comp_time",t1 )
    shuffler.save_shuffled_results("./demo/demo_shuffle/shuffled_config.json")
    shuffler.save_system_info("./demo/demo_shuffle/shuffle_system.json")
    # mpc_solver=MPC(shuffler.get_paths())
    
    # mpc_solver.mpcSolve()
    # mpc_solver.save_plans_as_json("./demo/demo_shuffle/shuffle_paths.json")
    # shuffler.save_paths_as_json("./demo/demo_shuffle/shuffle_paths.json")