from numpy import average
from common import *
from statistics import mean

MAX_HORIZON=200
OUTVERTEX=(-20,20)
PARKING=0
RETRIEVING=1

cdef class Vertex(object):
    cdef public int x,y
    cdef public list visiting_agents
    def __init__(self,x,y):
        self.x=x
        self.y=y    
        self.visiting_agents=[]
 


cdef class onlineMCP(object):
    cdef int xmax,ymax,clock
    cdef dict location_id,old_location_id,future_location_id
    cdef list unused_agents,used_agents,new_parking_agents,new_retrieving_agents,agents
    cdef double parking_probability,retrieving_probability
    cdef set occupied_port
    cdef dict retrieval_request
    cdef dict moved,vertices,time_counter
    cdef set v_obs
    cdef list retrieval_time_data, parking_time_data
    cdef list distance_data



    def __init__(self,int xmax,int ymax):
        self.xmax=xmax
        self.ymax=ymax
        self.location_id=dict()
        self.old_location_id=dict()
        self.future_location_id=dict()
        self.occupied_port=set()
        self.unused_agents=[]
        self.used_agents=[]
        self.retrieval_request=dict()
        # self.parking_agents=[]
        self.new_parking_agents=[]
        self.new_retrieving_agents=[]
        self.parked_agents=[]
        # self.retrieving_agents=[]
        self.parking_probability=0.6
        self.retrieving_probability=0.01
        self.agents=[]
        self.moved=dict()
        self.v_obs=set()
        self.vertices=dict()
        # self.init_system()
        # self.init_full_system()
        self.init_system_only_retrieval("./demo/demo_shuffle/shuffled_config.json")
        self.clock=0
        self.time_counter=dict()
        self.retrieval_time_data=[]
        self.parking_time_data=[]
        self.distance_data=[0 for a in self.agents]

    
    cdef check_all_retrieved(self):
        for agent in self.agents:
            if agent.loc!=OUTVERTEX:
                return False
        return True
    
    def init_full_system(self):
        for i,slot in enumerate(self.parking_spots):
            self.agents[i].loc=slot
            self.agents[i].path[-1]=slot
            self.agents[i].future_loc=slot
            self.parked_agents.append(self.agents[i].id)
            self.future_location_id[slot]=self.agents[i].id
            self.location_id[slot]=self.agents[i].id
            self.unused_agents.clear()
            
    


    cdef fill_paths(self,int makespan=-1):
        # makespan=0
        if makespan<0:
            for agent in self.agents:
                makespan=max(len(agent.path),makespan)
        for agent in self.agents:
            while len(agent.path)<makespan:
                agent.path.append(agent.path[-1]) 


    cdef get_next_v(self,int agent):
        if agent in self.moved:
            return self.agents[agent].path[-1]
        else:
            if len(self.agents[agent].plan)==0:
                return self.agents[agent].path[-1]
            return self.agents[agent].plan[0]
        


    
    cdef init_configuration(self,str config_file):
        f1=open("./demo/recording.json")
        data_dict=json.load(f1)
        self.io_ports=data_dict["io_ports"]
        for x in range(1,self.xmax-1):
            self.io_ports.append((x,self.ymax-1))
    
        #parking spots
        self.parking_spots=[]
        for x in range(1,self.xmax-1):
            for y in range(0,self.ymax-2):
                self.parking_spots.append((x,y))

        # unused agents
        self.agents=[Agent(OUTVERTEX,None,i) for i in range(len(self.parking_spots))]
        self.unused_agents=[agent.id for agent in self.agents]
        # vertices and visiting orders
        for x in range(self.xmax):
            for y in range(self.ymax):
                self.vertices[(x,y)]=Vertex(x,y)
        
        
    def generate_new_request_nonrandom(self):
        dataT=self.recording[str(self.clock)]
        for data in dataT["retrieval"]:
            port=(data[0],data[1])
            ak=self.agents[data[2]]
            ak.goal=port
            print("retrieving car",ak.id,"to port",port)
            self.new_retrieving_agents.append(ak)
            # self.parked_agents.pop(k)
            self.occupied_port.add(port)
            ak.state=RETRIEVING

        for data in dataT["parking"]:
            port=(data[0],data[1])
            ak=self.agents[data[2]]
            ak.goal=port
            ak.loc=port

            ak.future_loc=port
            ak.state=PARKING
            # ak.path.append(port)
            ak.path[-1]=port

            self.location_id[ak.loc]=ak.id
            self.future_location_id[ak.loc]=ak.id
            self.new_parking_agents.append(ak)
            # self.unused_agents.pop(ak.id)
            self.occupied_port.add(port)




    cdef generate_new_request(self):
        cdef list r_agents,available_ports
        cdef tuple port
        cdef double eta
        cdef int num_p,k

        r_agents=[]
        available_ports=[port for port in self.io_ports if port not in self.occupied_port]
        if len(available_ports)==0:
            print("No avaiable ports")
        for port in available_ports:
            eta=np.random.random()
            if eta<self.retrieving_probability and len(self.parked_agents)!=0:
                num_p=len(self.parked_agents)
                k=np.random.randint(num_p)
                ak=self.agents[self.parked_agents[k]]
                ak.goal=port
                r_agents.append(ak.id)
                
                print("retrieving car",ak.id,"to port",port)
                self.new_retrieving_agents.append(ak)
                self.parked_agents.pop(k)
                self.occupied_port.add(port)
                self.time_counter[ak.id]=self.clock
                ak.state=RETRIEVING
                # data["retrieval"].append((port[0],port[1],ak.id))

                #retrieve a car
            elif eta<self.retrieving_probability+self.parking_probability and len(self.unused_agents)!=0:
                num_p=len(self.unused_agents)
                k=np.random.randint(num_p)
                ak=self.agents[self.unused_agents[k]]
                assert(ak.loc==OUTVERTEX)
                # print("parking car",ak.id,"from port",port)
                ak.loc=port
                ak.goal=port
                ak.future_loc=port
                ak.state=PARKING
                ak.path[-1]=port
                self.location_id[ak.loc]=ak.id
                self.future_location_id[ak.loc]=ak.id
                self.time_counter[ak.id]=self.clock
                self.new_parking_agents.append(ak)
                self.unused_agents.pop(k)
                # data["parking"].append((port[0],port[1],ak.id))
                self.occupied_port.add(port)
        self.retrieval_request[self.clock]=r_agents
        # self.recording[self.clock]=data


    cdef sequential_planning(self):
        for agent in self.new_retrieving_agents:
            # print("retrieving",agent)
            self.RetrieveMp(agent.id)
        for agent in self.new_parking_agents:
            print("parking for agent",agent.id,agent.loc)
            self.ParkingMp(agent)
        self.new_parking_agents.clear()
        self.new_retrieving_agents.clear()
            # self.future_location_id
    
    cdef count_num_robots_at_row(self,int row):
        cdef int num,i
        num=0
        for i in range(0,self.xmax):
            if (i,row) in self.future_location_id:
                num=num+1
        return num

    # find an empty slot at the same cplumn
    cdef find_empty_slot_at_column(self,tuple current_pos):
        cdef int xc, yc,y
        xc,yc=current_pos
        for y in range(self.ymax-3,-1,-1):
            if (xc,y) not in self.future_location_id:
                assert((xc,y) not in self.future_location_id)
                print("I find esocrt",(xc,y))
                return (xc,y)
        return None

    # find the an empty slot for parking
    cdef find_empty_slot(self,tuple current_pos): 
        cdef int xc,yc
        cdef int y
        cdef list open
        cdef set visited
        cdef tuple curr,right,left
        xc,yc=current_pos
        for y in range(self.ymax-3,-1,-1):
            # if self.count_num_robots_at_row(y)>=self.xmax-2:
            #     continue
            open=[(xc,y)]
            visited=set()
            while len(open)!=0:
                curr=open.pop(0)
                visited.add(curr)
                if curr not in self.future_location_id:
                    # print(" I find escort",curr)
                    # self.used_empty_slots.add(curr)
                    return curr
                right=(curr[0]+1,curr[1])
                if right[0]<self.xmax-1 and right not in visited:
                    open.append(right)
                left=(curr[0]-1,curr[1])
                if left[0]>=1 and left not in visited:
                    open.append(left)
        return None

    #bring an escort to column 
    cdef bring_escort_to_column(self,tuple escort,int desired_column):
        cdef int xe,ye
        cdef int robot,dx
        xe,ye=escort
        if xe==desired_column:
            return
        if xe<desired_column:
            dx=1
        elif xe>desired_column:
            dx=-1
        curr=xe
        while curr!=desired_column:
            #parking first
            print((curr,ye),desired_column,"bring to escort")
            if (curr+dx,ye) not in self.future_location_id:
                print((curr+dx,ye), "not in dict")
                return 
            robot=self.future_location_id[(curr+dx,ye)]
            next_v=(curr,ye)
            self.forwardMP(robot,(curr+dx,ye),next_v)
            curr=curr+dx




    cdef bring_escort_to_port(self,tuple escort,tuple curr):
        cdef int xe,ye
        cdef int yc,ys,robot
        cdef tuple next_v
        xe,ye=escort
        yc=self.ymax-3
        ys=ye
        while ys<yc:
            ys=ys+1
            
            assert((xe,ys) in self.future_location_id)
            if (xe,ys) not in self.future_location_id:
                return False
            robot=self.future_location_id[(xe,ys)]
            # if robot in self.planned:
            #     return False
            next_v=(xe,ys-1)
            # if (self.clock+1) in self.v_table and next_v in self.v_table[self.clock+1]:
            #     return False
            self.forwardMP(robot,(xe,ys),next_v)
        return True

    cdef ParkingMp(self,agent):
        cdef tuple curr,escort
        cdef bint flag
        curr=agent.future_loc
        if curr[0]>0 and curr[0]<self.xmax-1 and curr[1]<self.ymax-2:
            return
        escort=self.find_empty_slot_at_column(curr)
        if escort is None:
            escort=self.find_empty_slot(curr)
            if escort is not None:
                print("the escort X is", escort,"for agent ",agent.id,"at",curr)
                self.bring_escort_to_column(escort,curr[0])
        escort=self.find_empty_slot_at_column(curr)
        if escort is not None:
            print("the escort is", escort,"for agent ",agent.id,"at",curr)
            flag=self.bring_escort_to_port(escort,curr)
            if flag==True:
                x,y=curr
                while y>self.ymax-3:
                    y=y-1
                    agent.plan.append((x,y))
                    self.vertices[(x,y)].visiting_agents.append(agent.id)
                    agent.future_loc=(x,y)
                self.future_location_id.pop(curr)
                self.future_location_id[agent.future_loc]=agent.id
            else:
                return
        else:
            print("cannot find escorts")

    cdef RetrieveMp(self,int agent):
        cdef tuple next_v
        cdef int aj,x,y,xg,yg,dx,next_a
        ai=self.agents[agent]
        while ai.future_loc!=ai.goal:
            if ai.future_loc[1]<self.ymax-2:
                next_v=(ai.future_loc[0],ai.future_loc[1]+1)
                if next_v not in self.future_location_id:
                    if ai.future_loc not in self.future_location_id:
                        print(ai.id,ai.future_loc,next_v)
                        assert(False)
                    self.forwardMP(ai.id,ai.future_loc,next_v)
                else:
                    aj=self.future_location_id[next_v]
                    self.YieldMp(aj)          
            else:
                x,y=ai.future_loc
                self.future_location_id.pop((x,y))
                xg,yg=ai.goal
                if x<xg:
                    dx=1
                else:
                    dx=-1
                while x!=xg:
                    x=x+dx
                    ai.plan.append((x,y))
                    self.vertices[(x,y)].visiting_agents.append(ai.id)
                ai.plan.append((xg,yg))
                self.vertices[(xg,yg)].visiting_agents.append(ai.id)
                ai.future_loc=ai.goal
        ai.future_loc=OUTVERTEX
        for y in range(self.ymax):
            if (0,y) in self.future_location_id:
                aj=self.future_location_id[(0,y)]
                currX=0
                while aj is not None:
                    next_v=(currX+1,y)
                    print("moving ",aj,"back",next_v)
                    if next_v in self.future_location_id:
                        next_a=self.future_location_id[next_v]
                    else:
                        next_a=-1
                    self.forwardMP(aj,(currX,y),next_v)
                    aj=next_a
                    currX=currX+1
                    # assert(currX<self.xmax)
            if (self.xmax-1,y) in self.future_location_id:
                aj=self.future_location_id[(self.xmax-1,y)]
                currX=self.xmax-1
                while aj>0:
                    print("moving ",aj,"back",next_v)
                    next_v=(currX-1,y)
                    if next_v in self.future_location_id:
                        next_a=self.future_location_id[next_v]
                    else:
                        next_a=-1
                    # assert(currX-1>=0)
                    self.forwardMP(aj,(currX,y),next_v)
                    aj=next_a
                    currX=currX-1
                # self.future_location_id[ai.future_loc]=ai.id

    cdef mcp_move(self,int agent):
        cdef tuple next_v,curr_v,vj
        cdef bint flag
        cdef int agentj
        # print("mcp moving agent",agent)
        if agent in self.moved:
            return self.moved[agent]
        if len(self.agents[agent].plan)==0:
            ai=self.agents[agent]
            if ai.state==RETRIEVING:
                assert(ai.loc==ai.goal)
                self.location_id.pop(ai.loc)
                ai.loc=OUTVERTEX
                ai.future_loc=OUTVERTEX
                ai.path.append(ai.loc)
                ai.state=None
                self.unused_agents.append(ai.id)
                self.occupied_port.remove(ai.goal)
                self.retrieval_time_data.append(self.clock-self.time_counter[ai.id])
                return True
                
            self.wait_agent(agent,ai.loc)
            return False
        ai=self.agents[agent]
        next_v=ai.plan[0]
        curr_v=ai.loc
        # print("next v is ",next_v)
        if agent==self.vertices[next_v].visiting_agents[0]:
            if next_v not in self.old_location_id:
                if  next_v not in self.v_obs:
                    # if curr_v not in self.old_location_id:
                    #     print(agent,curr_v)
                    # assert(curr_v in self.old_location_id)
                    self.forward_agent(agent,curr_v,next_v)
                    return True
                else:
                    self.wait_agent(agent,curr_v)
                    return False
            else:
                agentj=self.old_location_id[next_v]
                # aj=self.agents[agentj]
                # print(aj,agent,self.original_paths[aj])
                # if len(self.original_paths[aj])!=0:
                # print("debug",aj.id,agent,self.original_paths[aj],self.original_paths[agent])
                # vj=self.original_paths[aj][0]
                vj=self.get_next_v(agentj)
                flag=self.mcp_move(agentj)
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
            print(agent,self.agents[agent].plan,curr_v,next_v,self.vertices[next_v].visiting_agents)
            print("not the correct ordering ")
            self.wait_agent(agent,curr_v)
            return False  

    cdef forwardMP(self,int agent, tuple curr,tuple next):
        # update the order of visiting vertices
        self.vertices[next].visiting_agents.append(agent) 
        self.agents[agent].future_loc=next
        self.agents[agent].plan.append(next)
        if self.future_location_id[curr]==agent:
            self.future_location_id.pop(curr)
        # print(curr,"is popoed for agent",agent,"move to ",next)
        self.future_location_id[next]=agent


        
        
    
    cdef YieldMp(self,int agent,direction=None):
        cdef int fdx
        cdef str first_direc,second_direc
        cdef tuple next_v
        cdef int aj
        xa,ya=self.agents[agent].future_loc
        # print("yileding", agent,(xa,ya),direction)
        # assert((xa,ya) in self.location_id)
        if xa>self.xmax/2:
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
            if next_v not in self.future_location_id and (next_v[0]>=0 and next_v[0]<self.xmax):
                self.forwardMP(agent,(xa,ya),next_v)
                return True
            elif next_v in self.future_location_id:
                # assert(next_v[0]>=0 and next_v[0]<self.xmax)
                aj=self.future_location_id[next_v]
                if self.YieldMp(aj,first_direc)==True:
                    self.forwardMP(agent,(xa,ya),next_v)
                    return True
            # print("will this be called?")

        if (direction is None or direction==second_direc):
            next_v=(xa+sdx,ya)
            if next_v not in self.future_location_id and (next_v[0]>=0 and next_v[0]<self.xmax):
                self.forwardMP(agent,(xa,ya),next_v)
                return True
            elif next_v in self.future_location_id:
                aj=self.future_location_id[next_v]
                if self.YieldMp(aj,second_direc)==True:
                    self.forwardMP(agent,(xa,ya),next_v)
                    return True
        return False



    

    cdef forward_agent(self,int agent,tuple curr_v,tuple next_v):
        if curr_v in self.location_id and self.location_id[curr_v]==agent:
            self.location_id.pop(curr_v)
        self.location_id[next_v]=agent
        self.agents[agent].loc=next_v
        self.agents[agent].plan.pop(0)
        self.vertices[next_v].visiting_agents.pop(0)
        self.moved[agent]=True
        self.agents[agent].path.append(next_v)
        self.v_obs.add(next_v)
        self.distance_data[agent]=self.distance_data[agent]+1

    cdef wait_agent(self,int agent,tuple curr_v):
        self.moved[agent]=False
        self.agents[agent].path.append(curr_v)
        self.agents[agent].loc=curr_v
        # self.v_obs.add(curr_v)

    cdef mcp_execute(self):
        cdef int agent
        self.moved.clear()
        self.v_obs.clear()
        self.old_location_id=self.location_id.copy()
        for agent in range(len(self.agents)):
            self.mcp_move(agent)
            if self.agents[agent].state==PARKING:
                xc,yc=self.agents[agent].loc
                if yc<=self.ymax-3:
                    self.agents[agent].state=None
                    self.parking_time_data.append(self.clock-self.time_counter[agent])
                    self.occupied_port.remove(self.agents[agent].goal)
                    self.agents[agent].goal=None
                    self.parked_agents.append(agent)


    def sim(self):
        while True:
            self.clock=self.clock+1
            # print("debug",self.agents[2].loc,self.agents[2].plan,self.vertices[(2,3)].visiting_agents,"agent4",self.agents[4].loc,self.agents[4].plan,(2,3) in self.future_location_id)
            # print("*************************\n")
            if self.clock>MAX_HORIZON:
                break
            self.main_loop()
            # if self.check_all_retrieved():
            #     break
        print("total makespan=",self.clock)
        # with open("./demo/recording.json","w") as fp:
        #     json.dump(self.recording,fp)

    cdef main_loop(self):
        # self.generate_new_request()
        self.retrieve_car_ordered()
        
        self.sequential_planning()
        # print("BUGGGGG",self.vertices[(2,3)].visiting_agents)
        
        self.mcp_execute()
        
    def init_spots(self):
        #io ports
        self.io_ports=[]
        for x in range(1,self.xmax-1):
            self.io_ports.append((x,self.ymax-1))
    
        #parking spots
        self.parking_spots=[]
        for x in range(1,self.xmax-1):
            for y in range(0,self.ymax-2):
                self.parking_spots.append((x,y))
                
        #
        for x in range(self.xmax):
            for y in range(self.ymax):
                self.vertices[(x,y)]=Vertex(x,y)
                
    def get_avg_data(self):
        average_retrieval_time=mean(self.retrieval_time_data)
        # average_parking_time=mean(self.parking_time_data)
        average_number_moves=sum(self.distance_data)
        print("average retrieval time",average_retrieval_time)
        # print("agerage parking time",average_parking_time)
        print("average number of moves",average_number_moves)
        print("makespan",self.clock)
        

    def init_system_only_retrieval(self,file_name):
        self.init_spots()
        f1=open(file_name)
        data_dict=json.load(f1)
        agents_list=data_dict["agents"]
        for agent_dict in agents_list:
            start=tuple(agent_dict["loc"])
            id=agent_dict["id"]
            priority=agent_dict["priority"]
            ai=Agent(start,None,id)
            ai.future_loc=start
            self.future_location_id[start]=ai.id
            self.location_id[start]=ai.id
            self.agents.append(ai)
            self.parked_agents.append(ai.id)
        self.agents.sort(key=lambda a:a.id)
        self.parked_agents.sort()
        # np.random.shuffle(self.parked_agents)
        
    def retrieve_car_ordered(self):
        available_ports=[port for port in self.io_ports if port not in self.occupied_port]
        if len(available_ports)==0:
            print("No available ports")
        for port in available_ports:
            if len(self.parked_agents)==0:
                # print("all retrieved")
                break
            agent_k=self.parked_agents[-1]
            self.time_counter[agent_k]=self.clock
            ak=self.agents[agent_k]
            ak.goal=port
            self.new_retrieving_agents.append(ak)
            self.parked_agents.pop()
            self.occupied_port.add(port)
            ak.state=RETRIEVING
        #prioritization
        self.new_parking_agents.sort(key=lambda ak:manhattan_distance(ak.loc,ak.goal))
        
        

    def init_system(self):
        self.init_spots()

        # unused agents
        self.agents=[Agent(OUTVERTEX,None,i) for i in range(len(self.parking_spots))]
        self.unused_agents=[agent.id for agent in self.agents]
        # vertices and visiting orders


        data_dict=dict()
        file_name="./demo/system_info.json"
        data_dict["xmax"]=self.xmax
        data_dict["ymax"]=self.ymax
        data_dict["IOports"]=self.io_ports
        data_dict["ParkingSlots"]=self.parking_spots
        with open(file_name,"w") as fp:
            json.dump(data_dict,fp)


    def check_perpendicular(self,v1,v2,v3):
        x1,y1=v1
        x2,y2=v2
        x3,y3=v3
        if (x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)!=0:
            # print("perpendicular following conflicts")
            return True
        return False

    
    def get_paths(self):
        paths=[]
        for agent in self.agents:
            paths.append(agent.path)
        return paths
    
    def evaluateRT(self):
        paths=self.get_paths()
        makespan,makespanLB,sod,sor=evaluate_paths(paths)
        avt=0
        for id,re_time in self.retrieval_time.items():
            avt=avt+re_time
        print("makespan",makespan)
        print("sod",sod/len(paths))
        print("sor",sor/len(paths))
        print("avt",avt/len(paths))
        
    
    def save_paths_as_json(self,file_name):
        data_dict=dict()
        sol=[]
        for agent in self.agents:
            sol.append(agent.path)
        data_dict["paths"]=sol
        data_dict["retrieval_record"]=self.retrieval_request
        with open(file_name,"w") as fp:
            json.dump(data_dict,fp)
        
if __name__=="__main__":
    test_system=onlineMCP(22,22)
    test_system.sim()
    # test_system.get_avg_data()
    # test_system.evaluateRT()
    test_system.save_paths_as_json("./demo/online_mcp.json")
    pass