from oneshot import OneShotInstance
from common import *


MAX_HORIZON=10000
OUTVERTEX=(-10,-10)
PARKING=0
RETRIEVING=1


cdef class SingleSolver(object):
    cdef list agents
    cdef int clock
    cdef dict location_id
    cdef list parking_agents
    cdef list retrieving_agents
    cdef dict moved
    cdef int xmax,ymax

    def __init__(self,instance) -> None:
        self.xmax=instance.xmax
        self.ymax=instance.ymax
        self.agents=[]
        self.clock=1
        self.location_id=dict()
        # self.old_location_id=dict()
        self.parking_agents=[]
        self.retrieving_agents=[]
        # self.entangled_agents=set()
        # self.outOfBoundAgents=set()
        self.moved=dict()

        instance.retrieval_agents.sort(key=lambda agent:-agent.loc[1])

        for agent in instance.retrieval_agents:
            self.location_id[agent.loc]=agent.id
            self.agents.append(agent)
            self.retrieving_agents.append(agent)
        # assert(len(self.instance.parking_agents)==0)
        for agent in instance.already_parked_agents:
            self.location_id[agent.loc]=agent.id
            self.agents.append(agent)

        for agent in instance.parking_agents:
            self.location_id[agent.loc]=agent.id
            self.agents.append(agent)
            self.parking_agents.append(agent)
        for agent in self.agents:
            agent.path.append(agent.loc)
        self.agents.sort(key=lambda a:a.id)
        # print("cars need to be parked num=",len(self.instance.parking_agents))

    cdef fill_paths(self,int makespan=-1):
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
        for agent in self.parking_agents:
            self.ParkingMp(agent)
        for agent in self.retrieving_agents:
            self.RetrieveMp(agent.id)
        
        print("solved")
        self.save_as_json("./demo/single.json")

    def getPaths(self):
        paths=[]
        for agent in self.agents:
            paths.append(agent.path)
        return paths

    cdef count_num_robots_at_row(self,int row):
        cdef int num
        num=0
        for i in range(0,self.xmax):
            if (i,row) in self.location_id:
                num=num+1
        return num

     # find an empty slot at the same cplumn
    cdef find_empty_slot_at_column(self,tuple current_pos):
        cdef int xc,yc
        xc,yc=current_pos
        for y in range(self.ymax-3,-1,-1):
            if (xc,y) not in self.location_id:
                return (xc,y)
        return None

    # find the an empty slot for parking
    cdef find_empty_slot(self,tuple current_pos):
        cdef list open
        cdef set visited
        cdef tuple curr,right,left
        xc,yc=current_pos
        for y in range(self.ymax-3,-1,-1):
            if self.count_num_robots_at_row(y)>=self.xmax-2:
                continue
            open=[(xc,y)]
            visited=set()
            while len(open)!=0:
                curr=open.pop(0)
                visited.add(curr)
                if curr not in self.location_id:
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
        cdef int curr,robot
        cdef tuple next_v
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
            # print((curr,ye),desired_column,"bring to escort")
            # print((curr,xe))
            if (curr+dx,ye) not in self.location_id:
                # print((curr+dx,ye), "not in dict")
                return 
            robot=self.location_id[(curr+dx,ye)]
            # if robot in self.planned:
            #     print(robot,"already planned line 162")
            #     return 
            next_v=(curr,ye)
            # if self.clock+1 in self.v_table and next_v in self.v_table[self.clock+1]:
            #     print("collisions",(curr+dx,ye),"to",next_v,"already occupied")
            #     return
            self.forward(robot,(curr+dx,ye),next_v)
            curr=curr+dx
            # print("curr=",curr)
            # self.plans[robot]=[()]
            # self.planned.add(robot)



    cdef bring_escort_to_port(self,tuple escort,tuple curr):
        cdef int xe,ye,yc,ys,robot
        cdef tuple next_v
        xe,ye=escort
        yc=self.ymax-3
        assert(xe==curr[0])
        ys=ye
        while ys<yc:
            ys=ys+1
            # if (xe,ys) not in self.location_id:
            #     print(escort,(xe,ys),(xe,ys) in self.used_empty_slots)
            # assert((xe,ys) in self.location_id)
            if (xe,ys) not in self.location_id:
                return False
            robot=self.location_id[(xe,ys)]
            # if robot in self.planned:
            #     return False
            next_v=(xe,ys-1)
            # if (self.clock+1) in self.v_table and next_v in self.v_table[self.clock+1]:
            #     return False
            self.forward(robot,(xe,ys),next_v)
        return True

    cdef ParkingMp(self,agent):
        cdef tuple curr,escort
        cdef bint flag
        cdef int x,y
        curr=agent.loc
        if curr[1]<self.ymax-2:
            return
        escort=self.find_empty_slot_at_column(curr)
        if escort is None:
            escort=self.find_empty_slot(curr)
            if escort is not None:
                # print("the escort X is", escort,"for agent ",agent.id,"at",curr)
                self.bring_escort_to_column(escort,curr[0])
                
                self.clock=self.clock+1
                self.fill_paths(self.clock)
        escort=self.find_empty_slot_at_column(curr)
        if escort is not None:
            # print("the escort is", escort,"for agent ",agent.id,"at",curr)
            flag=self.bring_escort_to_port(escort,curr)
            if flag==True:
                x,y=curr
                while y>self.ymax-3:
                    y=y-1
                    self.clock=self.clock+1
                    agent.path.append((x,y))
                self.location_id.pop(curr)
                agent.loc=agent.path[-1]
                self.location_id[agent.loc]=agent.id
                self.fill_paths(self.clock)
                # self.plans[agent]=path[1:]
                # self.reserve_path(path)
            else:
                return
            #plan using timespace A *
        # elif escort is None:
        #     escort=self.find_empty_slot(curr)
        #     if escort is not None:
        #         print("the escort X is", escort,"for agent ",agent,"at",curr)
        #         self.bring_escort_to_column(escort,curr[0])
       

    # def MoveBack(self,agent:int,reserved:set,dx=None):
    #     xa,ya=self.agents[agent].loc
    #     if dx is None:
    #         if xa==0:
    #             dx=1
    #         elif xa==self.xmax-1:
    #             dx=-1
    #         else:
    #             self.outOfBoundAgents.remove(agent)
    #             return False
    #     # print("debug",(xa,ya),dx,agent in self.outOfBoundAgents,"agent=",agent)
    #     next_v=(xa+dx,ya)
    #     # print("?????")
    #     if next_v not in self.location_id and next_v not in reserved:
    #         self.forward(agent,(xa,ya),next_v)
    #     elif next_v in self.location_id:
    #         aj=self.location_id[next_v]
    #         self.MoveBack(aj,reserved,dx)

        
        

    cdef RetrieveMp(self,int agent):
        cdef set reserved
        cdef tuple next_v,next_next_v
        cdef int aj,ajj,x,y,xg,yg
        ai=self.agents[agent]
        
        reserved=set()
        while ai.loc!=ai.goal:
            
            if self.clock>MAX_HORIZON:
                break
            self.moved.clear()
            reserved.clear()
            if ai.loc[1]<self.ymax-2:
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
                if next_next_v[1]<self.ymax-1 and next_next_v in self.location_id:
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
                # if self.agents[a].loc[0]>0 and self.agents[a].loc[0]<self.xmax-1:
                #     self.outOfBoundAgents.remove(a)
                
            self.fill_paths(self.clock)
            # print("timestep=",self.clock)

      

    cdef forward(self,int agent,tuple curr,tuple next):
        self.agents[agent].loc=next
        self.agents[agent].path.append(next)
        self.location_id.pop(curr)
        # print(curr,"is popoed for agent",agent,"move to ",next)
        self.location_id[next]=agent
        self.moved[agent]=True
        # if next[0]==0 or next[0]==self.xmax-1:
        #     print("agent out of bounds, agent=",agent ,next)
        #     self.outOfBoundAgents.add(agent)

        
        
    
    cdef YieldMp(self,int agent,set reserved,direction=None):
        cdef int fdx,sdx,aj
        cdef tuple next_v
        if agent in self.moved:
            return self.moved[agent]
        xa,ya=self.agents[agent].loc
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
            if next_v not in self.location_id and (next_v[0]>=0 and next_v[0]<self.xmax):
                self.forward(agent,(xa,ya),next_v)
                return True
            elif next_v in self.location_id:
                assert(next_v[0]>=0 and next_v[0]<self.xmax)
                aj=self.location_id[next_v]
                if self.YieldMp(aj,reserved,first_direc)==True:
                    self.forward(agent,(xa,ya),next_v)
                    return True
            # print("will this be called?")
        if (direction is None or direction==second_direc):
            next_v=(xa+sdx,ya)
            if next_v not in self.location_id and (next_v[0]>=0 and next_v[0]<self.xmax):
                self.forward(agent,(xa,ya),next_v)
                return True
            elif next_v in self.location_id:
                aj=self.location_id[next_v]
                if self.YieldMp(aj,reserved,second_direc)==True:
                    self.forward(agent,(xa,ya),next_v)
                    return True

        self.moved[agent]=False
        return False

# if __name__=="__main__":
#     problem=OneShotInstance("./demo/example.json")
#     # test_solver=oneshotMCP(problem)
#     # test_solver=BCPRSolver(problem)
#     test_solver=SingleSolver(problem)