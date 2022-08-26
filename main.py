from common import *
from oneshot import OneShotInstance
from MPC import *
import time

def create_instance(xmax:int,ymax:int,num_parking:int,num_retrieving:int,num_other:int):
    problem= OneShotInstance()
    problem.xmax=xmax
    problem.ymax=ymax
    for x in range(1,xmax-1):
        for y in range(1,ymax-1):
            problem.all_slots.append((x,y))

    for x in range(1,xmax-1):
        problem.io_ports.append((x,ymax-1))
        
    io_ports_copy=problem.io_ports.copy()
    slots_copy=problem.all_slots.copy()
    np.random.shuffle(io_ports_copy)
    np.random.shuffle(slots_copy)
    k=0
    for i in range(num_retrieving):
        start_i=slots_copy.pop()
        goal_i=io_ports_copy.pop()
        ai=Agent(start_i,goal_i,k)
        problem.retrieval_agents.append(ai)
        k=k+1
    
    for i in range(num_parking):
        start_i=io_ports_copy.pop()
        ai=Agent(start_i,None,k)
        problem.parking_agents.append(ai)
        k=k+1
    for i in range(num_other):
        start_i=slots_copy.pop()
        ai=Agent(start_i,None,k)
        problem.already_parked_agents.append(ai)
        k=k+1
    return problem



def BCPR_solve(problem:OneShotInstance):
    problem=OneShotInstance("./demo/large_parking_retrieval.json")
    solver=BCPRSolver(problem)
    solver.solve()
    paths=[]
    for a in  solver.agents:
        paths.append(a.path)
    mpc=MPC(paths)
    mpc.mpcSolve()
    return mpc.plans
    

def create_full_density_instance(xmax:int,ymax:int):
    total_IO_ports=xmax-2;
    num_total_slots=(xmax-2)*(ymax-2)
    num_parking=total_IO_ports/2
    num_retrieving=total_IO_ports/2
    num_other=num_total_slots-num_parking-num_retrieving
    problem=create_instance(xmax,ymax,num_parking,num_retrieving,num_other)
    return problem
    




def exp_scalibility_grid_size():
    num_trials=10
    grid_size=[5,10,15,20,25,30,35,40,45,50]
    foldername="../data/BCPR/";
    makespan_data=[]
    sod_data=[]
    sor_data=[]
    time_data=[]
    for m in grid_size:
        total_makespan=0
        total_moves=0
        total_rp_time=0
        total_comp_time=0
        for k in range(num_trials):
            problem=create_full_density_instance(m,m)
            total_cars=len(problem.retrieval_agents)+len(problem.already_parked_agents)+len(problem.parking_agents)
            t0=time.time()
            paths=BCPR_solve(problem)
            t1=time.time()-t0
            total_comp_time=total_comp_time+t1
            makespan,makespanLB,sod,sor=evaluate_paths(paths,len(problem.retrieval_agents))
            total_makespan+=makespan
            total_moves=total_moves+sod/total_cars
            total_rp_time=total_rp_time+sor/len(problem.retrieval_agents)
        makespan_data.append(total_makespan/num_trials)
        sod_data.append(total_moves/num_trials)
        sor_data.append(total_rp_time/num_trials)
        time_data.append(total_comp_time/num_trials)
    save_data_as_csv(foldername+"makespan_data.csv",grid_size,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",grid_size,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",grid_size,sor_data);
    save_data_as_csv(foldername+"time_data.csv",grid_size,time_data);
            


if __name__=="__main__":
    exp_scalibility_grid_size()
    
    