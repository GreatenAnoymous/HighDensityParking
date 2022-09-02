
from common import *
from oneshot import OneShotInstance
# from oneshotMCP import oneshotMCP
from SINGLE import SingleSolver
from MCP import oneshotMCP
# from SINGLE import BCPRSolver
import time

def create_instance(xmax:int,ymax:int,num_parking:int,num_retrieving:int,num_other:int):
    problem= OneShotInstance()
    problem.xmax=xmax
    problem.ymax=ymax
    for x in range(1,xmax-1):
        for y in range(0,ymax-2):
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
    

    for i in range(num_other):
        start_i=slots_copy.pop()
        ai=Agent(start_i,None,k)
        problem.already_parked_agents.append(ai)
        k=k+1
    
    for i in range(num_parking):
        start_i=io_ports_copy.pop()
        ai=Agent(start_i,None,k)
        problem.parking_agents.append(ai)
        k=k+1
    return problem



def BCPR_solve(problem:OneShotInstance):
    # problem=OneShotInstance("./demo/large_parking_retrieval.json")
    # solver=BCPRSolver(problem)
    solver=oneshotMCP(problem)
    solver.sim()
 
    paths=solver.get_paths()
    return paths

def SINGLE_solve(problem:OneShotInstance):
    solver=SingleSolver(problem)
    solver.solve()
    paths=solver.getPaths()
    return paths
    

def create_full_density_instance(xmax:int,ymax:int):
    total_IO_ports=xmax-2
    num_total_slots=(xmax-2)*(ymax-2)
    num_parking=int(total_IO_ports/2)
    num_retrieving=int(total_IO_ports/2)
    num_other=num_total_slots-num_parking-num_retrieving
    problem=create_instance(xmax,ymax,num_parking,num_retrieving,num_other)
    return problem
    

def create_density_instance(num_robots,xmax,ymax):
    total_IO_ports=xmax-2
    num_total_slots=(xmax-2)*(ymax-2)
    num_parking=int(total_IO_ports/2)
    num_retrieving=int(total_IO_ports/2)
    num_other=num_robots-num_parking-num_retrieving
    problem=create_instance(xmax,ymax,num_parking,num_retrieving,num_other)
    return problem




def exp_scalibility_grid_size():
    num_trials=20
    grid_size=list(range(6,51,4))
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
        count=0
        for k in range(num_trials):
            
            problem=create_full_density_instance(m,m)
            # problem.save_as_json("./demo/example.json")
            # print("save as json")
            total_cars=len(problem.retrieval_agents)+len(problem.already_parked_agents)+len(problem.parking_agents)
            t0=time.time()
            paths=BCPR_solve(problem)
            t1=time.time()-t0
            total_comp_time=total_comp_time+t1
            makespan,makespanLB,sod,sor=evaluate_paths(paths,len(problem.retrieval_agents))
            print("makespan=",makespan)
            total_makespan+=makespan
            total_moves=total_moves+sod/(len(problem.retrieval_agents)+len(problem.parking_agents))
            count=count+1
            total_rp_time=total_rp_time+sor/len(problem.retrieval_agents)
            # exit(0)

        makespan_data.append(total_makespan/count)
        sod_data.append(total_moves/count)
        sor_data.append(total_rp_time/count)
        time_data.append(total_comp_time/count)
    save_data_as_csv(foldername+"makespan_data.csv",grid_size,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",grid_size,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",grid_size,sor_data);
    save_data_as_csv(foldername+"time_data.csv",grid_size,time_data);
        
        
def exp_scalibility_grid_size_single():
    num_trials=20
    grid_size=list(range(6,51,4))
    foldername="../data/SINGLE/";
    makespan_data=[]
    sod_data=[]
    sor_data=[]
    time_data=[]
    for m in grid_size:
        total_makespan=0
        total_moves=0
        total_rp_time=0
        total_comp_time=0
        count=0
        for k in range(num_trials):
            
            problem=create_full_density_instance(m,m)
            # problem.save_as_json("./demo/example.json")
            # print("save as json")
            total_cars=len(problem.retrieval_agents)+len(problem.already_parked_agents)+len(problem.parking_agents)
            t0=time.time()
            # paths=BCPR_solve(problem)
            paths=SINGLE_solve(problem)
            t1=time.time()-t0
            total_comp_time=total_comp_time+t1
            makespan,makespanLB,sod,sor=evaluate_paths(paths,len(problem.retrieval_agents))
            print("makespan=",makespan)
            total_makespan+=makespan
            total_moves=total_moves+sod/(len(problem.retrieval_agents)+len(problem.parking_agents))
            count=count+1
            total_rp_time=total_rp_time+sor/len(problem.retrieval_agents)
            # exit(0)

        makespan_data.append(total_makespan/count)
        sod_data.append(total_moves/count)
        sor_data.append(total_rp_time/count)
        time_data.append(total_comp_time/count)
    save_data_as_csv(foldername+"makespan_data.csv",grid_size,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",grid_size,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",grid_size,sor_data);
    save_data_as_csv(foldername+"time_data.csv",grid_size,time_data);        

def exp_scalibility_density():
    xmax=20
    ymax=20
    num_trials=20
    area=xmax*ymax
    num_robots=list(range(20,301,20))
    foldername="../data/BCPR/";
    makespan_data=[]
    sod_data=[]
    sor_data=[]
    time_data=[]
    for k in range(len(num_robots)):
        n=num_robots[k]
        num_robots[k]=num_robots[k]/area
        total_makespan=0
        total_moves=0
        total_rp_time=0
        total_comp_time=0
        count=0
        for k in range(num_trials):
            
            # problem=create_full_density_instance(m,m)
            problem=create_density_instance(n,xmax,ymax)
            # problem.save_as_json("./demo/example.json")
            # print("save as json")
            total_cars=len(problem.retrieval_agents)+len(problem.already_parked_agents)+len(problem.parking_agents)
            t0=time.time()
            paths=BCPR_solve(problem)
            t1=time.time()-t0
            total_comp_time=total_comp_time+t1
            makespan,makespanLB,sod,sor=evaluate_paths(paths,len(problem.retrieval_agents))
            print("makespan=",makespan)
            total_makespan+=makespan
            total_moves=total_moves+sod/(len(problem.retrieval_agents)+len(problem.parking_agents))
            count=count+1
            total_rp_time=total_rp_time+sor/len(problem.retrieval_agents)
            # exit(0)

        makespan_data.append(total_makespan/count)
        sod_data.append(total_moves/count)
        sor_data.append(total_rp_time/count)
        time_data.append(total_comp_time/count)
    save_data_as_csv(foldername+"makespan_data.csv",num_robots,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",num_robots,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",num_robots,sor_data);
    save_data_as_csv(foldername+"time_data.csv",num_robots,time_data);
    


def generate_demo():
    problem=OneShotInstance("./demo/large_parking_retrieval.json")
    paths=BCPR_solve(problem)
    data_dict=dict()
    data_dict["paths"]=paths
    with open("./demo/mcp.json","w") as fp:
        json.dump(data_dict,fp)


def exp_scalibility_density_single():
    xmax=20
    ymax=20
    num_trials=20
    area=xmax*ymax
    num_robots=list(range(20,301,20))
    foldername="../data/SINGLE/";
    makespan_data=[]
    sod_data=[]
    sor_data=[]
    time_data=[]
    for k in range(len(num_robots)):
        n=num_robots[k]
        num_robots[k]=num_robots[k]/area
        total_makespan=0
        total_moves=0
        total_rp_time=0
        total_comp_time=0
        count=0
        for k in range(num_trials):
            
            # problem=create_full_density_instance(m,m)
            problem=create_density_instance(n,xmax,ymax)
            # problem.save_as_json("./demo/example.json")
            # print("save as json")
            total_cars=len(problem.retrieval_agents)+len(problem.already_parked_agents)+len(problem.parking_agents)
            t0=time.time()
            paths=SINGLE_solve(problem)
            t1=time.time()-t0
            total_comp_time=total_comp_time+t1
            makespan,makespanLB,sod,sor=evaluate_paths(paths,len(problem.retrieval_agents))
            print("makespan=",makespan)
            total_makespan+=makespan
            total_moves=total_moves+sod/(len(problem.retrieval_agents)+len(problem.parking_agents))
            count=count+1
            total_rp_time=total_rp_time+sor/len(problem.retrieval_agents)
            # exit(0)

        makespan_data.append(total_makespan/count)
        sod_data.append(total_moves/count)
        sor_data.append(total_rp_time/count)
        time_data.append(total_comp_time/count)
    save_data_as_csv(foldername+"makespan_data.csv",num_robots,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",num_robots,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",num_robots,sor_data);
    save_data_as_csv(foldername+"time_data.csv",num_robots,time_data);
    


if __name__=="__main__":
    # exp_scalibility_density()
    # exp_scalibility_grid_size_single()
    # exp_scalibility_density_single()
    generate_demo()
    
    