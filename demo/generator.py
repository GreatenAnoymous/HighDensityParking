import numpy as np
import json

def generate_instance():
    file_name="./demo.json"
    data_dict=dict()
    xmax=29
    ymax=14
    data_dict["xmax"]=29
    data_dict["ymax"]=14
    io_ports=[(x,0) for x in range(0,xmax,3)]+[(x,ymax-1) for x in range(0,xmax,3)]
    parking_slots=[(x,y) for x in range(1,xmax-1) for y in range(1,ymax-1)]
    num_retrieval=5
    num_parking=5
    num_already_parking=100
    
    retrieval_goals=io_ports.copy()
    np.random.shuffle(retrieval_goals)
    retrieval_goals=retrieval_goals[0:num_retrieval]
    
    slots_copy=parking_slots.copy()
    np.random.shuffle(slots_copy)
    
    retrieval_starts=slots_copy[0:num_retrieval]
    other_starts=io_ports.copy()
    np.random.shuffle(other_starts)
    other_starts=other_starts[0:num_parking]
    other_starts.extend(slots_copy[num_retrieval:num_retrieval+num_already_parking])
    
    
    data_dict["num_retrieval"]=num_retrieval
    data_dict["num_parking"]=num_parking+num_already_parking
    data_dict["IOports"]=io_ports
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["ParkingSlots"]=parking_slots
    data_dict["other_starts"]=other_starts
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)
    
    
    


def generate_instance_well_formed():
    data_dict=dict()
    

def generate_debug_demo():
    file_name="./demo.json"
    data_dict=dict()
    data_dict["xmax"]=2
    data_dict["ymax"]=2
    data_dict["num_retrieval"]=2
    data_dict["num_parking"]=0
    data_dict["IOports"]=[]
    data_dict["retrieval_starts"]=[(0,0),(1,0)]
    data_dict["retrieval_goals"]=[(0,1),(0,0)]
    data_dict["ParkingSlots"]=[(1,0)]
    data_dict["other_starts"]=[]
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)


def high_density_demo():
    file_name="./demo_retrieve_one_car.json"
    data_dict=dict()
    data_dict["xmax"]=10
    data_dict["ymax"]=8
    data_dict["num_retrieval"]=0
    data_dict["num_parking"]=1
    data_dict["IOports"]=[(x,7) for x in range(2,8)]

    data_dict["ParkingSlots"]=[(x,y) for x in range(1,9) for y in range(0,6)]
    data_dict["WellformedSlots"]=[(x,y) for x in range(1,9) for y in range(0,6) if x %3!=0]
    retrieval_starts,retrieval_goals,parking_starts,other_starts=generate_random_instance(data_dict["ParkingSlots"],data_dict["IOports"],data_dict["num_retrieval"],data_dict["num_parking"],len(data_dict["ParkingSlots"])-1)
    print(len(retrieval_starts),len(retrieval_goals),len(other_starts))
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["parking_starts"]=parking_starts
    data_dict["other_starts"]=other_starts
    # with open(file_name,"w") as fp:
    #     json.dump(data_dict,fp)

    # pathfile_name="./demo_retrieve_one_car_path.json"
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)

def generate_random_instance(parking_slots,io_ports,num_retrieval,num_parking,num_already_parked):
    """_summary_

    Args:
        parking_slots (_type_): _description_
        io_ports (_type_): _description_
        num_retrieval (_type_): _description_
        num_parking (_type_): _description_
        num_already_parked (_type_): _description_

    Returns:
        _type_: _description_
    """
    retrieval_goals=io_ports.copy()
    np.random.shuffle(retrieval_goals)
    retrieval_goals=retrieval_goals[0:num_retrieval]
    
    slots_copy=parking_slots.copy()
    np.random.shuffle(slots_copy)
    
    retrieval_starts=slots_copy[0:num_retrieval]
    parking_starts=io_ports.copy()
    np.random.shuffle(parking_starts)
    parking_starts=parking_starts[0:num_parking]
    
    # print(num_retrieval,num_already_parked)
    other_starts=slots_copy[num_retrieval:num_retrieval+num_already_parked]

    return retrieval_starts,retrieval_goals,parking_starts,other_starts



def wellformed_simple_demo():
    file_name="./demo_well_formed.json"
    data_dict=dict()
    data_dict["xmax"]=10
    data_dict["ymax"]=8
    data_dict["num_retrieval"]=2
    data_dict["num_parking"]=3
    data_dict["IOports"]=[(x,7) for x in range(2,8)]

    data_dict["ParkingSlots"]=[(x,y) for x in range(1,9) for y in range(0,6)]
    data_dict["WellformedSlots"]=[(x,y) for x in range(1,9) for y in range(0,6) if x %3!=0]
    
    retrieval_starts,retrieval_goals,parking_starts,other_starts=generate_random_instance(data_dict["WellformedSlots"],data_dict["IOports"],data_dict["num_retrieval"],data_dict["num_parking"],20)
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["parking_starts"]=parking_starts
    data_dict["other_starts"]=other_starts
    print(len(data_dict["ParkingSlots"]),len(data_dict["WellformedSlots"]))
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)


def full_only_retrieval():
    file_name="./demo_only_retrieval.json"
    data_dict=dict()
    data_dict["xmax"]=10
    data_dict["ymax"]=8
    data_dict["num_retrieval"]=5
    data_dict["num_parking"]=0
    data_dict["IOports"]=[(x,7) for x in range(2,8)]

    data_dict["ParkingSlots"]=[(x,y) for x in range(1,9) for y in range(0,6)]
    data_dict["WellformedSlots"]=[(x,y) for x in range(1,9) for y in range(0,6) if x %3!=0]
    retrieval_starts,retrieval_goals,parking_starts,other_starts=generate_random_instance(data_dict["ParkingSlots"],data_dict["IOports"],data_dict["num_retrieval"],data_dict["num_parking"],len(data_dict["ParkingSlots"]))
    print(len(retrieval_starts),len(retrieval_goals),len(other_starts))
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["parking_starts"]=[]
    data_dict["other_starts"]=other_starts
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)


def full_only_retrieval_one():
    file_name="./demo_full.json"
    data_dict=dict()
    data_dict["xmax"]=10
    data_dict["ymax"]=8
    data_dict["num_retrieval"]=1
    io_port=(3,7)
    data_dict["IOports"]=[io_port]

    data_dict["ParkingSlots"]=[(x,y) for x in range(0,10) for y in range(0,8)]
    data_dict["ParkingSlots"].remove(io_port)
    data_dict["num_parking"]=len(data_dict["ParkingSlots"])-2
    data_dict["WellformedSlots"]=[(x,y) for x in range(1,9) for y in range(0,6) if x %3!=0]
    retrieval_starts,retrieval_goals,parking_starts,other_starts=generate_random_instance(data_dict["ParkingSlots"],data_dict["IOports"],data_dict["num_retrieval"],data_dict["num_parking"],len(data_dict["ParkingSlots"]))
    data_dict["num_parking"]=len(data_dict["ParkingSlots"])
    print(len(retrieval_starts),len(retrieval_goals),len(other_starts),len(data_dict["ParkingSlots"]))
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["parking_starts"]=[]
    data_dict["other_starts"]=other_starts
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)

def parking_test():
    file_name="./demo_parking.json"
    data_dict=dict()
    data_dict["xmax"]=10
    data_dict["ymax"]=8
    data_dict["num_retrieval"]=0
    data_dict["num_parking"]=5
    data_dict["IOports"]=[(x,7) for x in range(2,8)]

    data_dict["ParkingSlots"]=[(x,y) for x in range(1,9) for y in range(0,6)]
    data_dict["WellformedSlots"]=[(x,y) for x in range(1,9) for y in range(0,6) if x %3!=0]
    retrieval_starts,retrieval_goals,parking_starts,other_starts=generate_random_instance(data_dict["ParkingSlots"],data_dict["IOports"],data_dict["num_retrieval"],data_dict["num_parking"],int(len(data_dict["ParkingSlots"])-data_dict["num_parking"]))
    print(len(retrieval_starts),len(retrieval_goals),len(other_starts))
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["parking_starts"]=parking_starts
    data_dict["other_starts"]=other_starts
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)

def parking_and_retrieval():
    file_name="./demo_parking_retrieval.json"
    data_dict=dict()
    data_dict["xmax"]=10
    data_dict["ymax"]=8
    data_dict["num_retrieval"]=3
    data_dict["num_parking"]=2
    data_dict["IOports"]=[(x,7) for x in range(2,8)]

    data_dict["ParkingSlots"]=[(x,y) for x in range(1,9) for y in range(0,6)]
    data_dict["WellformedSlots"]=[(x,y) for x in range(1,9) for y in range(0,6) if x %3!=0]
    retrieval_starts,retrieval_goals,parking_starts,other_starts=generate_random_instance(data_dict["ParkingSlots"],data_dict["IOports"],data_dict["num_retrieval"],data_dict["num_parking"],int(len(data_dict["ParkingSlots"])-4*data_dict["num_parking"]))
    print(len(retrieval_starts),len(retrieval_goals),len(other_starts))
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["parking_starts"]=parking_starts
    data_dict["other_starts"]=other_starts
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)

def large_parking_and_retrieval():
    file_name="./large_parking_retrieval.json"
    data_dict=dict()
    xmax=22
    ymax=22
    num_retrieval=10
    num_parking=10
    data_dict["xmax"]=xmax
    data_dict["ymax"]=ymax
    data_dict["num_retrieval"]=num_retrieval
    data_dict["num_parking"]=num_parking
    data_dict["IOports"]=[(x,ymax-1) for x in range(1,xmax-1)]
    num_other=(xmax-2)*(ymax-2)-num_parking-num_retrieval
    data_dict["ParkingSlots"]=[(x,y) for x in range(1,xmax-1) for y in range(0,ymax-2)]
    # data_dict["WellformedSlots"]=[(x,y) for x in range(1,9) for y in range(0,6) if x %3!=0]
    retrieval_starts,retrieval_goals,parking_starts,other_starts=generate_random_instance(data_dict["ParkingSlots"],data_dict["IOports"],data_dict["num_retrieval"],data_dict["num_parking"],num_other)
    print(len(retrieval_starts),len(retrieval_goals),len(other_starts))
    data_dict["retrieval_starts"]=retrieval_starts
    data_dict["retrieval_goals"]=retrieval_goals
    data_dict["parking_starts"]=parking_starts
    data_dict["other_starts"]=other_starts
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)



   

if __name__=="__main__":
    #generate_instance()
    # high_density_demo()
    # wellformed_simple_demo()
    # full_only_retrieval()
    # full_only_retrieval_one()
    # generate_debug_demo()
    # parking_test()
    # parking_and_retrieval()
    large_parking_and_retrieval()
    # high_density_demo()