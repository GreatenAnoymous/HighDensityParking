/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include"common.hpp"
#include"instance.hpp"
#include "ILP.hpp"
#include"ecbs.hpp"

void test_instance(){
    Instance problem("./python/demo.json");
    std::cout<<problem.xmax<<"  "<<problem.ymax<<std::endl;
    std::cout<<"num parking="<<problem.num_parking<<std::endl;
    std::cout<<"num retrieval="<<problem.num_retrieval<<std::endl;
    std::cout<<"slots size="<<problem.ParkingSlots.size()<<std::endl;
}

void test_ILP(){
    Instance problem("./python/small_parking_retrieval.json");
    std::cout<<problem.xmax<<"  "<<problem.ymax<<std::endl;
    std::cout<<"num parking="<<problem.num_parking<<std::endl;
    std::cout<<"num retrieval="<<problem.num_retrieval<<std::endl;
    std::cout<<"slots size="<<problem.ParkingSlots.size()<<std::endl;
    ILPSolver* solver=new ILPSolver();
    solver->solve(problem);
    save_result_as_json("./python/paths_small_ilp.json",solver->final_paths,0,true);
    delete solver;
}

void test_ECBS(){
    Instance problem("./python/large_parking_retrieval.json");
    std::cout<<problem.xmax<<"  "<<problem.ymax<<std::endl;
    std::cout<<"num parking="<<problem.num_parking<<std::endl;
    std::cout<<"num retrieval="<<problem.num_retrieval<<std::endl;
    std::cout<<"slots size="<<problem.ParkingSlots.size()<<std::endl;
    ECBS *solver=new ECBS(problem,1.5);
    solver->solve();
    save_result_as_json("./python/ecbs_paths.json",solver->solution,0,true);
    delete solver;
}

void test_generate_ILP(){
    int xmax=10;
    int ymax=10;
    int num_total_slots=(xmax-2)*(ymax-2);
    int num_paking=4;
    int num_retrieving=4;
    int num_other=num_total_slots-num_paking-num_retrieving;

    Instance problem=create_instance(xmax,ymax,num_paking,num_retrieving,num_other);
    std::cout<<"problem is created!"<<std::endl;
    std::cout<<"num parking="<<problem.num_parking<<"  "<<problem.other_starts.size()<<std::endl;
    std::cout<<"num retrieval="<<problem.num_retrieval<<"  "<<problem.retrieval_starts.size()<<std::endl;
    ILPSolver* solver=new ILPSolver();
    solver->solve(problem);
    save_result_as_json("./python/paths_large.json",solver->final_paths,0,true);
    delete solver;
}

Instance create_full_density_instance(int xmax,int ymax){
    int total_IO_ports=(xmax-2);
    int num_total_slots=(xmax-2)*(ymax-2);
    int num_paking=total_IO_ports/2;
    int num_retrieving=total_IO_ports/2;
    int num_other=num_total_slots-num_paking-num_retrieving;
    // std::cout<<num_paking<<"  "<<num_retrieving<<"  "<<num_other<<std::endl;
    Instance problem=create_instance(xmax,ymax,num_paking,num_retrieving,num_other);
    return problem;
}

Instance create_density_instance(int num_total_robots,int xmax=30,int ymax=30){
    int total_IO_ports=(xmax-2);
    int num_total_slots=(xmax-2)*(ymax-2);
    int num_paking=total_IO_ports/2;
    int num_retrieving=total_IO_ports/2;
    int num_other=num_total_robots-num_paking-num_retrieving;
    Instance problem=create_instance(xmax,ymax,num_paking,num_retrieving,num_other);
    return problem;
}


void exp_scalibility_grid_size(){
    int num_trials=20;
    std::vector<double> grid_size={6,8,10};
    
    std::string foldername="./data/ILP/";
    std::vector<double> makespan_data,sod_data,sor_data,time_data;
    for(auto m:grid_size){
        double total_makespan=0,total_moves=0,total_rp_time=0,total_comp_time=0;
        for(int k=0;k<num_trials;k++){
            int mi=(int)m;
            Instance problem=create_full_density_instance(mi,mi);
            std::cout<<"grid size="<<m<<std::endl;
            ILPSolver* solver=new ILPSolver();
            int total_cars=problem.retrieval_starts.size()+problem.other_starts.size();
            auto start_time = std::chrono::high_resolution_clock::now();
            solver->solve(problem);
            double time_cost = time_elapsed(start_time);
            double makespan,makespanLB,sod,sor;
            total_comp_time+=time_cost;
            evaluate_result2(solver->final_paths,makespan,makespanLB,sod,sor,problem.num_retrieval);
            total_makespan+=makespan;
            total_moves+=sod/(problem.num_retrieval*2);
            total_rp_time+=sor/problem.num_retrieval;
            delete solver;
        }
        makespan_data.push_back(total_makespan/num_trials);
        sod_data.push_back(total_moves/num_trials);
        sor_data.push_back(total_rp_time/num_trials);
        time_data.push_back(total_comp_time/num_trials);
    }
    save_data_as_csv(foldername+"makespan_data.csv",grid_size,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",grid_size,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",grid_size,sor_data);
    save_data_as_csv(foldername+"time_data.csv",grid_size,time_data);
}

void exp_scalibility_grid_size_ecbs(){
    int num_trials=10;
    std::vector<double> grid_size={6,8,10};
    
    std::string foldername="./data/ECBS/";
    std::vector<double> makespan_data,sod_data,sor_data,time_data;
    for(auto m:grid_size){
        double total_makespan=0,total_moves=0,total_rp_time=0,total_comp_time=0;
        for(int k=0;k<num_trials;k++){
            int mi=(int)m;
            Instance problem=create_full_density_instance(mi,mi);
            std::cout<<"grid size="<<m<<std::endl;
            ECBS* solver=new ECBS(problem);
            int total_cars=problem.retrieval_starts.size()+problem.other_starts.size();
            auto start_time = std::chrono::high_resolution_clock::now();
            solver->solve();
            double time_cost = time_elapsed(start_time);
            double makespan,makespanLB,sod,sor;
            if(solver->solution.size()==0){delete solver;continue;}
            total_comp_time+=time_cost;
            
            evaluate_result2(solver->solution,makespan,makespanLB,sod,sor,problem.num_retrieval);
            total_makespan+=makespan;
            total_moves+=sod/(problem.num_retrieval*2);
            total_rp_time+=sor/problem.num_retrieval;
            delete solver;
        }
        makespan_data.push_back(total_makespan/num_trials);
        sod_data.push_back(total_moves/num_trials);
        sor_data.push_back(total_rp_time/num_trials);
        time_data.push_back(total_comp_time/num_trials);
    }
    save_data_as_csv(foldername+"makespan_data.csv",grid_size,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",grid_size,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",grid_size,sor_data);
    save_data_as_csv(foldername+"time_data.csv",grid_size,time_data);
}


void exp_density_ecbs(){
    int num_trials=10;
    std::vector<double>num_robots={20,30,40,50,60,70,80,90};
    double area=20*20;
    std::string foldername="./data/ECBS_density/";
    std::vector<double> makespan_data,sod_data,sor_data,time_data;
    for(int i=0;i<num_robots.size();i++){
        int n=num_robots[i];
        num_robots[i]/=area;
        double total_makespan=0,total_moves=0,total_rp_time=0,total_comp_time=0;
        for(int k=0;k<num_trials;k++){
            
            Instance problem=create_density_instance(n,20,20);
            std::cout<<"num robots="<<n<<"  "<<problem.num_retrieval+problem.other_starts.size()<<std::endl;
            ECBS* solver=new ECBS(problem);
            int total_cars=problem.retrieval_starts.size()+problem.other_starts.size();
            auto start_time = std::chrono::high_resolution_clock::now();
            solver->solve();
            double time_cost = time_elapsed(start_time);
            double makespan,makespanLB,sod,sor;
            if(solver->solution.size()==0){
                delete solver;
                continue;
            }
            total_comp_time+=time_cost;
            evaluate_result2(solver->solution,makespan,makespanLB,sod,sor,problem.num_retrieval);
            total_makespan+=makespan;
            total_moves+=sod/(problem.num_retrieval*2);

            total_rp_time+=sor/problem.num_retrieval;
        }
        makespan_data.push_back(total_makespan/num_trials);
        sod_data.push_back(total_moves/num_trials);
        sor_data.push_back(total_rp_time/num_trials);
        time_data.push_back(total_comp_time/num_trials);
    }
    save_data_as_csv(foldername+"makespan_data.csv",num_robots,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",num_robots,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",num_robots,sor_data);
    save_data_as_csv(foldername+"time_data.csv",num_robots,time_data);
}



void exp_density(){
    int num_trials=5;
    std::vector<double>num_robots={20,30,40,50,60,70,80,90};
    double area=20*20;
    std::string foldername="./data/ILP_density/";
    std::vector<double> makespan_data,sod_data,sor_data,time_data;
    for(int i=0;i<num_robots.size();i++){
        int n=num_robots[i];
        num_robots[i]/=area;
        double total_makespan=0,total_moves=0,total_rp_time=0,total_comp_time=0;
        for(int k=0;k<num_trials;k++){
            
            Instance problem=create_density_instance(n,20,20);
            std::cout<<"num robots="<<n<<"  "<<problem.num_retrieval+problem.other_starts.size()<<std::endl;
            ILPSolver* solver=new ILPSolver();
            int total_cars=problem.retrieval_starts.size()+problem.other_starts.size();
            auto start_time = std::chrono::high_resolution_clock::now();
            solver->solve(problem);
            double time_cost = time_elapsed(start_time);
            double makespan,makespanLB,sod,sor;
            total_comp_time+=time_cost;
            evaluate_result2(solver->final_paths,makespan,makespanLB,sod,sor,problem.num_retrieval);
            total_makespan+=makespan;
            total_moves+=sod/(problem.num_retrieval*2);

            total_rp_time+=sor/problem.num_retrieval;
        }
        makespan_data.push_back(total_makespan/num_trials);
        sod_data.push_back(total_moves/num_trials);
        sor_data.push_back(total_rp_time/num_trials);
        time_data.push_back(total_comp_time/num_trials);
    }
    save_data_as_csv(foldername+"makespan_data.csv",num_robots,makespan_data);
    save_data_as_csv(foldername+"sod_data.csv",num_robots,sod_data);
    save_data_as_csv(foldername+"sor_data.csv",num_robots,sor_data);
    save_data_as_csv(foldername+"time_data.csv",num_robots,time_data);
}


int main(){
    test_ILP();
    // exp_scalibility_grid_size();
    // exp_density();
    // test_ECBS();
    // exp_scalibility_grid_size_ecbs();
    // exp_density_ecbs();
}