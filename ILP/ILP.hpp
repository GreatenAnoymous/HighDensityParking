/**
 * @file ILP.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "common.hpp"
#include"gurobi_c++.h"
#include "instance.hpp"

static GRBEnv genv;

class ILPSolver{
public:
    ILPSolver();
    GRBEnv *envp=&genv;
    using id_type=std::string;
    void solve(Instance &instance);
    GRBModel prepare_model(size_t time_steps,Instance &instance); 
    void retrieve_paths(GRBModel &model,size_t time_steps,Instance &instance);

    inline id_type get_id(size_t r, Location* v1, Location* v2, size_t t);
    inline void store_var_for_vertices(GRBVar &var, Location* v1, Location* v2, size_t t1, size_t t2);
    inline void store_var_for_robots(GRBVar &var, size_t r, Location* v1, Location* v2, size_t t1, size_t t2);
    inline void store_var_for_edges(GRBVar &var, Location* v1, Location* v2, size_t t);
    inline void store_var_for_horizontal_edges(GRBVar &var,Location *v1,size_t t);
    inline void store_var_for_vertical_edges(GRBVar &var,Location *v1,size_t t);

    std::map<id_type, GRBVar> edge_var_map = std::map<id_type, GRBVar>();
    std::map<id_type, std::vector<GRBVar>> edge_time_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> time_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> time_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> robot_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> robot_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> edge_horizontal=std::map<id_type, std::vector<GRBVar>>();
    std::map<id_type, std::vector<GRBVar>> edge_vertical=std::map<id_type, std::vector<GRBVar>>();
    std::vector<std::set<size_t>> reachability = std::vector<std::set<size_t>>();
    std::vector<std::vector<size_t>> individual_paths = std::vector<std::vector<size_t>>();
    std::vector<std::vector<Location*>> final_paths;


    std::string check_edge_type(Location * ,Location *);



};


