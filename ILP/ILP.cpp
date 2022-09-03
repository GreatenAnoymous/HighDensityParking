#include "ILP.hpp"

#define MIN_TOAL_COSTS 1

ILPSolver::ILPSolver()
{
}



// void ILPSolver::solve(OneShotTask &task, Graph &graph, size_t split)
// {
//     if (split == 0)
//         solve_original(task, graph);
//     else
//         solve_split();
// }




GRBModel ILPSolver::prepare_model(size_t time_steps,Instance &instance)
{
    bool relaxed = false;
    // auto start_time = std::chrono::high_resolution_clock::now();
    edge_var_map = std::map<id_type, GRBVar>();
    edge_time_vector_map = std::map<id_type, std::vector<GRBVar>>();
    time_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    time_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    robot_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    robot_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    edge_vertical.clear();
    edge_horizontal.clear();
    GRBModel model = GRBModel(*envp);
    model.set(GRB_IntParam_OutputFlag, 0);
    // Set variables
    std::set<Location*> reachable_vertices;
    std::set<Location*> new_vertices;
    std::vector<GRBVar> obj_variables;
    std::vector<GRBVar> feedback_variables;
    for (size_t r = 0; r <= instance.num_retrieval; r++)
    {
        reachable_vertices = std::set<Location *>();
        if(r==instance.num_retrieval){
            for(auto &start:instance.other_starts){
                id_type id = get_id(r, nullptr, start, - 1);
                GRBVar x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
                store_var_for_robots(x, r, nullptr, start, - 1, 0);
                reachable_vertices.insert(start);
            }
        }else{
            reachable_vertices.insert(instance.retrieval_starts[r]);
        }
        
        for (size_t t = 1; t < time_steps + 1; t++)
        {
            new_vertices = std::set<Location*>();
            for (auto n : reachable_vertices)
            {
                auto nbrs=instance.grids->getNeighbors(n);
                nbrs.push_back(n);
                for (auto nbr : nbrs)
                {
                    // Check goal reachability
                    // if (get_manhattan_distance(nbr, task.goals[r]) > time_steps - t)
                    //     continue;
                    if(r<instance.num_retrieval){
                        if(nbr->manhattan_dist(instance.retrieval_goals[r])>time_steps-t) continue;
                    }
                    new_vertices.insert(nbr);
                    id_type id = get_id(r, n, nbr, t - 1);
                    GRBVar x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
                    if(nbr!=n)
                        obj_variables.push_back(x);
                    edge_var_map.insert(std::make_pair(id, x));
                    // printf("(%d,%d,%d,%d)\n",n->id,nbr->id,t-1,t);
                    store_var_for_vertices(x, n, nbr, t - 1, t);
                    store_var_for_robots(x, r, n, nbr, t - 1, t);
                    store_var_for_edges(x, n, nbr, t - 1);
                    if(check_edge_type(n,nbr)=="horizontal"){
                        // std::cout<<"horizontal"<<std::endl;
                        store_var_for_horizontal_edges(x,n,t-1);
                        store_var_for_horizontal_edges(x,nbr,t-1);
                    }
                    else if(check_edge_type(n,nbr)=="vertical"){
                        store_var_for_vertical_edges(x,n,t-1);
                        store_var_for_vertical_edges(x,nbr,t-1);
                        // std::cout<<"vertical"<<std::endl;
                    }
                }
                // Check goal reachability
                // if (get_manhattan_distance(n, task.goals[r]) > time_steps - t)
                //     continue;
                // new_vertices.insert(n);
                // id_type id = get_id(r, n, n, t - 1);
                // GRBVar x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
                // edge_var_map.insert(std::make_pair(id, x));
                // store_var_for_vertices(x, n, n, t - 1, t);
                // store_var_for_robots(x, r, n, n, t - 1, t);
            }
            reachable_vertices = new_vertices;
        }
    }
    // Set objective
    GRBLinExpr obj_expr = GRBLinExpr();
    GRBLinExpr feedback_expr=GRBLinExpr();
    //cars  should be retrieved
    for (size_t r = 0; r < instance.num_retrieval; r++)
    {
        id_type id = get_id(r, instance.retrieval_goals[r], instance.retrieval_starts[r], time_steps);
        GRBVar x;
        x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
        edge_var_map.insert(std::make_pair(id, x));
        store_var_for_vertices(x, instance.retrieval_goals[r], instance.retrieval_starts[r], time_steps, 0);
        store_var_for_robots(x, r, instance.retrieval_goals[r], instance.retrieval_starts[r], time_steps, 0);
        // obj_expr += x;
        feedback_expr+=x;
        //model.addConstr(x, GRB_EQUAL, 1);
    }
  

    // for(auto &[id,s]:edge_var_map){
    //     std::cout<<"debug"<<id<<std::endl;
    // }

    // cars should be parked at the parking slots
    for(auto &p :instance.ParkingSlots){
        
        id_type id = get_id(instance.num_retrieval,p,nullptr, time_steps);
        GRBVar x;
        x=model.addVar(0.0,1.0,0.0,relaxed? GRB_CONTINUOUS:GRB_BINARY,id);
        edge_var_map.insert(std::make_pair(id,x));
        store_var_for_vertices(x,p,nullptr,time_steps,0);
        store_var_for_robots(x,instance.num_retrieval,p,nullptr,time_steps,0);
        feedback_expr+=x;
    }

    model.addConstr(feedback_expr,GRB_EQUAL,instance.num_retrieval+instance.num_parking);
    for(auto &var:obj_variables)obj_expr+=var;
    model.setObjective(obj_expr, GRB_MINIMIZE);
    // Set constraints
    // vertex constraints
    for (size_t t = 0; t <= time_steps; t++){
        for (size_t v = 0; v < instance.grids->getNodesSize(); v++)
        {
            auto it = time_in_vector_map.find(get_id(99999, instance.grids->getVertex(v), nullptr, t));
            if (it == time_in_vector_map.end())
                continue;
            GRBLinExpr expr = GRBLinExpr();
            for (size_t i = 0; i < it->second.size(); i++)
                expr += it->second[i];
            if (expr.size() > 1)
                model.addConstr(expr, GRB_LESS_EQUAL, 1);
        }
    }

    //flow constraints
    for (size_t r = 0; r <= instance.num_retrieval; r++)
    {
        for (size_t t = 0; t <= time_steps; t++)
        {
            for (size_t v = 0; v < instance.grids->getNodesSize(); v++)
            {
                GRBLinExpr expr = GRBLinExpr();
                auto it = robot_in_vector_map.find(get_id(r, instance.grids->getVertex(v), nullptr, t));
                if (it != robot_in_vector_map.end())
                    for (size_t i = 0; i < it->second.size(); i++)
                        expr -= it->second[i];
                it = robot_out_vector_map.find(get_id(r,instance.grids->getVertex(v), nullptr, t));
                if (it != robot_out_vector_map.end())
                    for (size_t i = 0; i < it->second.size(); i++)
                        expr += it->second[i];
                if (expr.size() > 0)
                    model.addConstr(expr, GRB_EQUAL, 0);
            }
        }
    }

    //edge constraints
    for (auto it = edge_time_vector_map.begin(); it != edge_time_vector_map.end(); it++)
    {
        GRBLinExpr expr = GRBLinExpr();
        for (size_t i = 0; i < it->second.size(); i++)
            expr += it->second[i];
        if (expr.size() > 1)
            model.addConstr(expr, GRB_LESS_EQUAL, 1);
    }

    //corner following
    std::set<std::string> edge_keys;
    for(auto &e:edge_horizontal) edge_keys.insert(e.first);
    for(auto &e:edge_vertical) edge_keys.insert(e.first);

    for(auto &e:edge_keys){
        std::vector<GRBVar> horizontal_variables;
        std::vector<GRBVar> vertical_variables;
        if(edge_horizontal.find(e)!=edge_horizontal.end()) horizontal_variables=edge_horizontal[e];
        if(edge_vertical.find(e)!=edge_vertical.end()) vertical_variables=edge_vertical[e];
        GRBLinExpr horizontal_expr, vertical_expr;
        GRBVar horizontal_usage_variable=model.addVar(0,1,0,GRB_BINARY);
        GRBVar vertical_usage_variable=model.addVar(0,1,0,GRB_BINARY);
        for(auto &var:horizontal_variables){
            horizontal_expr+=var;
        }
        // std::cout<<horizontal_expr.size()<<" debug horizontal   ";
        horizontal_expr-=horizontal_expr.size()*horizontal_usage_variable;
        model.addConstr(horizontal_expr,GRB_LESS_EQUAL,0);
        for(auto &var:vertical_variables){
            vertical_expr+=var;
        }
        // std::cout<<vertical_expr.size()<<" debug vertical key="<< e<<std::endl;
        vertical_expr-=vertical_expr.size()*vertical_usage_variable;
        model.addConstr(vertical_expr,GRB_LESS_EQUAL,0);
        
        GRBLinExpr usage_expr;
        usage_expr=vertical_usage_variable+horizontal_usage_variable;
        model.addConstr(usage_expr,GRB_LESS_EQUAL,1);

    }

    //model.write("m.lp");
    model.update();
    return model;
}

void ILPSolver::retrieve_paths(GRBModel &model,size_t time_steps,Instance &instance)
{
    auto vars = model.getVars();
    final_paths = std::vector<std::vector<Location*>>(instance.num_retrieval, std::vector<Location*>(time_steps + 1, nullptr));
    // paths for retrieving  cars
    for (size_t r = 0; r < instance.num_retrieval; r++)
    {
        //std::cout<<" Robot "<<r<<" #####################"<<std::endl;
        final_paths[r][0] = instance.retrieval_starts[r];
        for (size_t t = 1; t < time_steps + 1; t++)
        {
            // std::cout<<" timestep "<<t<<std::endl;
            auto v1 = final_paths[r][t - 1];
            std::vector<Location *> nbrs = instance.grids->getNeighbors(v1);
            nbrs.push_back(v1);
            bool feasible=false;
            for (size_t i = 0; i < nbrs.size(); i++)
            {
                try
                {
                    if (fabs(model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X) -1.0)<1e-2)
                    {
                       // std::cout<<"neighbor "<<i<<"  "<<nbrs[i]<<"  size="<<nbrs.size()<<"  variable = "<<model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X)<<std::endl;
                        final_paths[r][t] = nbrs[i];
                        feasible=true;
                        break;
                    }
                    else{
                        //std::cout<<"neighbor  "<<nbrs[i]<<" var name="<<get_id(r, v1, nbrs[i], t - 1)<<" variable value= "<<model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X)<<std::endl;
                    }
                }
                catch (const GRBException &)
                {
                    // std::cout<<"GRBE exception"<<std::endl;
                    continue;
                }
            
            }
            // std::cout<<"current vertex is "<<v1<<std::endl;
        }
    }
     // paths for parking the cars
    for(size_t r=0;r<instance.num_parking;r++){
        Path pr(time_steps+1,nullptr);
        pr[0]=instance.other_starts[r];
        for(size_t t=1;t<time_steps+1;t++){
            Location* v1=pr[t-1];
            auto nbrs=instance.grids->getNeighbors(v1);
            nbrs.push_back(v1);
            for(size_t i=0;i<nbrs.size();i++){
                try{
                    if(fabs(model.getVarByName(get_id(instance.num_retrieval, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X) -1.0)<1e-2){
                        pr[t]=nbrs[i];
                        break;
                    }
                }
                catch (const GRBException &)
                {
                    // std::cout<<"GRBE exception"<<std::endl;
                    continue;
                }
            }

        }
        final_paths.push_back(pr);
    }
}

inline ILPSolver::id_type ILPSolver::get_id(size_t r, Location* v1, Location* v2, size_t t)
{
    std::string result;
    if(v2==nullptr) result=result+"null-";
    else result=result+std::to_string(v2->id)+'-';
    if(v1==nullptr) result=result+"null-";
    else result=result+std::to_string(v1->id)+"-";
    result+= std::to_string(r) + "-" + std::to_string(t);
    return result;
    // return std::to_string(v2.x) + "-" + std::to_string(v2.y) + "-" + std::to_string(v1.x) + "-" + std::to_string(v1.y) + "-" + std::to_string(r) + "-" + std::to_string(t);
}

inline void ILPSolver::store_var_for_vertices(GRBVar &var, Location* v1, Location* v2, size_t t1, size_t t2)
{
    id_type id = get_id(99999, v1, nullptr, t1);
    auto it = time_out_vector_map.find(id);
    if (it == time_out_vector_map.end())
        time_out_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    time_out_vector_map.find(id)->second.push_back(var);
    id = get_id(99999, v2, nullptr, t2);
    it = time_in_vector_map.find(id);
    if (it == time_in_vector_map.end())
        time_in_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    time_in_vector_map.find(id)->second.push_back(var);
}

inline void ILPSolver::store_var_for_robots(GRBVar &var, size_t r,Location* v1, Location* v2, size_t t1, size_t t2)
{
    
    id_type id = get_id(r, v1, nullptr, t1);
    auto it = robot_out_vector_map.find(id);
    if (it == robot_out_vector_map.end())
        robot_out_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    robot_out_vector_map.find(id)->second.push_back(var);
    id = get_id(r, v2, nullptr, t2);
    it = robot_in_vector_map.find(id);
    if (it == robot_in_vector_map.end())
        robot_in_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    robot_in_vector_map.find(id)->second.push_back(var);
}

inline void ILPSolver::store_var_for_edges(GRBVar &var, Location * v1, Location * v2, size_t t)
{
    id_type id = get_id(99999, v1, v2, t);
    if (v2 < v1)
        id = get_id(99999, v2, v1, t);
    auto it = edge_time_vector_map.find(id);
    if (it == edge_time_vector_map.end())
        edge_time_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    edge_time_vector_map.find(id)->second.push_back(var);
}

inline void ILPSolver::store_var_for_horizontal_edges(GRBVar &var,Location *v1,size_t t){
    id_type id=get_id(99999,v1,nullptr,t);
    auto it=edge_horizontal.find(id);
    if(it==edge_horizontal.end()) edge_horizontal.insert(make_pair(id,std::vector<GRBVar>()));
    edge_horizontal.find(id)->second.push_back(var);
    
}

inline void ILPSolver::store_var_for_vertical_edges(GRBVar &var,Location *v1,size_t t){
    id_type id=get_id(99999,v1,nullptr,t);
    auto it=edge_vertical.find(id);
    if(it==edge_vertical.end()) edge_vertical.insert(make_pair(id,std::vector<GRBVar>()));
    edge_vertical.find(id)->second.push_back(var);
}

void ILPSolver::solve(Instance &instance)
{
    // auto path_planner =
    //     SingleRobotPathPlanner<OmniDirectionalRobot::State,
    //                            OmniDirectionalRobot::Action,
    //                            OmniDirectionalRobot::Environment>(graph);
    // auto path_lengths = std::vector<size_t>();
    // // First step: calculate paths for each robot independently
    // for (size_t i = 0; i < task.num_robots; i++)
    //     path_lengths.push_back(path_planner.reversed_search(
    //                                            task.starts[i], task.goals[i])
    //                                .size());
    // Second step: Solve ILP
    // size_t time_step = *std::max_element(path_lengths.begin(), path_lengths.end()) - 1;
    // std::cout<<graph.adj_list[Node(0,0)].size()<<std::endl;
    int time_step=0;
    for(int i=0;i<instance.retrieval_starts.size();i++){
        time_step=std::max(instance.retrieval_starts[i]->manhattan_dist(instance.retrieval_goals[i]),time_step);
    }
    while (true)
    {
        try{
            GRBModel model = prepare_model(time_step, instance);
            model.optimize();
            double obj_val = model.get(GRB_DoubleAttr_ObjVal);
            
            // if(time_step>=1) break;
            std::cout<<"Current timestep="<<time_step<<"    "<<obj_val<<std::endl;
            // if (obj_val == instance.num_parking+instance.num_retrieval)
            // {
                std::cout<<"Solution found"<<std::endl;
                retrieve_paths(model, time_step,instance);
                return;
            // }
        }
        catch(const std::exception &e){
            time_step += 1;
        }
        catch(const GRBException &e){
            time_step+=1;
        }

    }
}


std::string ILPSolver::check_edge_type(Location *v1,Location *v2){
    if(v1==nullptr or v2==nullptr) return "None";
    if(v1->x==v2->x and v1->y+1==v2->y) return "vertical";
    if(v1->x==v2->x and v1->y-1==v2->y) return "vertical";
    if(v1->x+1==v2->x and v1->y==v2->y) return "horizontal";
    if(v1->x-1==v2->x and v1->y==v2->y) return "horizontal";
    return  "None";
}