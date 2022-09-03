/**
 * @file maxflow.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "common.hpp"
#include <ortools/graph/max_flow.h>
#include <ortools/graph/min_cost_flow.h>
#include "search.hpp"

class maxflowSolver{
public:
    using flowNode=std::tuple<int,int,int>;
    maxflowSolver(Grids *grids);
    void setConstraints(std::vector<Constraint_p> &constraints);
    Paths solveWeighted();

private:
    Configs starts;
    Configs goals;
    Grids *grids;
    void prepare(int timestep);
    void add_edge(flowNode &u,flowNode &v);
    void insert_node(int &id,flowNode &node);
    void resolveEdgeConflicts(Paths &result);
    void retrievePaths(operations_research::MinCostFlow &flow, Paths& result);
    void switchPaths(int i,int j,int t,Paths &result);
    std::vector<int> startNodes;
    std::vector<int> endNodes;
    //std::vector<int> capacities;
    std::unordered_map<int,flowNode> id_node;
    std::unordered_map<flowNode,int,boost::hash<flowNode>>node_id;
    std::vector<Constraint_p> m_constraints;
    void evaluateLBfast();
    void evaluateLB();
    int source_id;
    int sink_id;
};