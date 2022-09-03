/**
 * @file common.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include<vector>
#include<iostream>
#include<memory>
#include<unordered_map>
#include<unordered_set>
#include<vector>
#include<queue>
#include<fstream>
#include <chrono>
#include <cmath>
#include <regex>
#include <boost/functional/hash.hpp>
#include <stack>
#include <limits>
#include "json.hpp"
#include <random>

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;
typedef std::pair<int,int> point2d;

///////////////////////////////////////////////////////////////////////////////
class Location{
public:
    Location(int _id,int _x,int _y):x(_x),y(_y),id(_id){}
    int x,y,id;
    int manhattan_dist(Location *other){
        return abs(x-other->x)+abs(y-other->y);
    }
    std::string print(){
        return "("+std::to_string(x)+","+std::to_string(y)+")";
    }
    // std::vector<Location3d*> neighbors;
};
//////////////////////////////////////////////////////////////////////////////

class Grids{
public:

    Grids(std::string file_name);
    Grids(int xmax,int ymax);
    Grids(int xmax,int ymax,std::vector<std::vector<int>> &obstacles);
 
    std::vector<Location *> getNeighbors(Location *);
    Location*getVertex(int x,int y);
    Location*getVertex(int id);

    int xmax,ymax;
    int getNodesSize(){
        return nodes.size();
    }
    std::vector<Location*> getNodes(){
        return nodes; 
    }



    bool isBlocked(Location *v){
        return this->nodes[v->id]==nullptr;
        // return obstacles.find(v)!=obstacles.end();
    }

protected:
    std::vector<Location *> nodes;
    // std::set<Location *> obstacles;
};
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
using Configs=std::vector<Location*>;
using Path=std::vector<Location*>;
using Paths=std::vector<Path>;
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class Agent{
public:
    Agent(){}
    Location *current;
    Location *goal;
    int id;
    int priority;
    Agent(Location *current,int id):current(current),id(id){}
    Path path;
    Path plan;

};

using Agent_p=std::shared_ptr<Agent>;
using Agents=std::vector<Agent_p>;
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// void read_graph(std::string file_name,Grids *&graph);
void read_scen(std::string file_name,Configs&starts,Configs &goals,Grids *graph);
void save_solutions(std::string file_name,Paths&paths,double runtime,bool save_paths);
void check_feasible_bruteForce(Paths &);
void format_paths(Paths &paths);
void evaluate_result(Paths &,int &makespan,int &makespanLB,int &soc,int &socLB);   
void fill_paths(Paths &,int makespan=-1);
void shrink_paths(Paths&paths);
void save_result_as_json(std::string file_name,Paths &paths,double runtime=0,bool save_paths=false);
void linear_sum_assignment(const Configs &starts,const Configs &goals,Configs &assigned_goal);
void save_data_as_csv(std::string file_name,std::vector<double> & data1,std::vector<double> &data2);
void evaluate_result2(Paths &,double &makespan,double &makespanLB,double &sod,double &sor,int num_retrieval=0);

inline double time_elapsed(
    std::chrono::time_point<std::chrono::high_resolution_clock>& start_time) {
    return double(static_cast<long long int>(
               std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now() - start_time)
                   .count())) /
           1000000;
}
// void precompute_dist(Grids*graph,Configs&starts,Configs&goals,std::vector<std::tuple<int,int,double>>&costEdges);
//////////////////////////////////////////////////////////////////////////////////