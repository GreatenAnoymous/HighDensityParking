/**
 * @file instance.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
#include "common.hpp"

class Instance{
public:
    Grids *grids;
    Instance(){
        
    }

    void readLocationHelper( const nlohmann::json &data_dict,std::string item,std::vector<Location*>&output){
        output.clear();
        std::vector<std::vector<int>> infoVector=data_dict[item];
        for(auto&vec:infoVector){
            output.push_back(grids->getVertex(vec[0],vec[1]));
        }
    }

    Instance(std::string instance_json){
        nlohmann::json setting_json;
        std::ifstream ifs(instance_json);
        setting_json=nlohmann::json::parse(ifs);
        xmax=setting_json["xmax"];
        ymax=setting_json["ymax"];
        grids=new Grids(xmax,ymax);
        num_retrieval=setting_json["num_retrieval"];
       
        ////////////////////////////////////////////////////
        readLocationHelper(setting_json,"ParkingSlots",ParkingSlots);
        /////////////////////////////////////////////////////////////
        readLocationHelper(setting_json,"IOports",IOports);
        ////////////////////////////////////////////////////////////////
        readLocationHelper(setting_json,"retrieval_starts",retrieval_starts);
        /////////////////////////////////////////////////////////////////////
        readLocationHelper(setting_json,"retrieval_goals",retrieval_goals);
        assert(retrieval_goals.size()==retrieval_starts.size());
        assert(retrieval_starts.size()==num_retrieval);
        ////////////////////////////////////////////////////////////////////////
        readLocationHelper(setting_json,"other_starts",other_starts);
        num_parking=setting_json["num_parking"];
        num_parking+=other_starts.size();

        Configs parking_starts;
        readLocationHelper(setting_json,"parking_starts",parking_starts);
        for(auto &ps:parking_starts) other_starts.push_back(ps);
        // assert(num_parking==other_starts.size());
        ////////////////////////////////////////////////////////////////////
    }
    int num_retrieval;
    int num_parking;
    int xmax;
    int ymax;
    // std::vector<std::vector<int>> obstacles;// if any
    std::vector<Location*> ParkingSlots;
    std::vector<Location*> IOports;
    Configs retrieval_starts;
    Configs retrieval_goals;
    Configs other_starts;
    
};

/**
 * @brief Create a instance object
 * 
 * @param xmax 
 * @param ymax 
 * @param num_parking 
 * @param num_retrieval 
 * @param num_other 
 * @return Instance 
 */
inline Instance create_instance(int xmax,int ymax,int num_parking,int num_retrieval,int num_other){
    std::random_device rd;
    std::mt19937 g(rd());
    Instance instance;
    instance.xmax=xmax;
    instance.ymax=ymax;
    instance.grids=new Grids(xmax,ymax);
    instance.num_retrieval=num_retrieval;
    instance.num_parking=num_parking+num_other;
    for(int x=1;x<=xmax-2;x++){
        for(int y=0;y<ymax-2;y++){
            instance.ParkingSlots.push_back(instance.grids->getVertex(x,y));
        }
    }

    for(int x=1;x<=xmax-2;x++){
        instance.IOports.push_back(instance.grids->getVertex(x,ymax-1));
    }

    auto IOPorts_copy=instance.IOports;
    auto slots_copy=instance.ParkingSlots;
    std::shuffle(IOPorts_copy.begin(),IOPorts_copy.end(),g);
    std::shuffle(slots_copy.begin(),slots_copy.end(),g);

    for(int i=0;i<num_retrieval;i++){
        auto start_i=slots_copy.back();
        slots_copy.pop_back();
        auto goal_i=IOPorts_copy.back();
        IOPorts_copy.pop_back();
        instance.retrieval_starts.push_back(start_i);
        instance.retrieval_goals.push_back(goal_i);
    }

    for(int i=0;i<num_parking;i++){
        auto start_i=IOPorts_copy.back();
        assert(IOPorts_copy.empty()==false);
        IOPorts_copy.pop_back();
        // instance.grids
        instance.other_starts.push_back(start_i);
    }

    for(int i=0;i<num_other;i++){
        auto start_i=slots_copy.back();
        slots_copy.pop_back();
        instance.other_starts.push_back(start_i);
    }
    return instance;
}