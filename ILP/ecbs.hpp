/**
 * @file ecbs.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
#include "common.hpp"
#include "instance.hpp"
#include "search.hpp"


class ECBSNode{
public:
    int id;
    Paths paths;
    Constraints constraints;
    int makespan;
    int soc;
    int f;
    bool valid;
    int LB;
    std::vector<int> fmins;
    ECBSNode(){};
    ECBSNode(int _id,Paths _paths, Constraints _c, int _m, int _soc,int _f,int _LB,bool _valid,std::vector<int>& _fmins){
        this->id=_id;
        this->paths=_paths;
        this->constraints=_c;               //constraints     
        this->makespan=_m;                //makespan
        this->soc=_soc;                  //sum of costs
        this->f=_f;                     //focal heuristic=====number of conflicts
        this->valid=_valid;
        this->LB=_LB;                   //lowerbound
        this->fmins=_fmins;
    }
    ECBSNode(int _id,Paths _paths, Constraints _c, int _m, int _soc,int _f,int _LB=0,bool _valid=true):
        id(_id),paths(_paths),constraints(_c),makespan(_m),soc(_soc),f(_f),LB(_LB),valid(_valid){}
};



class ECBS{
public:
    using ECBSNode_p=std::shared_ptr<ECBSNode>;
    using ECBSNodes=std::vector<ECBSNode_p>;

    ECBS(double w=1.5){this->w=w;}
    ECBS(Instance &instance,double w=1.5);
    void solve();

    void setInitialNode(ECBSNode_p node);
    void setInitialHighLevelNodeFocal(ECBSNode_p node);
    void invoke(ECBSNode_p node,int id);
    std::function<bool(ECBSNode_p,ECBSNode_p)> compareOpen;
    std::function<bool(ECBSNode_p,ECBSNode_p)> compareFocal;
    std::vector<Constraints> findFirstConstraints(ECBSNode_p n);
    using openList=std::priority_queue<ECBSNode_p,ECBSNodes,decltype(compareOpen)>;
    using focalList=std::priority_queue<ECBSNode_p,ECBSNodes,decltype(compareFocal)>;
    // void setSingleSolver();
    void singleSolver(ECBSNode_p node,int agent,Path &path,int &fLB);
    Paths solution;

private:
    double w;
    Grids *m_grids;
    Configs starts,goals;
    // Instance *m_instance;
    int countConflict(Paths &paths,Path &path,int id,int makespan);
    int countConflicts(Paths &paths,int makespan);
    void initFunc();
};

