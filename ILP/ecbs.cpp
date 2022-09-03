/**
 * @file ecbs.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include"ecbs.hpp"

ECBS::ECBS(Instance &instance,double w){
    this->w=w;
    for(int i=0;i<instance.num_retrieval;i++){
        starts.push_back(instance.retrieval_starts[i]);
        goals.push_back(instance.retrieval_goals[i]);
    }
    Configs assigned_goals;
    Configs parking_starts;

    for(int i=0;i<instance.num_parking;i++){
        parking_starts.push_back(instance.other_starts[i]);

    }
    // std::cout<<parking_starts.size()<<std::endl;
    linear_sum_assignment(parking_starts,instance.ParkingSlots,assigned_goals);
    // std::cout<<parking_starts.size()<<std::endl;
    for(int i=0;i<parking_starts.size();i++){
        // std::cout<<"ECBS  "<<parking_starts[i]->print()<<"      "<<std::endl;
        starts.push_back(parking_starts[i]);
        goals.push_back(assigned_goals[i]);
    }
    // std::cout<<"Assigned goals size="<<assigned_goals.size()<<"   "<<assigned_goals[0]->print()<<std::endl;
    // std::cout<<"DEBUG  TOTAL NUMBER OF ROBOTS "<<starts.size()<<"   "<<goals.size()<<std::endl;
    m_grids=instance.grids;
    initFunc();
}

void ECBS::initFunc(){
    compareOpen=[&](ECBSNode_p n1,ECBSNode_p n2){
        return n1->soc>n2->soc;
    };
    compareFocal=[&](ECBSNode_p n1,ECBSNode_p n2){
        return n1->f>n2->f;
    };
}


// void ECBS::setSingleSolver(){
//     singleAgentSolver=[&](Grid* grid,Location* s, Location*g,Constraints &cs,Paths &paths, int id)->CBS::SinglePath{
//         A_star_epsilon ll_solver(grid,s,g,cs,paths,id,w);
//         ll_solver.initFunctions();
//         ll_solver.computeFocal=[&](AStarPlanner::AStarNode_p n2)->int{
//             int numConflicts=0;
//             auto n1=n2->parent;
//             int numAgents=paths.size();
            
//             for(int j=0;j<numAgents;j++){
//                 if(j==id) continue;
//                 int lj=paths[j].size();
//                 if(lj==0) continue;
//                 int tj=std::min(n2->t,lj-1);
//                 //vertex conflicts
//                 if(paths[j][tj]==n2->v) numConflicts++;

//                 if(n2->t>=lj) continue;
//                 //edge conflicts
//                 if(paths[j][n2->t]==n1->v and paths[j][n1->t]==n2->v){
//                     numConflicts++;
//                     continue;
//                 }
//                 //corner following conflicts
//                 int t=n2->t;
//                 int dxi=n2->v->pos.x-n1->v->pos.x;
//                 int dyi=n2->v->pos.y-n1->v->pos.y;
//                 int dxj=paths[j][t]->pos.x-paths[j][t-1]->pos.x;
//                 int dyj=paths[j][t]->pos.y-paths[j][t-1]->pos.y;

//                 //j follow i
//                 if(paths[j][t-1]==n2->v and dxi!=dxj and dyi!=dyj) numConflicts++;
//                 //i follow j
//                 if(paths[j][t]==n1->v and dxi!=dxj and dyi!=dyj) numConflicts++;
//             }
//             return numConflicts;

//         };
//         ll_solver.solve();
//         return ll_solver.getSolution();
//     };
// }


std::vector<Constraints> ECBS::findFirstConstraints(ECBSNode_p n){
    auto check_edge_feasible=[](Location *u,Location *v,Location *e1,Location *e2){
        int dx=v->x-u->x;
        int dy=v->y-u->y;
        int dx2=e2->x-e1->x;
        int dy2=e2->y-e1->y;
        return dx==dx2 and dy==dy2;
    };
    int num_agents=n->paths.size();
    int makespan=n->makespan;
    for(int t=1;t<makespan;t++){
        for(int i=0;i<num_agents;i++){
            for(int j=i+1;j<num_agents;j++){
                //vertex conflicts
                int li=n->paths[i].size();
                int lj=n->paths[j].size();
                if(t>=li and t>=lj) continue;
                int ti=std::min(li-1,t);
                int tj=std::min(lj-1,t);
                if(n->paths[i][ti]==n->paths[j][tj]){
                    Constraint_p c1=std::make_shared<Constraint>(i,t,n->paths[i][ti],nullptr);
                    Constraint_p c2=std::make_shared<Constraint>(j,t,n->paths[j][tj],nullptr);
                    // std::cout<<"vertex conflict ("<<i<<" "<<j<<" t="<<t<<"  "<<n->paths[j][tj]->print()<<" )";
                    // n->paths[i][ti]->print();
                    // std::cout<<"\n";
                    return {{c1},{c2}};
                }

                if(t>=li or t>=lj) continue;
                if(n->paths[i][t-1]==n->paths[i][t] or n->paths[j][t-1]==n->paths[j][t]) continue;
                
                auto vit=n->paths[i][t];
                auto vitt=n->paths[i][t-1];
                auto vjt=n->paths[j][t];
                auto vjtt=n->paths[j][t-1];


                //edge conflicts
                if(n->paths[i][t-1]==n->paths[j][t] and n->paths[i][t]==n->paths[j][t-1]){
                    Constraints ci,cj;
                    // auto nbrs=m_grids->getNeighbors(vitt);
                    auto c_=std::make_shared<Constraint>(i,t,vitt,vit);
                        ci.push_back(c_);
                    c_=std::make_shared<Constraint>(j,t,vjtt,vjt);
                            cj.push_back(c_);
                    // for(auto n:nbrs){
                    //     if(check_edge_feasible(vitt,n,vjtt,vjt)==false){
                    //         auto c_=std::make_shared<Constraint>(i,t,vitt,vit);
                    //         ci.push_back(c_);
                    //     }
                    // }
                    // for(auto n:nbrs){
                    //     if(check_edge_feasible(vjtt,n,vitt,vit)==false){
                    //         auto c_=std::make_shared<Constraint>(j,t,vjtt,vjt);
                    //         cj.push_back(c_);
                    //     }
                    // }
                    // std::cout<<"edge conflicts ("<<ci[0]->as_string()<<"   "<<cj[0]->as_string()<<std::endl;
                    return {ci,cj};
                }
                //corner following conflicts
                
                int dxi=vit->x-vitt->x;
                int dxj=vjt->x-vjtt->x;
                int dyi=vit->y-vitt->y;
                int dyj=vjt->y-vjtt->y;
                //i follow j
                if(n->paths[j][t-1]==n->paths[i][t] and (dxi!=dxj or dyi!=dyj)){
                    Constraints ci,cj;
                    //for agent i
                    auto c1=std::make_shared<Constraint>(i,t,vitt,vit);
                    ci.push_back(c1);
                    //for agent j
                    // auto nbrs=m_grids->getNeighbors(vjtt);
                    // for(auto n:nbrs){
                        // if(check_edge_feasible(vjtt,n,vitt,vit)==false){
                    auto c_=std::make_shared<Constraint>(j,t,vjtt,vjt);
                    cj.push_back(c_);
                        // }
                    // }
                    // std::cout<<"corner conflict ("<<i<<","<<j<<","<<t<<")"<<std::endl;
                    // vitt->print();
                    // std::cout<<"--->";
                    // vit->print();
                    // std::cout<<"\n";
                    // vjtt->print();
                    // std::cout<<"--->";
                    // vjt->print();
                    return {ci,cj};
                }
                // j follow i
                if(n->paths[i][t-1]==n->paths[j][t] and (dxi!=dxj or dyi!=dyj)){
                    Constraints ci,cj;
                    //for agent j
                    auto c1=std::make_shared<Constraint>(j,t,vjtt,vjt);
                    ci.push_back(c1);
                    auto c_=std::make_shared<Constraint>(i,t,vitt,vit);
                    cj.push_back(c_);
                    // auto nbrs=m_grids->getNeighbors(vjtt);
                    //for agent i
                    // for(auto n:nbrs){
                    //     if(check_edge_feasible(vitt,n,vjtt,vjt)==false){
                    //         auto c_=std::make_shared<Constraint>(i,t,vitt,n);
                    //         cj.push_back(c_);
                    //     }
                    // }
                    // std::cout<<"corner conflict ("<<i<<","<<j<<","<<t<<")"<<std::endl;
        
                    return {ci,cj};
                }
            }
        }
    }
    return {};
}



void debug_helper(Constraints &c_set,Constraint_p ci){
    for(auto &c:c_set){
        if(c->agent_id==ci->agent_id and c->t==ci->t and c->u==ci->u and c->v==ci->v){
            std::cout<<"Constraint already exists! ("<<ci->agent_id<<","<<ci->t<<","<<ci->u->print()<<")";
            if(c->v!=nullptr) std::cout<<ci->v->print()<<std::endl;
            assert(false);
        }
    }
}



void ECBS::solve(){
    auto t0 = Time::now();
    openList open(compareOpen);
    focalList focal(compareFocal);

    ECBSNode_p n=std::make_shared<ECBSNode>();
    // setInitialHighLevelNode(n);
    setInitialHighLevelNodeFocal(n);
    open.push(n);
    bool success=false;
    int hid=0;
    int currentLB=0;
    // std::unordered_set<HighLevelNode_p> closed;
    while(open.empty()==false){
        auto t1=Time::now();
        fsec fs=t1-t0;
        auto duration=fs.count();
        if(duration>300) {
            std::cout<<"TIME OUT"<<std::endl;
            // throw std::runtime_error("time out!");
            break;
        }
        auto best=open.top();
        // std::cout<<"current open size="<<open.size()<<std::endl;
        if(best->valid==false){
            open.pop();
            // std::cout<<"this node is invalid"<<std::endl;
            continue;
        }
        // assert(closed.find(best)==closed.end());
        // numHighLevelExpansions++;
    
        if(currentLB!=best->LB){                        //update the lowerbound
            ECBSNodes tmp;
            currentLB=best->LB;
            focal=focalList(compareFocal);
            
            while(open.empty()==false){
                auto node=open.top();
                if(node->LB>w*currentLB) break;
                open.pop();
                tmp.push_back(node);
                focal.push(node);
            }  
            // assert(focal.empty()==false);
            auto focalNode=focal.top();
            for(auto &n:tmp){
                if(n==focalNode) continue;
                open.push(n);                           //push back n
            }
        }

        // std::cout<<open.size()<<" "<<focal.size()<<" "<<<<std::endl;
        auto focalNode=focal.top();
        // assert(closed.find(focalNode)==closed.end());
        focal.pop();
        // closed.insert(focalNode);
        focalNode->valid=false;
        // std::cout<<"id="<<numHighLevelExpansions<<" open size="<<open.size()<<" soc="<<focalNode->soc<<" num conflicts="<<focalNode->f<<" constraints size="<<focalNode->constraints.size()<<std::endl;        
        //adding new nodes
        // if(numHighLevelExpansions>200) assert(false);
        // Constraints newConstraints=findFirstConstraints(focalNode);
        std::vector<Constraints> newConstraints=findFirstConstraints(focalNode);
        if(newConstraints.empty()){
            success=true;
            solution.swap(focalNode->paths);
            std::cout<<"I found the solutions, yeah!"<<std::endl;
            break;
        }
        for(auto &cs:newConstraints){
            // assert(newConstraints.size()==2);
            bool new_c_added=false;
            Constraints new_c=focalNode->constraints;                   //new constraints
            for(auto &c:cs){
                // std::cout<<"adding the constraint  "<<c->as_string()<<std::endl;
                // debug_helper(new_c,c);
                new_c.push_back(c);
                // bool found=false;
                // for(auto &ci:new_c){
                //     if(ci->t==c->id and ci->u==ci->u and ci->v==ci->v){
                //         std::cout<<" the constraint already exists"<<std::endl;
                //         found=true;
                //         break;
                //     }
                // }
                // if(!found){
                //     new_c.push_back(c);
                //     new_c_added=true;
                // }
            }
            // if(new_c_added==false) continue;
            ECBSNode_p new_node=std::make_shared<ECBSNode>(hid,focalNode->paths,new_c,focalNode->makespan,focalNode->soc,focalNode->f,focalNode->LB,true,focalNode->fmins);
            invoke(new_node,cs[0]->agent_id);
            if(new_node->valid==true){
                open.push(new_node);
                if(new_node->LB<=w*currentLB) focal.push(new_node);
            }   
            // assert(new_node->constraints.size()-focalNode->constraints.size()==1);
            // else std::cout<<"invalid!!!!!!!!!!\n";
            
        }

        
    }
    if(success==false){
        solution={};
        // std::cout<<"is open empty? "<<open.size()<<std::endl;
        // throw std::runtime_error("I failed to find the solutions!");
    }

}


void ECBS::singleSolver(ECBSNode_p node,int agent,Path &path,int &fLB){
    AStarEps searcher;
    int max_constraint_time=0;
    for(auto const &c:node->constraints){
        if(c->agent_id==agent)
            max_constraint_time=std::max(max_constraint_time,c->t);
    }

    searcher.compareOpen=[&](AStarEps::AStarEpsNode_p a1,AStarEps::AStarEpsNode_p a2)->bool{
        if(a1->f!=a2->f) return a1->f> a2->f;
        // if(a1->t!=a2->t) return a1->t< a2->t;
        return false;
    };

    searcher.compareFocal=[&](AStarEps::AStarEpsNode_p a1,AStarEps::AStarEpsNode_p a2)->bool{
        if(a1->focalH!=a2->focalH) return a1->focalH > a2->focalH;
        if(a1->f!=a2->f) return a1->f >a2->f;
        if(a1->t!=a2->t) return a1->t < a2->t;
        return false;
    };

    searcher.isSolution=[&](AStarEps::AStarEpsNode_p n)->bool{
        return n->v==this->goals[agent] &&n->t >=max_constraint_time;
    };

    searcher.checkValid=[&](AStarEps::AStarEpsNode_p n2)->bool{
        // for(auto &c:node->constraints){

        //     std::cout<<c->agent_id<<"    t="<<c->t<<"   u= "<<c->u->print()<<"    ";
        //     if(c->v!=nullptr) c->v->print();
        //     std::cout<<std::endl;
        // }
        // std::cout<<"check valid for agent  "<<agent<<" AstarNode "<<n2->v->print()<<"  "<<n2->t;
        for(auto &c:node->constraints){
            if(c->v==nullptr){
                //vertex constraints
                if(c->t==n2->t && c->u==n2->v &&c->agent_id==agent) {
                    // std::cout<<" false "<<std::endl;
                    return false;
                }
            }else{
                //edge constraints
                auto n1=n2->parent;
                if(n1==nullptr) return true;
                if(c->t==n2->t&& c->u==n1->v &&c->v==n2->v &&c->agent_id==agent) {
                    // std::cout<<" false "<<std::endl;
                    return false;
                }
            }
        }
        // std::cout<<" true "<<std::endl;
        return true;
    };

    searcher.getSuccessors=[&](AStarEps::AStarEpsNode_p n){
        AStarEps::AStarEpsNodes children;
        auto neighbors=m_grids->getNeighbors(n->v);
        neighbors.push_back(n->v);
        for(auto &vn:neighbors){
            int tn=n->t+1;                                      // tn
            int fn=n->v->manhattan_dist(goals[agent])+tn;                           //fn=gn+hn     
            AStarEps::AStarEpsNode_p new_node=std::make_shared<AStarEpsNode>(vn,fn,tn,0,n);
            children.push_back(new_node);
        }
        return children;  
    };

    searcher.computeFocal=[&](AStarEps::AStarEpsNode_p n2)->int{      //n1--->parent    n2--> children
        int num_conflicts=0;
        auto n1=n2->parent;
        int numAgents=node->paths.size();
        for(int i=0;i<numAgents;i++){
            if(i==agent) continue;
            int li=node->paths[agent].size()-1;
            if(li<0) continue;
            int ti=std::min(n2->t,li);
            int tj=std::min(n1->t,li);
            if(node->paths[agent][ti]==n2->v) num_conflicts++;     //vertex conflicts
            else{
                if(n2->t>li) continue;
                if(node->paths[i][tj]==n2->v &&node->paths[i][ti]==n1->v) num_conflicts++; //edge conflicts
            }
            
        }
        return num_conflicts;
    };

    path=searcher.solve(m_grids,starts[agent],goals[agent],agent);
    fLB=searcher.fmin;

}


void ECBS::invoke(ECBSNode_p node,int id){
    // SinglePath solution=singleAgentSolver(p->graph,p->starts[id],p->goals[id],node->constraints,node->paths,id);
    // if(solution.second.empty()){
    //     node->valid=false;
    //     return;
    // }
    int newLB=0;
    Path path;
    // std::cout<<"replanning for agent"<<id<<std::endl;
    singleSolver(node,id,path,newLB);
    if(path.size()==0){
        node->valid=false;
        return;
    }
    // int newLB=solution.first;
    // assert(node->fmins.size*()!=0);
    node->LB=node->LB-node->fmins[id]+newLB;
    node->soc=node->soc-node->paths[id].size()+path.size();
    
    //makespan
    int mkpn=0;
    int lengthSolution=path.size();
    if (path.size()>node->makespan) mkpn=lengthSolution;
    else if(node->paths[id].size()==node->makespan){
        for(int i=0;i<node->paths.size();i++){
            if(i==id) continue;
            mkpn=std::max(mkpn,(int)node->paths[i].size());
        }
        mkpn=std::max(mkpn,lengthSolution);
    } 
    else mkpn=node->makespan;

    //fvalue
    int f_new=countConflict(node->paths,path,id,mkpn);             //count the conflict number for old paths
    int f_old=countConflict(node->paths,node->paths[id],id,node->makespan);             //count the conflict number for new paths
    int fvalue=node->f-f_old+f_new;


    node->paths[id].swap(path);
    node->makespan=mkpn;
    node->f=fvalue;
    node->fmins[id]=newLB;
}
 

int ECBS::countConflict(Paths &paths, Path &path, int id,int makespan){
    int numConflicts=0;
    int numAgents=paths.size();
    for(int t=1;t<makespan;t++){
        for(int j=0;j<numAgents;j++){
            if(j==id) continue;
            int lj=paths[j].size();
            int li=path.size();
            //vertex conflicts
            if(t>=li and t>=lj) continue;
            int ti=std::min(li-1,t);
            int tj=std::min(lj-1,t);
            if((ti<path.size() and tj<paths[j].size())==false){
                std::cout<<ti<<"  "<<path.size()<<"  "<<tj<<"   "<<paths[j].size()<<std::endl;
                assert(ti<path.size() and tj<paths[j].size());
            }
            
            if(path[ti]==paths[j][tj]) numConflicts++;
            //edge and corner following conflicts
            if(t>=li or t>=lj) continue;
            //edge conflicts
            if(path[t]==paths[j][t-1] and path[t-1]==paths[j][t]) {
                numConflicts++;
                continue;
            }
            int dxi=path[t]->x-path[t-1]->x;
            int dyi=path[t]->x-path[t-1]->y;
            int dxj=paths[j][t]->x-paths[j][t-1]->x;
            int dyj=paths[j][t]->y-paths[j][t-1]->y;
            if(path[t]==paths[j][t-1] and dxi==dxj and dyi==dyj) numConflicts++;
            if(path[t-1]==paths[j][t] and dxi==dxj and dyi==dyj) numConflicts++; 
        }
    }
    return numConflicts;
}

int ECBS::countConflicts(Paths &paths,int makespan){
    int numConflicts=0;
    int numAgents=paths.size();
    for(int t=1;t<makespan;t++){
        for(int i=0;i<numAgents;i++){
            for(int j=i+1;j<numAgents;j++){
                int lj=paths[j].size();
                int li=paths[i].size();
                //vertex conflicts
                if(t>=li and t>=lj) continue;
                int ti=std::min(li-1,t);
                int tj=std::min(lj-1,t);
                if(paths[i][ti]==paths[j][tj]) numConflicts++;
                //edge and corner following conflicts
                if(t>=li or t>=lj) continue;
                //edge conflicts
                if(paths[i][t]==paths[j][t-1] and paths[i][t-1]==paths[j][t]) {
                    numConflicts++;
                    continue;
                }
                int dxi=paths[i][t]->x-paths[i][t-1]->x;
                int dyi=paths[i][t]->x-paths[i][t-1]->y;
                int dxj=paths[j][t]->x-paths[j][t-1]->x;
                int dyj=paths[j][t]->y-paths[j][t-1]->y;
                if(paths[i][t]==paths[j][t-1] and (dxi!=dxj and dyi!=dyj)) numConflicts++;
                if(paths[i][t-1]==paths[j][t] and (dxi!=dxj and dyi!=dyj)) numConflicts++; 
            }
        }
        
    }
}


void ECBS::setInitialHighLevelNodeFocal(ECBSNode_p node){
    int numAgents=starts.size();
    node->id=0;
    node->paths.resize(numAgents,{});
    node->fmins.clear();
    node->LB=0;
    int makespan=0,soc=0;
  
    for(int i=0;i<numAgents;i++){
        Constraints c={};
        int fLB;
        Path path;
        singleSolver(node,i,path,fLB);
        // auto solution=singleAgentSolver(p->graph,p->starts[i],p->goals[i],c,node->paths,w);
        soc+=path.size();
        makespan=std::max((int)path.size(),makespan);
        node->fmins.push_back(fLB);
        node->LB+=fLB;
        node->paths[i].swap(path);
    }
    node->makespan=makespan;
    node->soc=soc;
    node->constraints={};
    node->f=countConflicts(node->paths,makespan);
    node->valid=true;
}

