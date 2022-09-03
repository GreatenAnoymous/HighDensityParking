/**
 * @file search.hpp
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Constraint{
public:
    Location* u;   //
    Location* v;   //used for edge constraint
    int t;
    int agent_id;

    Constraint(int agent_id,int t,Location* u,Location* v=nullptr){
        this->agent_id=agent_id;
        this->t=t;
        this->u=u;
        this->v=v;
    }

    std::string as_string(){
        if(v!=nullptr)
            return std::to_string(agent_id)+"-"+std::to_string(t)+"-"+u->print()+"-"+v->print();
        return std::to_string(agent_id)+"-"+std::to_string(t)+"-"+u->print();
    }
};
using Constraint_p=std::shared_ptr<Constraint>;
using Constraints=std::vector<Constraint_p>;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class AStarNode{
public:
    using AStarNode_p=std::shared_ptr<AStarNode>;
    int f,t,numConflicts;
    Location *v;
    AStarNode_p parent;
    AStarNode(Location*v,int f,int t,int numConflicts,AStarNode_p parent=nullptr):v(v),f(f),t(t),numConflicts(numConflicts),parent(parent){}
    
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class AStar{
public:
    // using ReservationTable_p=std::shared_ptr<ReservationTable>;
    using AStarNode_p=std::shared_ptr<AStarNode>;
    using AStarNodes=std::vector<AStarNode_p>;
    AStar();
    Location *m_goal;
    // ReservationTable_p m_rt;

    Path search(Location *start,Location *goal,Grids *grids,int start_time=0){
        return{};
    }
    

    std::function<bool(AStarNode_p astarNode)> isSolution;
    std::function<bool(AStarNode_p astarNode)> checkValid;
    std::function<AStarNodes(AStarNode_p AStarNode)> getSuccessor;

    int countConflicts(AStarNode_p astarNode){
        int num_conflicts=0;
        return num_conflicts;
    }
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class AStarEpsNode{
public:
    Location *v;
    int f;
    int t;
    int focalH;
    std::shared_ptr<AStarEpsNode> parent;
    AStarEpsNode(Location* v,int f,int t,int focalH,std::shared_ptr<AStarEpsNode>parent=nullptr):
        v(v),f(f),t(t),focalH(focalH),parent(parent){}
    std::string to_string(){
        return std::to_string(v->id)+"-"+std::to_string(t);
    }
};


class AStarEps{
public:
    using AStarEpsNode_p=std::shared_ptr<AStarEpsNode>;
    using AStarEpsNodes=std::vector<AStarEpsNode_p>;
    AStarEps(){}
    int fmin;
    Path solve(Grids*grids,Location *start,Location *goal,int id,double w=1.5){
        openList open(compareOpen);
        focalList focal(compareFocal);
        Path result;
        std::unordered_set<std::string> closed;
        int f0,g0,fLB=-1;
        // std::cout<<"debug   "<<start->print()<<"   "<<goal->print()<<std::endl;
        f0=start->manhattan_dist(goal);
        AStarEpsNode_p start_node=std::make_shared <AStarEpsNode>(start,f0,0,0,nullptr);
        open.push(start_node);
        bool success=false;
        AStarEpsNode_p goalNode;
        // m_constraints=constraints;
        m_goal=goal;
        m_id=id;
        m_grids=grids;
        // std::cout<<"A star epsilon for agent ="<<id<<std::endl;
        // std::cout<<start->id<<" start"<<std::endl;
        // for(auto &c:*constraints){
        //     if(c->u==nullptr)
        //         std::cout<<"vertex:"<<c->id<<" "<<c->t<<" "<<c->v->id<<std::endl;
        //     else std::cout<<"edge:"<<c->id<<" "<<c->t<<" "<<c->v->id<<" "<<c->u->id<<std::endl;
        // }
        // assert(w==1.5);
        while(open.empty()==false){
            auto best=open.top();
            if(closed.find(best->to_string())!=closed.end()) {
                open.pop();
                continue;
            }
            // numberOfExpansions++;
            // std::cout<<"fLB="<<fLB<<" "<<best->f<<std::endl;
            // int fLB=best->f;
            if(fLB!=best->f){                                                      //Lowerbound updated
                AStarEpsNodes tmp;
                fLB=best->f;
                // std::cout<<"debug open size="<<open.size()<<std::endl;
                focal=focalList(compareFocal);
                while(open.empty()==false){
                    auto node=open.top();
                    if(node->f<=w*fLB){                                         // add nodes to focal list
                        open.pop();
                        if(closed.find(node->to_string())!=closed.end()) continue;
                        tmp.push_back(node);
                        focal.push(node);
                    }
                    else{
                        break;
                    }
                }
                auto focalNode=focal.top();
                for(auto &v:tmp){
                    if(v!=focalNode) open.push(v);                              //add  nodes back to open
                }
            }
            
        
            auto focalNode=focal.top();
            focal.pop();          //pop from focal  
            // std::cout<<"node visited "<<focalNode->t<<" ";
            // focalNode->v->print();
            // std::cout<<" "<<checkValid(focalNode)<<"\n";                                                
            // if(focalNode->v->id==2400){
            //     throw std::runtime_error("truly found!");
            // }
            // focalNode->v->println();
            if(closed.find(focalNode->to_string())!=closed.end()) continue;  //closed
            closed.insert(focalNode->to_string());                           // add to closed
        
            if(isSolution(focalNode)){                                        //found the solution!
                goalNode=focalNode;
                fmin=fLB;
                success=true;                                                   
                break;
            }   
            auto neighbors=getSuccessors(focalNode);
            // auto neighbors=focalNode->v->neighbor;
            // neighbors.push_back(focalNode->v);                              //successors
            for(auto &new_node:neighbors){
                // int tn=focalNode->t+1;                                      // tn
                // int fn=n->manhattanDist(goal)+tn;                           //fn=gn+hn     
                // // AStarNode_p new_node=new AStarNode(n,fn,tn,0,focalNode);
                // AStarNode_p new_node=std::make_shared<AStarNode>(n,fn,tn,0,focalNode);
                if(closed.find(new_node->to_string())!=closed.end()) continue;           // not closed
                if(checkValid(new_node)==false) continue;                 // check if it satisfies the constraints
                int focalH=computeFocal(new_node);                             //compute the number of conflicts
                new_node->focalH=focalH;                                                //focalH
                open.push(new_node);                                          //push to the open
                if(new_node->f<=w*fLB) focal.push(new_node);
            }
            
        }
        if(success==false){
            // std::cout<<"failed"<<std::endl;
            fLB=-1;
            result={};
        }
        else{
            //extract the path
            // std::cout<<"found! number of expansions="<<numberOfExpansions<<std::endl;
            AStarEpsNode_p current=goalNode;
            result.clear();
            while(true){
                result.push_back(current->v);
                if(current==start_node) break;
                current=current->parent;
            }
            std::reverse(result.begin(),result.end());
        }
        return result;
    }


    // void initFunctions(){
    //     int max_constraint_time=-1;

    //     for(auto const &c: m_constraints){
    //         max_constraint_time=std::max(max_constraint_time, c->t);
    //     }
    //     compareOpen=[&](AStarEpsNode_p a1, AStarEpsNode_p a2)->bool{
    //         if(a1->f!=a2->f) return a1->f> a2->f;
    //         // if(a1->t!=a2->t) return a1->t< a2->t;
    //         return false;
    //     };
    //     compareFocal=[&](AStarEpsNode_p a1,AStarEpsNode_p a2)->bool{
    //         if(a1->focalH!=a2->focalH) return a1->focalH > a2->focalH;
    //         if(a1->f!=a2->f) return a1->f >a2->f;
    //         if(a1->t!=a2->t) return a1->t < a2->t;
    //         return false;
    //     };
    //     isSolution=[&](AStarEpsNode_p n)->bool{
    //         // std::cout<<"max_constraint_time= "<<max_constraint_time<<std::endl;
    //         return n->v==this->m_goal &&n->t >=max_constraint_time;
    //     };

    //     checkValid=[&](AStarEpsNode_p n2)->bool{  //n1---parent     n2----children
    //         // assert(constraints!=nullptr);
    //         // if(n2->v==c->v)
    //         for(auto &c:m_constraints){
    //             if(c->u==nullptr){
    //                 //vertex constraints
                
    //                 if(c->t==n2->t && c->v==n2->v &&c->agent_id==m_id) return false;
    //             }else{
    //                 //edge constraints
    //                 auto n1=n2->parent;
    //                 if(n1==nullptr) return true;
    //                 if(c->t==n2->t&& c->u==n1->v &&c->v==n2->v &&c->agent_id==m_id) return false;
    //             }
    //         }
    //         return true;
    //     };

    //     getSuccessors=[&](AStarEpsNode_p n){
    //         AStarEpsNodes children;
    //         auto neighbors=m_grids->getNeighbors(n->v);
    //         neighbors.push_back(n->v);
    //         for(auto &vn:neighbors){
    //             int tn=n->t+1;                                      // tn
    //             int fn=n->v->manhattan_dist(m_goal)+tn;                           //fn=gn+hn     
    //             AStarEpsNode_p new_node=std::make_shared<AStarEpsNode>(vn,fn,tn,0,n);
    //             children.push_back(new_node);
    //         }
    //         return children;    
    //     };




    //     //   computeFocal=[&](AStarNode_p n2)->int{      //n1--->parent    n2--> children
        
    //     //     int num_conflicts=0;
    //     //     if(paths==nullptr) return num_conflicts;
    //     //     auto n1=n2->parent;
    //     //     if(reservationTable.size()==0) return 0;
    //     //     if(n2->t<reservationTable.size()){
    //     //         //vertex
    //     //         if(reservationTable[n2->t][n2->v->id]!=-1) num_conflicts++;
    //     //         //edge
    //     //         if(reservationTable[n1->t][n2->v->id]!=-1 and 
    //     //             reservationTable[n2->t][n1->v->id]!=-1){
    //     //                 if(reservationTable[n1->t][n2->v->id]==reservationTable[n2->t][n1->v->id]) num_conflicts++;
    //     //             }
                
    //     //     }else{
    //     //         if(reservationTable.back()[n2->v->id]!=-1) num_conflicts++;
    //     //     }
        
    //     //     return num_conflicts;
    //     // };


    //     //to do change it to path table for saving more time
    //     computeFocal=[&](AStarEpsNode_p n2)->int{      //n1--->parent    n2--> children
        
    //         int num_conflicts=0;
    //         auto n1=n2->parent;
    //         int numAgents=paths.size();
    //         for(int i=0;i<numAgents;i++){
    //             if(i==m_id) continue;
    //             int li=paths[i].size()-1;
    //             if(li<0) continue;
    //             int ti=std::min(n2->t,li);
    //             int tj=std::min(n1->t,li);
    //             if(paths[i][ti]==n2->v) num_conflicts++;     //vertex conflicts
    //             else{
    //                 if(n2->t>li) continue;
    //                 if(paths[i][tj]==n2->v &&paths[i][ti]==n1->v) num_conflicts++; //edge conflicts
    //             }
                
    //         }
    //         return num_conflicts;
    //     };
    // }
    std::function<bool(AStarEpsNode_p,AStarEpsNode_p)>compareOpen;
    std::function<bool(AStarEpsNode_p,AStarEpsNode_p)>compareFocal;
    std::function<bool(AStarEpsNode_p)>isSolution;                      //check if it is solition
    std::function<bool(AStarEpsNode_p)>computeFocal;        // used to compute the focal heuristic
    std::function<bool(AStarEpsNode_p)>checkValid;    // check if a state satisfies the constraints
    std::function<AStarEpsNodes(AStarEpsNode_p)> getSuccessors;           //get successors
    using openList=std::priority_queue<AStarEpsNode,AStarEpsNodes,decltype(compareOpen)> ;
    using focalList=std::priority_queue<AStarEpsNode,AStarEpsNodes,decltype(compareFocal)> ;

private:

    Location *m_goal;
    Grids *m_grids;
    int m_id;
    // std::vector<std::vector<int>>pathTable;
    // void getTable();

};