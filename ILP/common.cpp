/**
 * @file common.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-02-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "common.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/maximum_weighted_matching.hpp>


///////////////////////////////////////////////////////////////////// util functions //////////////////////////////////////////////////

/**
 * @brief
 *
 * @param file_name
 * @param starts
 * @param goals
 * @param graph
 */

/**
 * @brief Construct a new Grids:: Grids object
 * 
 * @param file_name 
 */
Grids::Grids(std::string file_name)
{
    std::string line;
    std::smatch results;
    std::regex r_height = std::regex(R"(height\s(\d+))");
    std::regex r_width = std::regex(R"(width\s(\d+))");
    std::regex r_map = std::regex(R"(map)");
    std::ifstream map_file(file_name);
    // fundamental graph params
    while (getline(map_file, line))
    {
        // for CRLF coding
        if (*(line.end() - 1) == 0x0d)
            line.pop_back();

        if (std::regex_match(line, results, r_height)){
            ymax = std::stoi(results[1].str());
        }
        if (std::regex_match(line, results, r_width)){
            xmax = std::stoi(results[1].str());
        }
        if (std::regex_match(line, results, r_map))
            break;
    }
    if (!(xmax> 0 && ymax > 0))
        throw std::runtime_error("incorrect graph size!");

    // create nodes
    nodes = Configs(xmax*ymax, nullptr);
    int y=0;
    while (getline(map_file, line))
    {
        // for CRLF coding
        if (*(line.end() - 1) == 0x0d)
            line.pop_back();

        if ((int)line.size() != ymax)
            throw std::runtime_error("incorrect graph size!");
        for (int x = 0; x < xmax; ++x)
        {
            char s = line[x];
            if (s == 'T' or s == '@')
                continue; // object
            int id = xmax * y + x;
            Location *v = new Location(id, x, y);
            // std::cout<<*v<<std::endl;
            nodes[id] = v;
        }
        ++y;
    }
}

Grids::Grids(int xmax,int ymax){
    this->xmax=xmax;
    this->ymax=ymax;
    nodes = Configs(xmax*ymax, nullptr);
    for(int x=0;x<xmax;x++){
        for(int y=0;y<ymax;y++){
            int id=xmax*y+x;
            nodes[id]=new Location(id,x,y);
        }
    }
}

Grids::Grids(int xmax,int ymax,std::vector<std::vector<int>> &obstacles){
    Grids(xmax,ymax);
    for(auto &obs:obstacles){
        int id=xmax*obs[1]+obs[0];
        nodes[id]=nullptr;
    }
}

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @return Location* 
 */
Location* Grids::getVertex(int x,int y){
    return nodes[x+xmax*y];
}


/**
 * @brief 
 * 
 * @param id 
 * @return Location* 
 */
Location *Grids::getVertex(int id){
    return nodes[id];
}


/**
 * @brief 
 * 
 * @param node 
 * @return Configs 
 */
Configs Grids::getNeighbors(Location *node){
    Configs neighbors;
    if(node->x-1 >=0 ){
        auto nc=getVertex(node->x-1,node->y);
        if(nc!=nullptr) neighbors.push_back(nc);
    } 
    if(node->y-1>=0) {
        auto nc=getVertex(node->x,node->y-1);
        if(nc!=nullptr) neighbors.push_back(nc);
    }
    if(node->y+1<ymax) {
        auto nc=getVertex(node->x,node->y+1);
        if(nc!=nullptr) neighbors.push_back(nc);
    }
    if(node->x+1<xmax){
        auto nc=getVertex(node->x+1,node->y);
        if(nc!=nullptr) neighbors.push_back(nc);
    }
    return neighbors;
}




////////////////////////////////////////////////////////////////////////////


/**
 * @brief 
 * 
 * @param file_name 
 * @param starts 
 * @param goals 
 * @param graph 
 */
void read_scen(std::string file_name,Configs&starts,Configs &goals,Grids *graph){
    starts.clear();
    goals.clear();
    // int xmax=-1,ymax=-1,zmax=-1;
    std::string line;
    std::smatch results;
    std::ifstream scen_file(file_name);
    // std::regex r_xmax=std::regex(R"(xmax=(\d+))");
    // std::regex r_ymax=std::regex(R"(ymax=(\d+))");
    // std::regex r_zmax=std::regex(R"(zmax=(\d+))");
    std::regex r_sg=std::regex(R"((\d+),(\d+),(\d+),(\d+))");
    if(!scen_file){
        std::cout<<"File not found! file name="<<file_name<<std::endl;
        exit(0);
    }
    int id=0;
    while(getline(scen_file,line)){
        //CRLF
        if(*(line.end()-1)==0x0d) line.pop_back();
        
        if(std::regex_match(line,results,r_sg)){
            
            int x_s=std::stoi(results[1].str());
            int y_s=std::stoi(results[2].str());
            int x_g=std::stoi(results[3].str());
            int y_g=std::stoi(results[4].str());
            starts.emplace_back(graph->getVertex(x_s,y_s));
            goals.emplace_back(graph->getVertex(x_g,y_g));
            // Robot *ri=new Robot(id,graph->getVertex(x_s,y_s,z_s),graph->getVertex(x_g,y_g,z_g));
            id++;
            continue;
        } 
    } 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 
 * 
 * @param file_name 
 * @param paths 
 * @param runtime 
 * @param save_paths 
 */
void save_solutions(std::string file_name,Paths&paths,double runtime,bool save_paths){
    std::ofstream out(file_name);
    int makespanLB,socLB,makespan,soc;
    evaluate_result(paths,makespan,makespanLB,soc,socLB);
    out<<"soc="<<soc<<std::endl;
    out<<"lb_soc="<<socLB<<std::endl;
    out<<"makespan="<<makespan<<std::endl;
    out<<"lb_makespan="<<makespanLB<<std::endl;
    out<<"comp_time="<<(int)(runtime*1000)<<std::endl;
    if(save_paths==false) return;
    out<<"solutions="<<std::endl;
    for(int i=0;i<paths.size();i++){
        out<<i<<":";
        for(const auto &v:paths[i]){
            out<<'('<<v->x<<","<<v->y<<"),";
        }
        out<<std::endl;
    }
}



/**
 * @brief 
 * 
 * @param paths 
 */
void check_feasible_bruteForce(Paths & paths){
    int makespan=0;
    for(auto &p:paths){
        makespan=std::max(makespan,(int)p.size());
    }

    fill_paths(paths);

    for(int t=1;t<makespan;t++){
        for(int i=0;i<paths.size();i++){
            for(int j=i+1;j<paths.size();j++){
                if(paths[i][t]==paths[j][t]) {
                    printf("Vertex collision (%d,%d,%d,%d)\n",i,j,t,paths[i][t]->id);
                    std::cout<<paths[i][t]->print()<<std::endl;
                    std::cout<<paths[j][t]->print()<<std::endl;
                    assert(false);
                }
                if(paths[i][t-1]==paths[j][t] and paths[i][t]==paths[j][t-1]){
                    printf("Edge collision (%d,%d,%d, %d->%d)\n",i,j,t,paths[i][t-1]->id,paths[i][t]->id);
                    assert(false);
                }
            }
        }
    }
    std::cout<<"no collision!"<<std::endl;
}


/**     espan=std::max((int)p.size(),makespan);
    }
    for(auto &p:paths){
        while(p.size()<makespan) p.emplace_back(p.back());
    }
}


/**
 * @brief 
 * 
 * @param paths 
 * @param makespan 
 * @param makespanLB 
 * @param soc 
 * @param socLB 
 */
void evaluate_result(Paths &paths,int &makespan,int &makespanLB,int &soc,int &socLB){
    makespan=makespanLB=soc=socLB=0;
    for(auto &p:paths){
        makespan=std::max(makespan,(int)p.size());
        soc+=p.size()-1;
        socLB+=p[0]->manhattan_dist(p.back());
        makespanLB=std::max(makespanLB,p[0]->manhattan_dist(p.back()));
    }
}   

/**
 * @brief 
 * 
 * @param paths 
 */
void shrink_paths(Paths&paths){
    for(auto &p:paths){
        if(p.size()<=1) continue;
        while(p.back()==p.end()[-2]) p.pop_back();
    }
}


/**
 * @brief 
 * 
 * @param paths 
 * @param runtime 
 * @param save_paths 
 */
void save_result_as_json(std::string file_name,Paths &paths,double runtime,bool save_paths){
    nlohmann::json data_json;

    int makespan,makespanLB,soc,socLB;
    evaluate_result(paths,makespan,makespanLB,soc,socLB);
    data_json["makespan"]=makespan;
    data_json["makespanLB"]=makespanLB;
    data_json["runtime"]=runtime;
    data_json["soc"]=soc;
    data_json["socLB"]=socLB;
    if(save_paths){
        std::vector<std::vector<std::vector<int>>> paths_data(paths.size(),std::vector<std::vector<int>>());
        for(int i=0;i<paths.size();i++){
            for (auto v:paths[i]){
                paths_data[i].push_back({v->x,v->y});
            }
        }
        nlohmann::json paths_json(paths_data);
        data_json["paths"]=paths_json;
        
    }

    std::ofstream file;
    file.open(file_name);
    // std::cout<<data_json["tasks"].size()<<std::endl;
    std::cout<<"save as  "<<file_name<<std::endl;
    file<< data_json<<std::endl; 
}

void format_paths(Paths &paths){
    for(auto &p:paths){
        while(p.size()>=2 and p.back()==p[p.size()-2]){
            p.pop_back();
        }
        if(p.size()>=2) assert(p.back()!=p[p.size()-2]);
    }
}

void evaluate_result2(Paths & paths,double &makespan,double &makespanLB,double &sod,double &sor,int num_retrieval){
    makespan=makespanLB=sod=sor=0;
    format_paths(paths);
    std::cout<<"evaluate_result2  "<<num_retrieval<<std::endl;
    int i=0;
    for(auto &p:paths){
        makespan=std::max(makespan,(double)p.size());
        makespanLB=std::max(makespanLB,(double)p[0]->manhattan_dist(p.back()));
        double sodi=0;
        auto last_v=p[0];
        for(auto u:p){
            if(last_v!=u) sodi+=1;
            last_v=u;
        }
        sod+=sodi;
        if(i<num_retrieval){
            sor+=p.size();
        }
        i++;
    }

    
}

/**
 * @brief 
 * 
 * @param makespan 
 */
void fill_paths(Paths &paths,int makespan){
    if(makespan==-1){
        for(auto &p:paths){
            makespan=std::max((int)p.size(),makespan);
        }
    }
    for(auto &p:paths){
        while(p.size()<makespan){
            p.push_back(p.back());
        }
    }
}


/**
 * @brief 
 * 
 * @param starts 
 * @param goals 
 * @param assigned_goal 
 */
void linear_sum_assignment(const Configs &starts,const Configs &goals,Configs &assigned_goal){
    assert(starts.size()<=goals.size());
    using namespace boost;
    using EdgeProperty=property< edge_weight_t, float, property< edge_index_t, int > >;
    using my_graph=adjacency_list< vecS, vecS, undirectedS, no_property, EdgeProperty >;
    graph_traits< my_graph >::vertex_iterator vi, vi_end;
    int num_starts=starts.size();
    int num_goals=goals.size();
    
    const int n_vertices=num_starts+num_goals;
    my_graph graph(n_vertices);
    //add edges
    
    for(int i=0;i<num_starts;i++){
        for(int j=0;j<num_goals;j++){
            int dist=900-starts[i]->manhattan_dist(goals[j]);  //minimum sum of distance
            add_edge(i,j+num_starts,EdgeProperty(dist),graph);
        }
    }
    std::vector< graph_traits< my_graph >::vertex_descriptor > mate1(n_vertices);
    maximum_weighted_matching(graph, &mate1[0]);
    assigned_goal=Configs(num_starts);
    for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi){
        if (mate1[*vi] != graph_traits< my_graph >::null_vertex()&& *vi < mate1[*vi]){
            assigned_goal[*vi]=goals[(mate1[*vi]-num_starts)];
            // std::cout<<"DEBUG:  " << "{" << *vi << ", " << mate1[*vi] << "}" << std::endl;
        }
    }
    // std::cout<<"DEBUG wight sum: "<<matching_weight_sum(graph, &mate1[0])<<std::endl;
}

void save_data_as_csv(std::string file_name,std::vector<double> & data1,std::vector<double> &data2){
    std::ofstream my_file(file_name);
    for(int i=0;i<data1.size();i++){
        my_file<<data1[i]<<","<<data2[i]<<std::endl;
    }
}