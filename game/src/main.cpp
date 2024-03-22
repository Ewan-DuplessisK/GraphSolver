#include <iostream>
#include <vector>
#include <algorithm>
#include <array>
#include <string>
#include "raylib.h"

enum TerrainType{
    NORMAL,
    CHALLENGING,
    DIFFICULT,
    OBSTACLE
};

struct DijkstraAnswer{
    std::vector<int> distance;
    std::vector<int> previous;
};

struct AStarNode{
    float gCost=0;
    float hCost=0;
    float fCost=0;
    AStarNode* parent=nullptr;
    std::array<int,2> pos;
    TerrainType terrain=NORMAL;
    bool isPath = false;
};

DijkstraAnswer solveDijkstra(const int origin, const std::vector<std::vector<int>>* graph){
    std::vector<int> distance;
    std::vector<int> previous;
    std::vector<int> set;
    for(int i=0;i<graph->size();i++){
        distance.emplace_back(INT_MAX);
        previous.emplace_back(-1);
        set.emplace_back(i);
    }
    distance[origin]=0;
    int index = -1;
    while(!set.empty()){
        int temp = INT_MAX;
        for(int value:set){
            if(distance.at(value)<temp){
                temp=distance.at(value);
                index = value;
            }
        }
        set.erase(std::find(set.begin(),set.end(),index));
        for(int i=0;i<graph->size();i++){
            if(graph->at(index).at(i)!=INT_MAX &&graph->at(index).at(i)!=0){
                if(std::find(set.begin(), set.end(), i) != set.end()){
                    const int tmp = distance.at(index)+graph->at(index).at(i);
                    if(tmp<distance.at(i)){
                        distance.at(i)=tmp;
                        previous.at(i)=index;
                    }
                }
            }
        }
    }
    return DijkstraAnswer{distance,previous};
}

bool aStarAnswer(const std::array<int,2>origin,const std::array<int,2>goal,std::vector<std::vector<AStarNode>>* graph){
    std::vector<AStarNode*> openList;
    std::vector<AStarNode*> closedList;
    graph->at(origin.at(0)).at(origin.at(1)).fCost=0;
    openList.push_back(&graph->at(origin.at(0)).at(origin.at(1)));
    int count=0;
    while (!openList.empty()){
        AStarNode* currNode=openList.at(0);
        for(int i = 0; i < openList.size(); i++){
            if(openList.at(i)->fCost<currNode->fCost){
                currNode=openList.at(i);
            }
        }
        closedList.push_back(currNode);
        openList.erase(std::remove(openList.begin(), openList.end(), currNode), openList.end());

        if(currNode->pos==goal){
            graph->at(goal.at(0)).at(goal.at(1)).isPath=true;
            //std::string path = "["+std::to_string(currNode->pos.at(0))+";"+std::to_string(currNode->pos.at(1))+"]";
            std::vector<std::array<int,2>> path;
            AStarNode* step = currNode;
            do{
                step = step->parent;
                step->isPath=true;
                //path = "["+std::to_string(step->pos.at(0))+";"+std::to_string(step->pos.at(1))+"] - "+path;
            }while(step->parent!=nullptr);
            //path+=" "+std::to_string(currNode->gCost);
            return true;
        }else{
            for (int i = -1; i < 2; ++i){
                for (int j = -1; j < 2; ++j){
                    if(i!=0||j!=0){
                        std::array<int,2> childPos{currNode->pos.at(0)+i,currNode->pos.at(1)+j};
                        if(childPos.at(0)>=0 && childPos.at(0)<graph->size() && childPos.at(1)>=0 && childPos.at(1)<graph->at(0).size()){// if child in graph
                            AStarNode* child = new AStarNode;
                            float tmpGCost;
                            float tmpHCost;
                            float tmpFCost;
                            child->parent = currNode;
                            child->pos=childPos;
                            child->terrain=graph->at(childPos.at(0)).at(childPos.at(1)).terrain;
                            if(std::find(closedList.begin(),closedList.end(),&graph->at(childPos.at(0)).at(childPos.at(1)))==closedList.end()&& graph->at(childPos.at(0)).at(childPos.at(1)).terrain!=OBSTACLE){// if actual child not in closedList and not an obstacle
                                tmpGCost=currNode->gCost + ((i==0||j==0?10:14)*((child->terrain==NORMAL)?1:(child->terrain==CHALLENGING)?1.5f:2.0f));
                                const AStarNode* goalNode =&graph->at(goal.at(0)).at(goal.at(1));
                                tmpHCost = (abs(goalNode->pos.at(0)-childPos.at(0))+abs(goalNode->pos.at(1)-childPos.at(1)))*10;
                                tmpFCost = tmpGCost+tmpHCost;
                                if(std::find(openList.begin(),openList.end(),&graph->at(childPos.at(0)).at(childPos.at(1)))!=openList.end()){// if actual child is in openList
                                    if(graph->at(childPos.at(0)).at(childPos.at(1)).gCost>tmpGCost){// if recalculated cost is lower
                                        graph->at(childPos.at(0)).at(childPos.at(1)).parent=currNode;
                                        graph->at(childPos.at(0)).at(childPos.at(1)).gCost=tmpGCost;
                                        graph->at(childPos.at(0)).at(childPos.at(1)).hCost=tmpHCost;
                                        graph->at(childPos.at(0)).at(childPos.at(1)).fCost=tmpFCost;
                                    }
                                }else{// if not in openList //the calculated cost is used.
                                    graph->at(childPos.at(0)).at(childPos.at(1)).parent=currNode;
                                    graph->at(childPos.at(0)).at(childPos.at(1)).gCost=tmpGCost;
                                    graph->at(childPos.at(0)).at(childPos.at(1)).hCost=tmpHCost;
                                    graph->at(childPos.at(0)).at(childPos.at(1)).fCost=tmpFCost;
                                    openList.push_back(&graph->at(childPos.at(0)).at(childPos.at(1)));
                                }
                            }
                            
                        }
                    }
                }
            }
        }
    }
    return false;
    
}

int main(int argc, char* argv[]){

    InitWindow(500,500,"A Star");
    
    /*std::vector<std::vector<int>> graph = {
        std::vector<int>{0, 10, 15, INT_MAX, 30, INT_MAX, INT_MAX},
        std::vector<int>{INT_MAX, 0, INT_MAX, INT_MAX, INT_MAX, 57, INT_MAX},
        std::vector<int>{15, INT_MAX, 0, 16, INT_MAX, INT_MAX, 52},
        std::vector<int>{INT_MAX, INT_MAX, 13, 0, INT_MAX, INT_MAX, INT_MAX},
        std::vector<int>{30, INT_MAX, INT_MAX, INT_MAX, 0, 11, 34},
        std::vector<int>{INT_MAX, 49, INT_MAX, INT_MAX, 12, 0, INT_MAX},
        std::vector<int>{INT_MAX, INT_MAX, 63, INT_MAX, 35, INT_MAX, 0}
    };
    DijkstraAnswer ans = solveDijkstra(2,&graph);*/
    std::vector<std::vector<AStarNode>> graph;
    for (int i = 0; i < 5; ++i){
        std::vector<AStarNode> tmp;
        for (int j = 0; j < 5; ++j){
            AStarNode node;
            node.pos = std::array<int,2>{i,j};
            if(i==1)node.terrain=CHALLENGING;
            if(i==3&&j==3)node.terrain=OBSTACLE;
            tmp.push_back(node);
        }
        graph.push_back(tmp);
    }
    while (!WindowShouldClose()){
        BeginDrawing();
        if(aStarAnswer(std::array<int,2>{0,0},std::array<int,2>{4,4},&graph)){
            for (int i = 0; i < graph.size(); ++i){
                for (int j = 0; j < graph.at(0).size(); ++j){
                    std::cout << graph.at(i).at(j).isPath<<std::endl;
                    Rectangle rec;
                    rec.height=100.0f;
                    rec.width=100.0f;
                    rec.x=i*100.0f;
                    rec.y=j*100.0f;
                    if(graph.at(i).at(j).isPath){
                        DrawRectangleRec(rec,BLACK);
                    }else{
                        switch (graph.at(i).at(j).terrain){
                        case NORMAL:DrawRectangleRec(rec,GREEN);break;
                        case CHALLENGING:DrawRectangleRec(rec,BLUE);break;
                        case DIFFICULT:DrawRectangleRec(rec,BROWN);break;
                        case OBSTACLE:DrawRectangleRec(rec,RED);break;
                        }
                    }
                }
            }
        }
        std::cout << "break"<<std::endl<<std::endl;
        EndDrawing();
    }
    
    return 0;
}
