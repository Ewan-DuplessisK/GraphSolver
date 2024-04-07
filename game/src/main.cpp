#include <iostream>
#include <vector>
#include <algorithm>
#include <array>
#include <string>
#include "raylib.h"
#include <ctime>

#include "raymath.h"

enum TerrainType{
    NORMAL,
    CHALLENGING,
    DIFFICULT,
    OBSTACLE
};

TerrainType next(TerrainType t){
    if(t==NORMAL)return CHALLENGING;
    if(t==CHALLENGING)return DIFFICULT;
    if(t==DIFFICULT)return OBSTACLE;
    if(t==OBSTACLE)return NORMAL;
}

struct DijkstraAnswer{
    std::vector<int> distance;
    std::vector<int> previous;
};

struct AStarNode{
    float gCost=0;
    float hCost=0;
    float fCost=0;
    AStarNode* parent=nullptr;
    Vector2 pos;
    TerrainType terrain=NORMAL;
    bool isPath = false;
};

std::vector<std::vector<AStarNode>> graph;
Vector2 start{0,0};
Vector2 end{0,0};

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

void resetCosts(){
    for(int i=0;i<graph.size();i++){
        for(int j=0;j<graph.at(0).size();j++){
            AStarNode* node = &graph.at(i).at(j);
            node->fCost=0;
            node->hCost=0;
            node->gCost=0;
            node->parent=nullptr;
        }
    }
}

bool aStarAnswer(const Vector2 origin,const Vector2 goal,std::vector<std::vector<AStarNode>>* graph){
    resetCosts();
    std::vector<AStarNode*> openList;
    std::vector<AStarNode*> closedList;
    graph->at(origin.x).at(origin.y).fCost=0;
    openList.push_back(&graph->at(origin.x).at(origin.y));
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

        if(Vector2Equals(currNode->pos,goal)){
            graph->at(goal.x).at(goal.y).isPath=true;
            std::vector<Vector2> path;
            AStarNode* step = currNode;
            do{
                step = step->parent;
                step->isPath=true;
            }while(step->parent!=nullptr);
            return true;
        }else{
            for (int i = -1; i < 2; ++i){
                for (int j = -1; j < 2; ++j){
                    if(i!=0||j!=0){
                        Vector2 childPos{currNode->pos.x+i,currNode->pos.y+j};
                        if(childPos.x>=0 && childPos.x<graph->size() && childPos.y>=0 && childPos.y<graph->at(0).size()){// if child in graph
                            AStarNode* child = new AStarNode;
                            float tmpGCost;
                            float tmpHCost;
                            float tmpFCost;
                            child->parent = currNode;
                            child->pos=childPos;
                            child->terrain=graph->at(childPos.x).at(childPos.y).terrain;
                            if(std::find(closedList.begin(),closedList.end(),&graph->at(childPos.x).at(childPos.y))==closedList.end()&& graph->at(childPos.x).at(childPos.y).terrain!=OBSTACLE){// if actual child not in closedList and not an obstacle
                                tmpGCost=currNode->gCost + ((i==0||j==0?10:14)*((child->terrain==NORMAL)?1:(child->terrain==CHALLENGING)?1.5f:2.0f));
                                const AStarNode* goalNode =&graph->at(goal.x).at(goal.y);
                                tmpHCost = (abs(goalNode->pos.x-childPos.x)+abs(goalNode->pos.y-childPos.y))*10;
                                tmpFCost = tmpGCost+tmpHCost;
                                if(std::find(openList.begin(),openList.end(),&graph->at(childPos.x).at(childPos.y))!=openList.end()){// if actual child is in openList
                                    if(graph->at(childPos.x).at(childPos.y).gCost>tmpGCost){// if recalculated cost is lower
                                        graph->at(childPos.x).at(childPos.y).parent=currNode;
                                        graph->at(childPos.x).at(childPos.y).gCost=tmpGCost;
                                        graph->at(childPos.x).at(childPos.y).hCost=tmpHCost;
                                        graph->at(childPos.x).at(childPos.y).fCost=tmpFCost;
                                    }
                                }else{// if not in openList //the calculated cost is used.
                                    graph->at(childPos.x).at(childPos.y).parent=currNode;
                                    graph->at(childPos.x).at(childPos.y).gCost=tmpGCost;
                                    graph->at(childPos.x).at(childPos.y).hCost=tmpHCost;
                                    graph->at(childPos.x).at(childPos.y).fCost=tmpFCost;
                                    openList.push_back(&graph->at(childPos.x).at(childPos.y));
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

std::vector<std::vector<AStarNode>> generateGraph(int sizeX,int sizeY){
    SetRandomSeed(std::time(nullptr));
    std::vector<std::vector<AStarNode>> graph;
    for(int i=0;i<sizeX;i++){
        std::vector<AStarNode> tmp;
        for(int j=0;j<sizeY;j++){
            AStarNode node;
            node.pos = Vector2{float(i),float(j)};
            if(const int randType = GetRandomValue(0,11); randType<4){
                node.terrain=NORMAL;
            }else if(randType<7){
                node.terrain=CHALLENGING;
            }else if(randType<9){
                node.terrain=DIFFICULT;
            }else node.terrain=OBSTACLE;
            if(i==0&&j==0)node.terrain=NORMAL;
            tmp.push_back(node);
        }
        graph.push_back(tmp);
    } 
return graph;
}

void drawGraph(std::vector<std::vector<AStarNode>> graph,Vector2 start,Vector2 end){
    for (int i = 0; i < graph.size(); ++i){
        for (int j = 0; j < graph.at(0).size(); ++j){
            Rectangle rec;
            rec.height=20.0f;
            rec.width=20.0f;
            rec.x=i*20.0f;
            rec.y=j*20.0f;
            
            switch (graph.at(i).at(j).terrain){
                case NORMAL:DrawRectangleRec(rec,GREEN);break;
                case CHALLENGING:DrawRectangleRec(rec,ORANGE);break;
                case DIFFICULT:DrawRectangleRec(rec,RED);break;
                case OBSTACLE:DrawRectangleRec(rec,BLACK);break;
            }
            if(graph.at(i).at(j).isPath)
                DrawCircle(i*20.0f+10.0f,j*20.0f+10.0f,5.0f,BLACK);
            DrawCircle(start.x*20.0f+10.0f,start.y*20.0f+10.0f,5.0f,DARKGREEN);
            if(!Vector2Equals(end,Vector2{0,0}))DrawCircle(end.x*20.0f+10.0f,end.y*20.0f+10.0f,5.0f,BLUE);
        }
    }
}

void handleLeftClick(){
    for(auto line:graph){//greedy clicked node check
        for(auto node:line){
            if(CheckCollisionPointRec(GetMousePosition(),Rectangle{node.pos.x*20.0f,node.pos.y*20.0f,19.5f,19.5f})){//19.5 instead of 20 insure no overlap at 2.5% size cost
                start=Vector2{end};
                end=Vector2{node.pos};
            }
        }
    }
    aStarAnswer(start,end,&graph);
    drawGraph(graph,start,end);
}

void handleRightClick(){
    for(int i=0;i<graph.size();i++){
        for(int j=0;j<graph.at(0).size();j++){
            AStarNode* node = &graph.at(i).at(j);
            if(CheckCollisionPointRec(GetMousePosition(),Rectangle{node->pos.x*20.0f,node->pos.y*20.0f,19.5f,19.5f})){
                node->terrain=next(node->terrain);
            }
        }
    }
    drawGraph(graph,start,end);
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
    graph=generateGraph(25,25);
    drawGraph(graph,start,end);
    while (!WindowShouldClose()){
        BeginDrawing();
        std::cout << IsMouseButtonDown(MOUSE_BUTTON_LEFT)<<std::endl;
        if(IsMouseButtonReleased(MOUSE_BUTTON_LEFT)){
            handleLeftClick();
        }else if(IsMouseButtonReleased(MOUSE_BUTTON_RIGHT))handleRightClick(); //TODO change terrain difficulty on rightclick
        else drawGraph(graph,start,end);
        
        EndDrawing();
    }
    
    return 0;
}
