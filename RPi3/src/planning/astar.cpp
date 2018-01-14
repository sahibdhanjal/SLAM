#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>
#include <algorithm>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    std::priority_queue<Pose*, std::vector<Pose*>, fCompare> openSet;
    std::vector<std::vector<Pose>> poseMap;
    
    Pose initial;
    poseMap.resize(distances.widthInCells());
    for (int i = 0; i < poseMap.size(); ++i){
    	poseMap[i].resize(distances.heightInCells());
        for (int j = 0; j < poseMap.size(); ++j){
            poseMap[i][j] = initial;
            poseMap[i][j].x = i;
            poseMap[i][j].y = j;
            if (distances(i,j) <3){
                poseMap[i][j].isVisited = true;
                poseMap[i][j].isClosed = true;
            }
        }
    }

    Pose goal_pose;
    goal_pose.x = (goal.x - distances.originInGlobalFrame().x)*distances.cellsPerMeter();
    goal_pose.y = (goal.y - distances.originInGlobalFrame().y)*distances.cellsPerMeter();
    goal_pose.f = 100000;
    goal_pose.g = 100000;
    goal_pose.prev_x = 0;
    goal_pose.prev_y = 0;
    goal_pose.isVisited = false;
    goal_pose.isClosed = false;

    Pose start_pose;
    start_pose.x =  (start.x - distances.originInGlobalFrame().x)*distances.cellsPerMeter();
    start_pose.y =  (start.y - distances.originInGlobalFrame().y)*distances.cellsPerMeter();
    start_pose.f = heuristic(start_pose, goal_pose)/* + distances((start.x + distances.originInGlobalFrame().x)*distances.cellsPerMeter(), (start.y + distances.originInGlobalFrame().y)*distances.cellsPerMeter())*/;
    start_pose.g = 0;
    start_pose.prev_x = (start.x - distances.originInGlobalFrame().x)*distances.cellsPerMeter();
    start_pose.prev_y = (start.y - distances.originInGlobalFrame().y)*distances.cellsPerMeter();
    start_pose.isVisited = true;    
    goal_pose.isClosed = false;

    // std::cout << "goal: " << goal_pose.x << " " << goal_pose.y << std::endl;

    

    poseMap[start_pose.x][start_pose.y] = start_pose;
    openSet.push(&poseMap[start_pose.x][start_pose.y]);
    // std::cout << start_pose.x << std::endl;

    bool foundPath = false;
    robot_path_t path;
    while (openSet.size() != 0){
    	Pose* curr = openSet.top();
        

    	if (curr->x == goal_pose.x && curr->y == goal_pose.y) {
            // std::cout << poseMap[temp_w][temp_h].x << " " << poseMap[temp_w][temp_h].y << " " << poseMap[temp_w][temp_h].prev_x << " " << poseMap[temp_w][temp_h].prev_y << std::endl;
            path = reconstruct_path(poseMap, *curr, start_pose, distances);
            foundPath = true;
            break;
        }
    	openSet.pop();
        // std::cout << "*********** new round **********" << std::endl;
        // std::cout << curr->x << " " << curr->y << " " << curr->prev_x << " " << curr->prev_y << " " << (curr->x - distances.originInGlobalFrame().x)*distances.cellsPerMeter() << " " << (curr->y - distances.originInGlobalFrame().y)*distances.cellsPerMeter()<<std::endl;
        // std::cout << "********************************" << std::endl;
       
    	curr->isClosed = true;
        curr->isVisited = true;
        // std::cout << "x: " << curr->isClosed << " y: "<<curr->isVisited << std::endl;
    	for (int k = -1; k<=1; ++k) {
            for (int h = -1; h <= 1; ++h) {
                if (k == 0 && h == 0) continue;
                auto temp_w = curr->x + k;
                auto temp_h = curr->y + h;
                // std::cout << (curr->x - distances.originInGlobalFrame().x)*distances.cellsPerMeter()+k<<" "<<(curr->y - distances.originInGlobalFrame().y)*distances.cellsPerMeter()+h<< " " << temp_w << " " << temp_h << std::endl;
                if (distances.isCellInGrid(temp_w,temp_h)){
                    
                    Pose* neighbor = &poseMap[temp_w][temp_h];
                    
                    if (neighbor->isClosed) continue;
                    
                    double tentative_gScore = curr->g + sqrt(k*k+h*h);
                    if (tentative_gScore < neighbor->g){
                        neighbor->prev_x = curr->x;
                        neighbor->prev_y = curr->y;
                        // if (curr ->x < 5.001 && curr->x > 4.999 && curr ->y < 5.001 && curr->y > 4.999 && curr ->y < 5.001 && curr->y > 4.999) std::cout << "x: " << neighbor->prev_x << " y: "<<neighbor->prev_y << std::endl;
                        neighbor->g = tentative_gScore;
                        neighbor->f = neighbor->g + heuristic(*neighbor, goal_pose);   
                    }
                    
                    if (!neighbor->isVisited) {
                        neighbor->isVisited = true;
                        openSet.push(neighbor); 
                        // std::cout << temp_w << " " << temp_h << std::endl;
                    }
                }
                // else{
                //      std::cout << (2.55 - distances.originInGlobalFrame().x)*distances.cellsPerMeter() << std::endl;
                // }
            }
        }
    }
    
    if(foundPath){
        path.utime = start.utime;
        path.path.push_back(start);    
        path.path_length = path.path.size();
        return path;
    }
    else{
        return path;
    }


    
}

float heuristic(Pose curr, Pose goal){
	float dx = fabs(curr.x - goal.x);
	float dy = fabs(curr.y - goal.y);
	if (dx > dy) return sqrt(2.0)*dy + (dx - dy);
	else return sqrt(2.0)*dx + (dy - dx);
}

Pose::Pose(void)
: f(100000)
, g(100000)
, x(0)
, y(0)
, prev_x(0)
, prev_y(0)
, isVisited(false)
, isClosed(false){
}

robot_path_t reconstruct_path(std::vector<std::vector<Pose>>& poseMap, Pose last, Pose start, const ObstacleDistanceGrid& distances){
    FILE* f = fopen("astart.txt", "w");

    robot_path_t path;
    Pose curr = last;
    int count = 0;
    std::cout << "reconstruct_path" << std::endl;
    // std::cout << "x: " << curr.x << " y: "<< curr.y << std::endl;
    while (curr.x != start.x || curr.y != start.y){
        count++;
        if (count > 400) break;
        pose_xyt_t pose;
        pose.x = curr.x*distances.metersPerCell() + distances.originInGlobalFrame().x;
        pose.y = curr.y*distances.metersPerCell() + distances.originInGlobalFrame().y;
        pose.theta = 0;
        // std::cout << "x: " << curr.prev_x << " y: "<< curr.prev_y << std::endl;
        // std::cout << "x: " << pose.x << " y: "<< pose.y << std::endl;
        path.path.push_back(pose);

        fprintf(f, "%f, %f\n", pose.x ,pose.y);
        curr = poseMap[curr.prev_x][curr.prev_y];
        // std::cout << curr.prev_x << std::endl;
        // std::cout << (5 - distances.originInGlobalFrame().x)*distances.cellsPerMeter() << " " <<(4.95 - distances.originInGlobalFrame().x)*distances.cellsPerMeter()<< std::endl;
    }
    pose_xyt_t pose;
    pose.x = curr.x*distances.metersPerCell() + distances.originInGlobalFrame().x;
    pose.y = curr.y*distances.metersPerCell() + distances.originInGlobalFrame().y;
    pose.theta = 0;
    path.path.push_back(pose);

    std::reverse(path.path.begin(), path.path.end());
    return path;
}