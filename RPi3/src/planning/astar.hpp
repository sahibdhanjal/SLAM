#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
    SearchParams() : minDistanceToObstacle(0.1){}
};

class Pose{
public:
    double f;
    double g;
    int x;
    int y;
    int prev_x;
    int prev_y;
    bool isVisited;
    bool isClosed;

    Pose(void);
    Pose operator=(const Pose& p){
        f = p.f;
        g = p.g;
        x = p.x;
        y = p.y;
        prev_x = p.prev_x;
        prev_y = p.prev_y;
        isVisited = p.isVisited;
        isClosed = p.isClosed;
    };
    bool operator==(const Pose& p){
        return (x == p.x) && (y == p.y);
    };
};

class fCompare
{
public:
    bool operator() (const Pose* a, const Pose* b)
    {
        return a->f > b->f;
    }
};

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);
float heuristic(Pose curr, Pose goal);

robot_path_t reconstruct_path(std::vector<std::vector<Pose>>& poseMap, Pose last, Pose start, const ObstacleDistanceGrid& distances);

#endif // PLANNING_ASTAR_HPP
