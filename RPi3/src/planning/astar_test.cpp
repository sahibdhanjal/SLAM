#include <planning/motion_planner.hpp>
#include <slam/occupancy_grid.hpp>
#include <iostream>
#include <iterator>

const float kGridWidth = 15.0f;
const float kGridHeight = 15.0f;
const float kMetersPerCell = 0.05f;


bool test_empty_grid(void);
bool test_constricted_grid(void);
bool test_sparse_grid(void);
bool test_filled_grid(void);

OccupancyGrid generate_uniform_grid(int8_t logOdds);
OccupancyGrid generate_constricted_grid(double openingWidth);
OccupancyGrid generate_random_grid(double occupiedAmount);

std::ostream& operator<<(std::ostream& out, const pose_xyt_t& pose);


int main(int argc, char** argv)
{
    // if(test_empty_grid())
    // {
    //     std::cout << "PASSED: test_empty_grid\n";
    // }
    // else
    // {
    //     std::cout << "FAILED: test_empty_grid\n";
    // }
    
    if(test_constricted_grid())
    {
        std::cout << "PASSED: test_constricted_grid\n";
    }
    else
    {
        std::cout << "FAILED: test_constricted_grid\n";
    }
    
    // if(test_filled_grid())
    // {
    //     std::cout << "PASSED: test_filled_grid\n";
    // }
    // else
    // {
    //     std::cout << "FAILED: test_filled_grid\n";
    // }
    
    return 0;
}


bool test_empty_grid(void)
{
    OccupancyGrid grid = generate_uniform_grid(-10);
    MotionPlannerParams plannerParams;
    plannerParams.robotRadius = 0.1;
    
    MotionPlanner planner(plannerParams);
    planner.setMap(grid);
    
    int numTotalPaths = 0;
    int numCorrectPaths = 0;
    
    // See that a straight-line path has only two poses -- the start and end
    {
        pose_xyt_t start;
        start.x = -5.0;
        start.y = 0.0;
        
        pose_xyt_t goal;
        goal.x = 5.0;
        goal.y = 0.0;
        
        robot_path_t path = planner.planPath(start, goal);
        
        ++numTotalPaths;
        if(path.path_length == 2)
        {
            ++numCorrectPaths;
            std::cout << "Correct path for " << start << "->" << goal << '\n';
        }
        else
        {
            std::cout << "Incorrect path for " << start << "->" << goal << "\nFound path:";
        }
        
        std::copy(path.path.begin(), path.path.end(), std::ostream_iterator<pose_xyt_t>(std::cout, " "));
        std::cout << '\n';
    }

    // return false;
    // See that a straight-line path has only two poses -- the start and end
    {
        pose_xyt_t start;
        start.x = 1.0;
        start.y = 5.0;
        
        pose_xyt_t goal;
        goal.x = 1.0;
        goal.y = -5.0;
        
        robot_path_t path = planner.planPath(start, goal);
        
        ++numTotalPaths;
        if(path.path_length == 2)
        {
            ++numCorrectPaths;
            std::cout << "Correct path for " << start << "->" << goal << '\n';
        }
        else
        {
            std::cout << "Incorrect path for " << start << "->" << goal << "\nFound path:";
        }
        
        std::copy(path.path.begin(), path.path.end(), std::ostream_iterator<pose_xyt_t>(std::cout, " "));
        std::cout << '\n';
    }
    
    // See that a diagonal path has only two poses -- the start and end
    {
        pose_xyt_t start;
        start.x = 5.0;
        start.y = 5.0;
        
        pose_xyt_t goal;
        goal.x = -5.0;
        goal.y = -5.0;
        
        robot_path_t path = planner.planPath(start, goal);
        
        ++numTotalPaths;
        if(path.path_length == 2)
        {
            ++numCorrectPaths;
            std::cout << "Correct path for " << start << "->" << goal << '\n';
        }
        else
        {
            std::cout << "Incorrect path for " << start << "->" << goal << "\nFound path:";
        }
        
        std::copy(path.path.begin(), path.path.end(), std::ostream_iterator<pose_xyt_t>(std::cout, " "));
        std::cout << '\n';
    }
    
    return numTotalPaths == numCorrectPaths;
}


bool test_constricted_grid(void)
{
    MotionPlannerParams plannerParams;
    plannerParams.robotRadius = 0.075;
    
    MotionPlanner planner(plannerParams);
    
    int numTotalPaths = 0;
    int numCorrectPaths = 0;
    
    pose_xyt_t start;
    start.x = 2.0;
    start.y = 5.0;
    
    pose_xyt_t goal;
    goal.x = -1.0;
    goal.y = -4.0;
    
    // See that the search can go through a large opening
    {
        double constriction = plannerParams.robotRadius * 4;
        OccupancyGrid grid = generate_constricted_grid(constriction);
        planner.setMap(grid);
        
        robot_path_t path = planner.planPath(start, goal);
        
        ++numTotalPaths;
        
        if(path.path_length > 1)
        {
            ++numCorrectPaths;
            std::cout << "Correct path for " << start << "->" << goal << " with constriction " << constriction << '\n';
        }
        else
        {
            std::cout << "Incorrect path for " << start << "->" << goal << " with constriction " << constriction << '\n';
        }
        
        std::copy(path.path.begin(), path.path.end(), std::ostream_iterator<pose_xyt_t>(std::cout, " "));
        std::cout << '\n';
    }
    
    // // See that the search can start close to a wall
    // {
    //     pose_xyt_t collisionStart;
    //     collisionStart.x = -2.5;
    //     collisionStart.y = 0.051;
        
    //     double constriction = plannerParams.robotRadius * 4;
    //     OccupancyGrid grid = generate_constricted_grid(constriction);
    //     planner.setMap(grid);
        
    //     robot_path_t path = planner.planPath(collisionStart, goal);
        
    //     ++numTotalPaths;
        
    //     if(path.path_length > 1)
    //     {
    //         ++numCorrectPaths;
    //         std::cout << "Correct path for " << start << "->" << goal << " with constriction " << constriction 
    //             << " Starting against the wall.\n";
    //     }
    //     else
    //     {
    //         std::cout << "Incorrect path for " << start << "->" << goal << " with constriction " << constriction 
    //             << " Starting against the wall.\n";
    //     }
        
    //     std::copy(path.path.begin(), path.path.end(), std::ostream_iterator<pose_xyt_t>(std::cout, " "));
    //     std::cout << '\n';
    // }
    
    // // See that the search fails to get through a small opening
    // {
    //     double constriction = plannerParams.robotRadius - kMetersPerCell;
    //     OccupancyGrid grid = generate_constricted_grid(constriction);
    //     planner.setMap(grid);
        
    //     robot_path_t path = planner.planPath(start, goal);
        
    //     ++numTotalPaths;
        
    //     if(path.path_length == 1)
    //     {
    //         ++numCorrectPaths;
    //         std::cout << "Correctly found no path for " << start << "->" << goal << " with constriction " 
    //             << constriction << '\n';
    //     }
    //     else
    //     {
    //         std::cout << "Incorrect found a path for " << start << "->" << goal << " with constriction " 
    //             << constriction << '\n';
    //     }
        
    //     std::copy(path.path.begin(), path.path.end(), std::ostream_iterator<pose_xyt_t>(std::cout, " "));
    //     std::cout << '\n';
    // }
    
    return numCorrectPaths == numTotalPaths;
}


bool test_sparse_grid(void)
{
    int numTotalPaths = 0;
    int numCorrectPaths = 0;
    
    return numTotalPaths == numCorrectPaths;
}


bool test_filled_grid(void)
{
    OccupancyGrid grid = generate_uniform_grid(10);
    MotionPlannerParams plannerParams;
    plannerParams.robotRadius = 0.1;
    
    MotionPlanner planner(plannerParams);
    planner.setMap(grid);
    
    int numTotalPaths = 0;
    int numCorrectPaths = 0;
    
    // Ensure that no path exists in a filled grid
    {
        pose_xyt_t start;
        start.x = 1.0;
        start.y = 5.0;
        
        pose_xyt_t goal;
        goal.x = 1.0;
        goal.y = -5.0;
        
        robot_path_t path = planner.planPath(start, goal);
        
        ++numTotalPaths;
        if(path.path_length == 1)
        {
            ++numCorrectPaths;
            std::cout << "Correctly found no path for " << start << "->" << goal << '\n';
        }
        else
        {
            std::cout << "Incorrect path for " << start << "->" << goal << "\nFound path:";
        }
        
        std::copy(path.path.begin(), path.path.end(), std::ostream_iterator<pose_xyt_t>(std::cout, " "));
        std::cout << '\n';
    }
    
    return numTotalPaths == numCorrectPaths;
}


OccupancyGrid generate_uniform_grid(int8_t logOdds)
{
    OccupancyGrid grid(kGridWidth, kGridHeight, kMetersPerCell);
    
    for(int y = 0; y < grid.heightInCells(); ++y)
    {
        for(int x = 0; x < grid.widthInCells(); ++x)
        {
            grid(x, y) = logOdds;
        }
    }
    
    return grid;
}


OccupancyGrid generate_constricted_grid(double openingWidth)
{
    OccupancyGrid grid = generate_uniform_grid(-10);
    
    // Draw a line right through the middle of the grid. Have the line start after the opening.
    int openingCellY = grid.heightInCells() / 3;
    int openingWidthInCells = openingWidth * grid.cellsPerMeter();
    
    for(int x = openingWidthInCells; x < grid.widthInCells(); ++x)
    {
        grid(x, openingCellY) = 10;
    }
    
    return grid;
}


OccupancyGrid generate_random_grid(double occupiedAmount)
{
    OccupancyGrid grid(kGridWidth, kGridHeight, kMetersPerCell);
    
    for(int y = 0; y < grid.heightInCells(); ++y)
    {
        for(int x = 0; x < grid.widthInCells(); ++x)
        {
            grid(x, y) = (drand48() < occupiedAmount) ? 10 : -10;
        }
    }
    
    return grid;
}


std::ostream& operator<<(std::ostream& out, const pose_xyt_t& pose)
{
    out << '(' << pose.x << ',' << pose.y << ',' << pose.theta << ')';
    return out;
}
