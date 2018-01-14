#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <algorithm>

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    for (int i = 0; i < map.widthInCells(); ++i) {
        for (int j = 0; j < map.heightInCells(); ++j) {
            // std::cout << map(i,j) << std::endl;
            if (map(i,j) > 0) {
                for (int k = -2; k<=2; ++k) {
                    for (int h = -2; h <= 2; ++h) {
                        int temp_w = i + k;
                        int temp_h = j + h;
                        if (isCellInGrid(temp_w,temp_h)) {
                            int index = cellIndex(temp_w,temp_h);
                            // std::cout << map(i,j) << std::endl;
                            cells_[index] = std::min(int(cells_[index]),std::max(abs(h),abs(k)));
                        }
                    }
                }
            }
        }
    } 
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
    for (int i = 0; i < cells_.size(); ++i){
        cells_[i] = 3;
    }
}
