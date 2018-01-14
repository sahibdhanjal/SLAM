#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{	
	float size = map.cellsPerMeter();
    for(int i=0;i<scan.num_ranges;++i){

        float x = sample.pose.x + map.widthInMeters()/2.0;
        float y = sample.pose.y + map.heightInMeters()/2.0;
        float theta = -scan.thetas[i] + sample.pose.theta;

        // position according to the laser scan
        float real_x = scan.ranges[i]*cos(theta) + x;
        float real_y = scan.ranges[i]*sin(theta) + y;

        // position in the map
        int map_x = ceil(size * real_x);
        int map_y = ceil(size * real_y); 

        // if position in range within map
        if (map_x > 0 && map_x < 200 && map_y > 0 && map_y < 200){
        	if(map(map_x,map_y)>0){
	        	weight += 1 + map(map_x,map_y)/50;
	        }
        }
        
    }
    return weight;
}
