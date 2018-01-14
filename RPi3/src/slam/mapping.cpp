#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>

//void swap(int& a,int& b){
//    int temp = a;
//    a = b;
//    b = temp;
//}

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{   

    //std::cout << "updating map!!..." << std::endl;

    float gridsize = map.cellsPerMeter();
    //std::cout << gridsize << std::endl;   
    //robot coordinate in map
    //std::cout << pose.x << " " << map.widthInMeters()/2.0 << std::endl;   
    int map_rob_x_ref = ceil(gridsize * (pose.x + map.widthInMeters()/2.0));
    int map_rob_y_ref = ceil(gridsize * (pose.y + map.heightInMeters()/2.0));   
    //std::cout << map_rob_x << " " <<map_rob_y<<std::endl;   
    //for every lidar   
    for(int i=0;i<scan.num_ranges;++i){
        // if (scan.intensities[i] < 0.001){
        //     continue;
        // }//if intensity is too low

        int map_rob_x = map_rob_x_ref;
        int map_rob_y = map_rob_y_ref;
        // occupied points coordinates in real world
        float occ_x = scan.ranges[i]*cos((-scan.thetas[i] + pose.theta)) + pose.x + map.widthInMeters()/2.0;
        float occ_y = scan.ranges[i]*sin((-scan.thetas[i] + pose.theta)) + pose.y + map.heightInMeters()/2.0;
      //  std::cout<<"Theta: "<<pose.theta<<std::endl;

        //occupied points coordinates in map
        int map_occ_x = ceil(gridsize * occ_x);
        int map_occ_y = ceil(gridsize * occ_y);
        //std::cout<<"Curocc"<< map_occ_x<<" " <<map_occ_y<<std::endl;
        //coordinates of occupied in map
        if(map(map_occ_x,map_occ_y) + kHitOdds_ < 127){
            //std::cout<<"Curocc"<< map_occ_x<<map_occ_y<<map(map_occ_x,map_occ_y)+ kHitOdds_<<std::endl;           
            //std::cout<<"Set Cur_x Cur_y Occ Wrong!!"<<std::endl;
            map.setLogOdds(map_occ_x, map_occ_y, map(map_occ_x,map_occ_y)+ kHitOdds_); 
        }
        else{
            //std::cout<<"Curocc"<< map_occ_x<<map_occ_y<<std::endl;   
            //std::cout<<"Set Cur_x Cur_y Occ127 Wrong!!"<<std::endl;
            map.setLogOdds(map_occ_x, map_occ_y, 127);
        }
        //std::cout << map(map_occ_x,map_occ_y)+ kHitOdds_ << std::endl;
        //coordinates of free in map (using breshnham's algorithm) 
        //iterate through the map and assign value
        //std::cout<<"Before Swap!"<<std::endl;
        //std::cout<<"Cur Rob Position: "<< map_rob_x << " " << map_rob_y <<std::endl;
        //std::cout<<"Goal Rob Position: "<< map_occ_x << " " << map_occ_y <<std::endl;       
        bool steep = abs(map_occ_y - map_rob_y) > abs(map_occ_x - map_rob_x);
        if(steep){
            std::swap(map_occ_x,map_occ_y);
            std::swap(map_rob_x,map_rob_y);
        }
        if(map_occ_x - map_rob_x < 0){
            std::swap(map_occ_x, map_rob_x);
            std::swap(map_occ_y, map_rob_y);
        }   
        int dx = map_occ_x - map_rob_x;
        int dy = abs(map_occ_y - map_rob_y);
        int error = dx / 2;
        int ystep;
        int cur_y = map_rob_y;
        if(map_rob_y < map_occ_y){
            ystep = 1;
        }
        else{
            ystep = -1;   
        }
        //std::cout<<"After Swap!"<<std::endl;
        //std::cout<<"Cur Rob Position: "<< map_rob_x << " " << map_rob_y <<std::endl;
        //std::cout<<"Goal Rob Position: "<< map_occ_x << " " << map_occ_y <<std::endl;        

        for(int cur_x = map_rob_x;cur_x < map_occ_x;++cur_x){
            if(steep){
                //std::cout<<"Free yx LogOdds: "<<map(cur_y, cur_x) - kMissOdds_ <<std::endl;
                if(map(cur_y, cur_x) - kMissOdds_ > -127){
                    //std::cout<<"Cury"<< cur_y<<cur_x<<map(cur_y, cur_x) - kMissOdds_<<std::endl;   
                    //std::cout<<"Set Cur_y Cur_x Wrong!!"<<std::endl;
                    //std::cout<<"Curfreeyx: "<< cur_y<<" " <<cur_x<<std::endl;
                    map.setLogOdds(cur_y, cur_x, map(cur_y, cur_x) - kMissOdds_); 
                } 
                else{                   
                    //std::cout<<"Cury"<< cur_y<<cur_x<<std::endl;   
                    //std::cout<<"Set Cur_y Cur_x -127 Wrong!!"<<std::endl;
                    //std::cout<<"Curfreeyx-127"<< cur_y<<" " <<cur_x<<std::endl;
                    map.setLogOdds(cur_y, cur_x, -127);
                }             
             }//steep
             else{
                 //std::cout<<"Free xy LogOdds: "<<map(cur_x, cur_y) - kMissOdds_ <<std::endl;
                 if(map(cur_x, cur_y) - kMissOdds_ > -127){
                     //std::cout<<"Curx"<< cur_x<<cur_y<<map(cur_y, cur_x) - kMissOdds_<<std::endl;   
                     //std::cout<<"Set Cur_x Cur_y Wrong!!"<<std::endl;
                     //std::cout<<"Curfreexy"<< cur_x<<" "<<cur_y<<std::endl;
                     map.setLogOdds(cur_x, cur_y, map(cur_x, cur_y) - kMissOdds_); 
                 }
                 else{
                     //std::cout<<"Curx"<< cur_x<<cur_y<<std::endl;   
                     //std::cout<<"Set Cur_x Cur_y -127 Wrong!!"<<std::endl;
                    //std::cout<<"Curfreexy-127"<< cur_x<<" "<<cur_y<<std::endl;
                    map.setLogOdds(cur_x, cur_y, -127);
                }     
             }

             error = error - dy;
             if(error < 0){
                 cur_y = cur_y +ystep;
                 error = error + dx;
             }//error < 0
         }
     }
     //map.saveToFile("map.txt");
 }