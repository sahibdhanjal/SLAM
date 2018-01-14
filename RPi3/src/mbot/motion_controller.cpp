#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>


float clamp_speed(float speed)
{
    if(speed < -1.0f)
    {
        return -1.0f;
    }
    else if(speed > 1.0f)
    {
        return 1.0f;
    }
    
    return speed;
}


class MotionController{



public:

float start_position[2];
float current_position[2];
float base_direction[2];
float heading_direction[2];
float norm_distance;
float parallel_distance;
float heading_error;
float angle_x_position;
float angle_x_dest;



public:
    
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance) : lcmInstance(instance)
    {
        ////////// TODO: Initialize your controller state //////////////
        
        // Initially, there's no offset between odometry and the global state
        odomToGlobalFrame_.x = 0.0f;
        odomToGlobalFrame_.y = 0.0f;
        odomToGlobalFrame_.theta = 0.0f;

	    time_offset = 0;
	    timesync_initialized_ = false;

        confirm.utime = 0;
        confirm.creation_time = 0;
        confirm.channel = "";

        start_position[0] = 0.;
        start_position[1] = 0.;
        current_position[0] = 0.;
        current_position[1] = 0.;
        std::cout<< "initialized!" << std::endl;

    }
    
    /**
    * updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void)
    {
        //////////// TODO: Implement your feedback controller here. //////////////////////
        
        const float kPGain = 1.2f;
        const float kDGain = 0.9f;//1.0f;
        const float kIGain = 0.0f;//0.0001f;

        const float kSlowdownDist = 0.2f;//0.26f;
        const float kDesiredSpeed = 0.5f;//0.55f;
        const float kMinSpeed = 0.02f;//0.4f;
        const float kTurnSpeed = 1.0f;//0.3f;
        
        mbot_motor_command_t cmd;

        cmd.trans_v = 0.0f;
        cmd.angular_v = 0.0f;

        cmd.utime = now();
        
        if(haveReachedTarget())
        {
		std::cout << "TARGET REACHED\n";
            bool haveTarget = assignNextTarget();
            
            if(!haveTarget)
            {
                std::cout << "COMPLETED PATH!\n";
            }
        }
        
        /*if(targets_.empty() || odomTrace_.empty())
        {
            cmd.trans_v = 0.0f;
            cmd.angular_v = 0.0f;
        }*/
        if(!targets_.empty() && !odomTrace_.empty())
        {
            // Use feedback based on heading error for line-of-sight vector pointing to the target.
            pose_xyt_t target = targets_.back();
            
            // Convert odometry to the global coordinates
            pose_xyt_t pose = currentPose();

            // update the current position
            current_position[0] = pose.x;
            current_position[1] = pose.y;
            std::cout << "start: " << start_position[0] << " " << start_position[1] << std::endl;
            std::cout << "target: " << target.x << " " << target.y << std::endl;
            std::cout << "current: " << current_position[0] << " " << current_position[1] << std::endl;
            // update the desired direction

            base_direction[0] = target.x - start_position[0];
            base_direction[1] = target.y - start_position[1];

            // update the heading direction

            heading_direction[0] = target.x - current_position[0];
            heading_direction[1] = target.y - current_position[1];

            // calculate the normal projection

            angle_x_position = atan2(heading_direction[1],heading_direction[0]);

            angle_x_dest = atan2(base_direction[1],base_direction[0]);
            std::cout << "angle_x_position: " << angle_x_position<< std::endl;
            std::cout << "angle_x_dest: " <<angle_x_dest << std::endl;

            heading_error = angle_x_dest - angle_x_position;

            float angle_error = angle_x_dest - pose.theta;
             std::cout << "heading: " << heading_direction[0] << " " << heading_direction[1] << std::endl;

            norm_distance = 2*get_2_norm(heading_direction)*sin(heading_error) - 0.5*sin(angle_error);

            //std::cout << "norm_distance: "<< norm_distance << " parallel_distance: " << parallel_distance << std::endl;    
            // calculate the projection (parallel to the base) distance
            parallel_distance = get_2_norm(heading_direction)*cos(heading_error);
            std::cout << "norm_distance: " << norm_distance << std::endl;
            if (parallel_distance < 0.05 || fabs(norm_distance) > 0.2) {
                cmd.trans_v = 0.;
            }
            else  cmd.trans_v = 0.5;
            cmd.angular_v = norm_distance;

            return cmd;










            
            
  //           double targetHeading = std::atan2(target.y - pose.y, target.x - pose.x);
  //           double error = angle_diff(targetHeading, pose.theta);
  //           std::cout << "targetHeading: " << targetHeading << ", pose Theta: " << pose.theta << std::endl;
  //           std::cout << "Angle error:" << error << '\n';

  //           if(state_ == TURN)
  //           {
  //               if(std::abs(error) > 0.10) //0.10) turn in place until pointed approximately at the target
  //               {

  //                   cmd.trans_v = 0; //set translational velocity to 0

  //                   float turnspeed = kTurnSpeed;
		    
		//     //if the error is small, vary the turnspeed linearly with the error
  //                   if(std::abs(error) < 0.7) turnspeed = -0.35 + 4 * std::abs(error);
  //                   turnspeed = std::min(turnspeed, kTurnSpeed);//don't turn faster than the desired turnspeed

  //                   // Turn left if the target is to the left
  //                   if(error > 0.0)
  //                   {
  //                       std::cout << "Turning left\n";
  //                       cmd.trans_v = 0;
  //                       cmd.angular_v = turnspeed;
  //                   }
  //                   // Turn right if the target is to the right
  //                   else // if(error < 0.0)
  //                   {
  //                       std::cout << "Turning right\n";
  //                       cmd.trans_v = 0;
  //                       cmd.angular_v = -turnspeed;
  //                   }

  //               }
  //               else
  //               {
  //                   std::cout << "Entering DRIVE state.\n";
		//             state_ = DRIVE;
  //               }
  //           }
  //           else if(state_ == DRIVE) // Use feedback to drive to the target once approximately pointed in the correct direction
  //           {
  //               double speed = kDesiredSpeed;

  //               double distToGoal = std::sqrt(std::pow(target.x - pose.x, 2.0f) + std::pow(target.y - pose.y, 2.0f));
                
  //               //if distance is less than slowdown distance, make the speed vary with the sqrt
  //               //of the distance to the goal
  //               if(distToGoal < kSlowdownDist)
  //               {

  //                   speed = kMinSpeed + std::sqrt(distToGoal);
  //               }
                
  //               //update pid values
  //               double deltaError = error - lastError_;
  //               totalError_ += error;
  //               lastError_ = error;

  //               //go slower if the angle error is greater
		//         speed *= std::cos(error);
  //               //don't go backwards
  //               speed = std::max(0.0f, float(speed));

  //               cmd.trans_v = speed;

  //               //pid control the angular v based on angle error
  //               cmd.angular_v = (error * kPGain) + (deltaError * kDGain) + (totalError_ * kIGain);
  //               //angular velocity must not exceed 1.5 the desired turnspeed
  //               cmd.angular_v = std::min(cmd.angular_v, kTurnSpeed * 1.5f);
		// }
  //           else
  //           {
  //               std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
  //           }
        }
        
  //       return cmd;
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync){
	timesync_initialized_ = true;
	time_offset = timesync->utime-utime_now();
    }
    
    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        /////// TODO: Implement your handler for new paths here ////////////////////

        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	std::cout << "received new path at time: " << path->utime << "\n";
    	for(auto pose : targets_){
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}std::cout << "\n";


        assignNextTarget();

        confirm.utime = now();
        confirm.creation_time = path->utime;
        confirm.channel = channel;

        //confirm that the path was received
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }
    
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        /////// TODO: Implement your handler for new odometry data ////////////////////
        
        pose_xyt_t pose;
        pose.utime = odometry->utime;
        pose.x = odometry->x;
        pose.y = odometry->y;
        pose.theta = odometry->theta;
        odomTrace_.addPose(pose);
    }
    
    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        /////// TODO: Implement your handler for new pose data ////////////////////    
        computeOdometryOffset(*pose);
    }


    float get_2_norm(float p1[]) {
        return sqrt(pow(p1[0],2) + pow(p1[1],2));
    }

    
private:
    
    enum State
    {
        TURN,
        DRIVE,
    };
    
    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;
    
    // Error terms for the current target
    State state_;
    double lastError_;      // for D-term
    double totalError_;     // for I-term

    int64_t time_offset;

    bool timesync_initialized_;

    message_received_t confirm;
    lcm::LCM * lcmInstance;

    int64_t now(){
	return utime_now()+time_offset;
    }

    bool haveReachedTarget(void)
    {

        if (parallel_distance < 0.05f)
            return true;
        else
            return false;

        
     //    const float kPosTolerance = 0.1f;
	    // const float kFinalPosTolerance = 0.05f;

     //    //tolerance for intermediate waypoints can be more lenient
    	// float tolerance = (targets_.size() == 1) ? kFinalPosTolerance : kPosTolerance;
        
     //    // There's no target, so we're there by default.
     //    if(targets_.empty())
     //    {
     //        return false;
     //    }
     //    // If there's no odometry, then we're nowhere, so we couldn't be at a target
     //    if(odomTrace_.empty())
     //    {
     //        return false;
     //    }
        
     //    pose_xyt_t target = targets_.back();
     //    pose_xyt_t pose = currentPose();
        
     //    float xError = std::abs(target.x - pose.x);
     //    float yError = std::abs(target.y - pose.y);
        
     //    return (state_ == DRIVE) && (xError < tolerance) && (yError < tolerance);
    }
    
    bool assignNextTarget(void)
    {
        // If there was a target, remove it

        // start_position[0] = targets_.back()[0];
        // start_position[0] = targets_.back()[];

        if(!targets_.empty())
        {

            targets_.pop_back();
        }
        
        // Reset all error terms when switching to a new target
        lastError_ = 0.0f;
        totalError_ = 0.0f;
        state_ = TURN;

        start_position[0] = current_position[0];
        start_position[1] = current_position[1];

        // std::cout << current_position[0] << " " << current_position[0] << std::endl;

        
        return !targets_.empty();
    }
    
    void computeOdometryOffset(const pose_xyt_t& globalPose)
    {
        pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
        
        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated;
        odomToGlobalFrame_.theta = deltaTheta;
    }
    
    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        
        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) 
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
        
        return pose;
    }
};


int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    
    MotionController controller(&lcmInstance);
    lcmInstance.subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, &controller);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, &controller);
    lcmInstance.subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, &controller);
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, &controller);

    signal(SIGINT, exit);
    
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            	mbot_motor_command_t cmd = controller.updateCommand();
                std::cout << cmd.trans_v << " " << cmd.angular_v <<std::endl;

            	lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}
    }
    
    return 0;
}
