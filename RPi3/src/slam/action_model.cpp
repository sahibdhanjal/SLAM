#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


bool ActionModel::updateAction(const pose_xyt_t& odometry, const pose_xyt_t& odometry_pre)
{
	float dx = odometry.x - odometry_pre.x;
	float dy = odometry.y - odometry_pre.y;
	float dtheta = odometry.theta - odometry_pre.theta;

	d_rot1 = atan2(dy,dx) - odometry_pre.theta;
	d_trans = sqrt(dx*dx + dy*dy);
	d_rot2 = dtheta - d_rot1;

	if (dx== 0 && dy == 0) return false;
    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
	particle_t sample_new;

	float d_rot1_cap = d_rot1 - gauss_random(0, alpha_1*d_rot1 + alpha_2*d_trans);
	float d_trans_cap = d_trans - gauss_random(0, alpha_3*d_trans + alpha_4*(d_rot1 + d_rot2));
	float d_rot2_cap = delta_r2 - gauss_random(0, alpha_1*d_rot2 + alpha_2*d_trans);

	sample_new.parent_pose.x = sample_new.pose.x;
	sample_new.parent_pose.y = sample_new.pose.y;
	sample_new.parent_pose.theta = sample_new.pose.theta;

	sample_new.pose.x = sample.pose.x + d_trans_cap*cos(sample.pose.theta + d_rot1_cap);
	sample_new.pose.y = sample.pose.y + d_trans_cap*sin(sample.pose.theta + d_rot1_cap);
	sample_new.pose.theta = sample.pose.theta +  d_rot1_cap + d_rot2_cap;

    return sample_new;
}

double ActionModel::gauss_random(float mean, float stddev) {
	std::random_device de;
	std::default_random_engine generator(de());
  	std::normal_distribution<double> rand_distrib(mean,stddev);
  	return rand_distrib(generator);
}