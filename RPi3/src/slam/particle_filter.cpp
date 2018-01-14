#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
    prev_odo.x = 0;
    prev_odo.y = 0;
    prev_odo.theta = 0;
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    for (int i = 0; i < posterior_.size(); ++ i) {
        // set all the poses parent/current to the sent pose and make the weight =  kNumParticles 
        posterior_[i].pose.x = pose.x;
        posterior_[i].pose.y = pose.y;
        posterior_[i].pose.theta = pose.theta;

        posterior_[i].parent_pose.x = pose.x;
        posterior_[i].parent_pose.y = pose.y;
        posterior_[i].parent_pose.theta = pose.theta;
        posterior_[i].weight = 1/kNumParticles_;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t& odometry, const lidar_t& laser, const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry, prev_odo);
    if(hasRobotMoved)
    {
        prev_odo = odometry;
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    posteriorPose_.utime = odometry.utime;
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    for (int i = 0; i < kNumParticles_; ++i) {
        // Threshold = (1/knumparticles)*0.7
        if (posterior_[i].weight < 1/ kNumParticles_*0.7) {
            // set posterior current/parent to posterior_pose
            posterior_[i].pose.x = posteriorPose_.x;
            posterior_[i].pose.y = posteriorPose_.y;
            posterior_[i].pose.theta = posteriorPose_.theta;

            posterior_[i].parent_pose.x = posteriorPose_.x;
            posterior_[i].parent_pose.y = posteriorPose_.y;
            posterior_[i].parent_pose.theta = posteriorPose_.theta;
       }
    }
    return posterior_;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    std::vector<particle_t> proposal;
    for (auto it: prior){
        proposal.push_back(actionModel_.applyAction(it));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal, const lidar_t& laser, const OccupancyGrid& map)
{
    std::vector<particle_t> posterior;
    std::vector<double> weight_mat;   
    double weight = 0; 
    for(auto i : proposal){  
        double weight = sensorModel_.likelihood(i, laser, map);
        weight_mat.push_back(weight);
        weight += weight;
    }
    for(int i = 0; i < proposal.size(); ++i){
        auto sample = proposal[i];
        sample.weight = weight_mat[i]/weight;
        posterior.push_back(sample);
    }
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    float x_estimate = 0;
    float y_estimate = 0;
    float theta_estimate = 0;
    float weight_avg = 0;

    for(auto i : posterior){
        if (i.weight >= 1/kNumParticles_){
            x_estimate += i.pose.x * (i.weight - 1/kNumParticles_);
            y_estimate += i.pose.y * (i.weight - 1/kNumParticles_);
            theta_estimate += i.pose.theta * (i.weight - 1/kNumParticles_);
            weight_avg +=  (i.weight - 1/kNumParticles_);
        }
    }

    // Normalize everything
    weight_avg = fabs(weight_avg);
    pose_xyt_t pose;
    pose.x = x_estimate/weight_avg;
    pose.y = y_estimate/weight_avg; 
    pose.theta = theta_estimate/weight_avg;  
    return pose;
}