#ifndef TARGETCLASS_H
#define TARGETCLASS_H



#include <ros/ros.h>
#include <ros/console.h>
#include <catch_it_package/Target_pos.h>
#include <pthread.h>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Core>

namespace catchClass
{
    class Target
    {

    pthread_mutex_t count_mutex;
    const double reset_countdown_sec = 1.0;

    //all_pos is a vector of accumulated positions x,y,z with a time stamp
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> all_pos;

    public:
    Target();

    void pos_vis_callback(const catch_it_package::Target_pos::ConstPtr& pos);
    void reset();
    bool is_moving();
    bool in_range(int range);

    Eigen::Vector3d get_current_pos();
    Eigen::Vector4d get_current_pos_dt();
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> get_all_pos();
    };
}




#endif // TARGETCLASS_H
