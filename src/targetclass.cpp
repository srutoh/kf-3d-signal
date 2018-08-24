#include <catch_it_package/targetclass.h>

namespace catchClass
{

Target::Target()
{
  count_mutex = PTHREAD_MUTEX_INITIALIZER;
  //target_pos<<0.,0.,0.,0.;
}

void Target::pos_vis_callback(const catch_it_package::Target_pos::ConstPtr& pos)
{
  double x,y,z,time_stamp;
  Eigen::Vector4d target_pos;

  pthread_mutex_lock( &this->count_mutex );
  x = pos->x;
  y = pos->y;
  z = pos->z;
  time_stamp = ros::Time::now().toSec();
  target_pos << x,y,z,time_stamp;

  all_pos.push_back(target_pos);

  pthread_mutex_unlock( &this->count_mutex );
}

void Target::reset()
{
  pthread_mutex_lock( &this->count_mutex );
  all_pos.clear();
  pthread_mutex_unlock( &this->count_mutex );
}

bool Target::is_moving()
{
  if (!all_pos.empty())
  {
    double t_now, t_latest;
    t_now = ros::Time::now().toSec();
    pthread_mutex_lock( &this->count_mutex );
    t_latest = all_pos.back()(3);

    pthread_mutex_unlock( &this->count_mutex );

    if (t_now - t_latest < reset_countdown_sec)
      return true;
    else
      return false;
  }
  else
    return false;
}

bool Target::in_range(int range)
{
  if (this->is_moving() and all_pos.size() > range)
  {
    return true;
  }
  else
    return false;
}

Eigen::Vector3d Target::get_current_pos()
{
  Eigen::Vector4d pos_temp;
  Eigen::Vector3d current_pos;
  pthread_mutex_lock( &this->count_mutex );
  pos_temp = all_pos.back();

  pthread_mutex_unlock( &this->count_mutex );

  current_pos << pos_temp(0),pos_temp(1),pos_temp(2);
  return current_pos;
}

Eigen::Vector4d Target::get_current_pos_dt()
{
  Eigen::Vector4d pos_temp, current_pos;
  double t_prev, dt;
  pthread_mutex_lock( &this->count_mutex );
  t_prev = all_pos[all_pos.size()-2](3);
  pos_temp = all_pos.back();

  pthread_mutex_unlock( &this->count_mutex );
  dt = pos_temp(3) - t_prev;

  current_pos << pos_temp(0),pos_temp(1),pos_temp(2),dt;
  return current_pos;
}

std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > Target::get_all_pos()
{
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pos_return;
  pthread_mutex_lock( &this->count_mutex );
  pos_return = all_pos;

  pthread_mutex_unlock( &this->count_mutex );
  return pos_return;
}


}
