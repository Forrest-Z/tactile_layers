#ifndef TACTILE_LAYER_H_
#define TACTILE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

double filter_min(double data, double max, double min);
double getAngle(double fx, double fy, double init_fx, double init_fy);

namespace tactile_layer_namespace
{

class TactileLayer : public costmap_2d::Layer
{
public:
  TactileLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  //virtual void removeCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
   
private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  double touch_radius_;
  bool touch_status_;
  double mark_x_, mark_y_, fx_, fy_, init_fx_, init_fy_;
  double f_min_;
//   ros::Subscriber once_sub;
  ros::Subscriber ft_sub_;
  bool init_ft_flag_;
  void ftCallBack(const geometry_msgs::WrenchStampedConstPtr& ft_message);
//   void once_cb(const geometry_msgs::WrenchStampedConstPtr& once_message);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
