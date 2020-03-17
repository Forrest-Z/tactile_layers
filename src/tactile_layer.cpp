#include <tactile_layers/tactile_layer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <geometry_msgs/WrenchStamped.h>

PLUGINLIB_EXPORT_CLASS(tactile_layer_namespace::TactileLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using namespace std;
bool touch;

double filter_min(double data, double min)
{
    if (abs(data) < min)
        return 0;
    else
        return data;
}
double getAngle(double fx, double fy, double init_fx, double init_fy)
{
    double x, y, angle;
    x = filter_min((fx - init_fx), 0.1);
    y = filter_min(-(fy - init_fy), 0.1);
    if (x != 0 && y != 0)
    {
        touch = true;
        if (x != 0)
        {
            if (x > 0 && y > 0) // first quad
                angle = atan(y / x);
            else if (x < 0) // second and thrid quad
                angle = M_PI + atan(y / x);
            else // forth quad
                angle = 2 * M_PI + atan(y / x);
        }
        else
            angle = 0;
        return angle;
    }
    else
        touch = false;
}

namespace tactile_layer_namespace
{

TactileLayer::TactileLayer()
{
}

void TactileLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb
        = boost::bind(&TactileLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    ft_sub = nh.subscribe("/netft_data", 5,
        (boost::function<void(const geometry_msgs::WrenchStampedConstPtr&)>)boost::bind(
                              &TactileLayer::ft_cb, this, _1));
    once_sub = nh.subscribe("/netft_data", 1,
        (boost::function<void(const geometry_msgs::WrenchStampedConstPtr&)>)boost::bind(
                                &TactileLayer::once_cb, this, _1));
    touch = false;
}

void TactileLayer::ft_cb(const geometry_msgs::WrenchStampedConstPtr& ft)
{
    fx = ft->wrench.force.x;
    fy = ft->wrench.force.y;
}
void TactileLayer::once_cb(const geometry_msgs::WrenchStampedConstPtr& once)
{
    init_fx = once->wrench.force.x;
    init_fy = once->wrench.force.y;
    once_sub.shutdown();
}
void TactileLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level)
{
    enabled_ = config.enabled;
}

void TactileLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
    double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
        return;
    double radius = 0.1;
    double angle = getAngle(fx, fy, init_fx, init_fy);
    // cout << "init fx:" << init_fx << "\n";
    // cout << "init fy:" << init_fy << "\n";
    // cout << "touch:" << touch << "\n";
    if (touch == true)
    {
        if (angle > 0)
            mark_x_ = robot_x + radius * cos(angle);
        else
            mark_x_ = robot_x - 2 * radius * cos(angle);

        mark_y_ = robot_y + 2 * radius * sin(angle);
    }

    *min_x = std::min(*min_x, mark_x_ - radius);
    *min_y = std::min(*min_y, mark_y_ - radius);
    *max_x = std::max(*max_x, mark_x_ + radius);
    *max_y = std::max(*max_y, mark_y_ + radius);
    //  *min_x = std::min(*min_x, mark_x_);
    //  *min_y = std::min(*min_y, mark_y_);
    //  *max_x = std::max(*max_x, mark_x_);
    //  *max_y = std::max(*max_y, mark_y_);
}

void TactileLayer::updateCosts(
    costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;
    unsigned int mx, my;
    double radius = 0.1;
    double res = master_grid.getResolution();
    unsigned int start_x, start_y, end_x, end_y;

    //cout << "mark x =" << mark_x_ << "\n";
    //cout << "mark y =" << mark_y_ << "\n";
    if (touch == true)
    {
        master_grid.worldToMap(mark_x_, mark_y_, mx, my);
        start_x = mx - radius / res;
        start_y = my - radius / res;
        end_x = mx + radius / res;
        end_y = my + radius / res;
        for (int i = start_x; i < end_x; i++)
        {
            for (int j = start_y; j < end_y; j++)
            {
                master_grid.setCost(i, j, LETHAL_OBSTACLE);
            }
        }
    }
    else if (touch == false && (mark_x_ != 0 || mark_y_ != 0))
    {
        for (int i = start_x; i < end_x; i++)
        {
            for (int j = start_y; j < end_y; j++)
            {
                master_grid.setCost(i, j, FREE_SPACE);
            }
        }
    }
}

} // end namespace
