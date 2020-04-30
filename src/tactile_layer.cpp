#include <tactile_layers/tactile_layer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <geometry_msgs/WrenchStamped.h>

PLUGINLIB_EXPORT_CLASS ( tactile_layer_namespace::TactileLayer, costmap_2d::Layer )

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using namespace std;

// double filterMin(double data, double min)
// {
//     if (fabs(data) < min)
//         return 0;
//     else
//         return data;
// }

// double getAngle(double fx, double fy, double init_fx, double init_fy)
// {
//     double x, y, angle;
//     x = filterMin((fx - init_fx), 0.1);
//     y = filterMin(-(fy - init_fy), 0.1);
//     if (x != 0 && y != 0)
//     {
//         touch_status_ = true;
//         if (x != 0)
//         {
//             if (x > 0 && y > 0) // first quad
//                 angle = atan(y / x);
//             else if (x < 0) // second and thrid quad
//                 angle = M_PI + atan(y / x);
//             else // forth quad
//                 angle = 2 * M_PI + atan(y / x);
//         }
//         else
//             angle = 0;
//         return angle;
//     }
//     else
//         touch_status_ = false;
// }

namespace tactile_layer_namespace
{

TactileLayer::TactileLayer()
{
}

void TactileLayer::onInitialize()
{
    ros::NodeHandle nh ( "~/" + name_ );
    current_ = true;

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> ( nh );
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb
        = boost::bind ( &TactileLayer::reconfigureCB, this, _1, _2 );
    dsrv_->setCallback ( cb );
//     ft_sub = nh.subscribe("/netft_data", 1,
//         (boost::function<void(const geometry_msgs::WrenchStampedConstPtr&)>)boost::bind(
//                               &TactileLayer::ft_cb, this, _1));
//     once_sub = nh.subscribe("/netft_data", 1,
//         (boost::function<void(const geometry_msgs::WrenchStampedConstPtr&)>)boost::bind(
//                                 &TactileLayer::once_cb, this, _1));
    ft_sub_ = nh.subscribe<geometry_msgs::WrenchStamped> ( "/netft_data", 1, &TactileLayer::ftCallBack, this );
//     once_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, &TactileLayer::once_cb, this);
    touch_status_ = false;
    touch_radius_ = 0.4;
    f_min_ = 10.0;
}

void TactileLayer::ftCallBack ( const geometry_msgs::WrenchStampedConstPtr& ft )
{
    if ( !init_ft_flag_ ) {
//         init_fx_ = ft->wrench.force.x;
//         init_fy_ = ft->wrench.force.y;
        init_ft_flag_ = true;
    }
    fx_ = fabs ( ft->wrench.force.x - init_fx_ ) < f_min_ ? 0.0 : ft->wrench.force.x - init_fx_;
    fy_ = fabs ( ft->wrench.force.y - init_fy_ ) < f_min_ ? 0.0 : ft->wrench.force.y - init_fy_;
//     cout << "fx:" << fx_ << endl;
//     cout << "fy:" << fy_ << endl;

    if ( fabs ( fx_ ) < f_min_ && fabs ( fy_ ) < f_min_ ) {
        touch_status_ = false;
    } else {
        touch_status_ = true;
    }
}
// void TactileLayer::once_cb(const geometry_msgs::WrenchStampedConstPtr& once)
// {
//     init_fx = once->wrench.force.x;
//     init_fy = once->wrench.force.y;
//     cout << "init fx:" << init_fx << "\n";
//     cout << "init fy:" << init_fy << "\n";
//     once_sub.shutdown();
// }
void TactileLayer::reconfigureCB ( costmap_2d::GenericPluginConfig& config, uint32_t level )
{
    enabled_ = config.enabled;
}

void TactileLayer::updateBounds ( double robot_x, double robot_y, double robot_yaw, double* min_x,
                                  double* min_y, double* max_x, double* max_y )
{

    if ( !enabled_  || touch_status_ == false ) {
        return;
    }

    double angle = atan2 ( fy_, fx_ ); // getAngle ( fx, fy, init_fx, init_fy );
//         if ( angle > 0 ) {
//             mark_x_ = robot_x + 2 * touch_radius_ * cos ( angle );
//         } else {
//             mark_x_ = robot_x - 2 * touch_radius_ * cos ( angle );
//         }
//         mark_y_ = robot_y + 2 * radius * sin ( angle );

    static bool once_flag = false;

    if ( !once_flag ) {
        mark_x_ = robot_x + touch_radius_ * cos ( robot_yaw+angle );
        mark_y_ = robot_y + touch_radius_ * sin ( robot_yaw+angle );
	once_flag = true;
    }

    //cout << "touch_status_:" << touch_status_ << "\n";
//     cout << "angle:" << angle << endl;
//     cout << "robot_yaw: " << robot_yaw << endl;
//     cout << "fx:" << fx_ << endl;
//     cout << "fy:" << fy_ << endl;
//     cout << "robot_x:" << robot_x << endl;
//     cout << "robot_y:" << robot_y << endl;

    *min_x = std::min ( *min_x, mark_x_ );
    *min_y = std::min ( *min_y, mark_y_ );
    *max_x = std::max ( *max_x, mark_x_ );
    *max_y = std::max ( *max_y, mark_y_ );
}

void TactileLayer::updateCosts ( costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j )
{
    if ( !enabled_  || touch_status_ == false ) {
        return;
    }
    uint mx, my;
    double radius = 0.1;
    double res = master_grid.getResolution();
//     unsigned int start_x, start_y, end_x, end_y;
    //cout << "Add cost\n";
    //cout << "mark x =" << mark_x_ << "\n";
    //cout << "mark y =" << mark_y_ << "\n";
//     if ( touch_status_ == false ) {
//         return;
//     }

    master_grid.worldToMap ( mark_x_, mark_y_, mx, my );
    uint start_x = mx - radius / res;
    uint start_y = my - radius / res;
    uint end_x = mx + radius / res;
    uint end_y = my + radius / res;
    for ( int i = start_x; i < end_x; i++ ) {
        for ( int j = start_y; j < end_y; j++ ) {
            master_grid.setCost ( i, j, LETHAL_OBSTACLE );
        }
    }

//     if ( master_grid.worldToMap ( mark_x_, mark_y_, mx, my ) ) {
//         master_grid.setCost ( mx, my, LETHAL_OBSTACLE );
// //         ros::Duration(0.5).sleep();// adding delay results in lower map update rate
//     }
    return;
}

} // end namespace
