/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include "Dstar.h"
#include "pathSplineSmoother.h"
using std::string;

#define LIDAR_RADIUS 8
#define RETRY_INFLATION 0.1
#define OBSTACLE_THRESHOLD 128
#define RETRY_THRESHOLD 140
#define DO_SMOOTH_PLAN true
#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace dstar_gp
{

    class DstarGP : public nav_core::BaseGlobalPlanner
    {
    private:
        ros::NodeHandle nh;
        ros::Publisher path_pub;
        ros::Subscriber costmap_sub;
        costmap_2d::Costmap2DROS *costmap_ros;
        costmap_2d::Costmap2D *costmap;
        nav_msgs::OccupancyGrid gridmap;
        PathSplineSmoother *spline_smoother;
        Dstar *planner;
        double update_sidelen;
        int plan_counter;
        int planner_is_waiting;
        int is_planner_initialized;
        int do_print;

    public:
        DstarGP();
        DstarGP(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        void costmapUpdateCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const> &grid_map);
        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        void publishPlan(std::vector<geometry_msgs::PoseStamped> path);
        vector<RealPoint> SmoothPlan(list<state> path);
    };
}
#endif