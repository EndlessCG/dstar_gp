#include <pluginlib/class_list_macros.h>
#include "dstar_gp.h"
#include <ctime>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dstar_gp::DstarGP, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace dstar_gp
{

  DstarGP::DstarGP()
  {
    is_planner_initialized = 0;
    plan_counter = 0;
  }

  DstarGP::DstarGP(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    is_planner_initialized = 0;
    initialize(name, costmap_ros);
  }

  vector<RealPoint> DstarGP::SmoothPlan(list<state> path)
  {

    ROS_DEBUG("SmoothPlan / getting costmap infos");
    double costmap_resolution = costmap->getResolution();
    double origin_costmap_x = costmap->getOriginX();
    double origin_costmap_y = costmap->getOriginY();

    /// copying the path in a different format
    int initial_path_size = (int)path.size();
    vector<RealPoint> input_path;
    input_path.clear();
    if (initial_path_size == 0)
    {
      ROS_ERROR("Path not valid for smoothing, returning");
      return input_path;
    }

    ROS_DEBUG("SmoothPlan, filling the points");

    std::list<state>::const_iterator iterator;
    double t, old_x, old_y, old_th, dt;
    dt = 0.1;
    int cnt = 0;
    for (iterator = path.begin(); iterator != path.end(); ++iterator)
    {

      state node = *iterator;

      /// giving as input path the cartesian path
      double x, y;
      if (node.x < 0 || node.y < 0)
        ROS_ERROR("Invalid node passed to smoother.");
      costmap->mapToWorld(node.x, node.y, x, y);
      ROS_DEBUG("Dealing with original point %f, %f", x, y);
      // double x = (node.x + 0.5) * costmap_resolution + origin_costmap_x;
      // double y = (node.y + 0.5) * costmap_resolution + origin_costmap_y;

      if (cnt > 0)
      {

        t = dt;

        while (t < 1)
        {
          RealPoint p_new;
          p_new.x = (x - old_x) * t + old_x;
          p_new.y = (y - old_y) * t + old_y;
          p_new.theta = 0;
          input_path.push_back(p_new);
          ROS_DEBUG("Adding point %f %f ", p_new.x, p_new.y);
          t = t + dt;
        }
      }
      else
      {

        RealPoint p;
        p.x = x;
        p.y = y;
        p.theta = 0;
        input_path.push_back(p);
        ROS_DEBUG("Adding Initial point %f %f of a segment ", x, y);
      }
      if (x < gridmap.info.origin.position.x || x > gridmap.info.origin.position.x + gridmap.info.width * gridmap.info.resolution || y < gridmap.info.origin.position.y || y > gridmap.info.origin.position.y + gridmap.info.height * gridmap.info.resolution)
        break;
      old_x = x;
      old_y = y;
      old_th = 0;
      cnt++;
    }

    // do not smooth if the path has not enough points
    if (initial_path_size < 3)
    {
      ROS_DEBUG("Returning path, without smoothing it");
      return input_path;
    }
    //return input_path;
    ROS_DEBUG("SmoothPlan, Providing the path to the smoother");
    spline_smoother->readPathFromStruct(input_path);
    ROS_DEBUG("SmoothPlan, Filtering path");
    spline_smoother->filterPath(1);
    ROS_DEBUG("SmoothPlan, Smoothing path");
    // spline_smoother_->smoothPath2D();
    spline_smoother->smoothWhileDistanceLessThan(0.05, 1.01);
    ROS_DEBUG("SmoothPlan, getting path");
    vector<RealPoint> smoothed_path = spline_smoother->getSmoothPath();
    for (; iterator != path.end(); iterator++)
    {
      double x, y;
      RealPoint p;
      if (iterator->x < 0 || iterator->y < 0)
        ROS_ERROR("Invalid node passed to smoother.");
      costmap->mapToWorld(iterator->x, iterator->y, x, y);
      p.x = x;
      p.y = y;
      p.theta = 0;
      input_path.push_back(p);
    }
    return smoothed_path;
  }
  void DstarGP::costmapUpdateCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const> &map)
  {
    ROS_INFO("Local costmap got!");
    gridmap = *map;
    planner_is_waiting = 1;
    for (auto it = gridmap.data.begin(); it != gridmap.data.end(); it++)
    {
      if (*it > OBSTACLE_THRESHOLD)
        planner_is_waiting = 0;
      //else planner_is_waiting = 1;
    }
    //printf("map checked, planner_is_waiting=%d", planner_is_waiting);
  }

  void DstarGP::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    this->planner_is_waiting = 1;
    this->do_print = 0;
    ROS_INFO("Dstar GP is initializing...");
    nh = ros::NodeHandle("dstar_gp");
    spline_smoother = new PathSplineSmoother();
    //nh.getParam("DstarGP/map_update_sidelen", update_sidelen);
    //update_sidelen = 6;
    this->costmap_ros = costmap_ros;
    this->costmap = costmap_ros->getCostmap();
    this->path_pub = nh.advertise<nav_msgs::Path>("path", 10);
    this->costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1, boost::bind(&DstarGP::costmapUpdateCallback, this, _1));
    planner = new Dstar();
    plan_counter = 0;
    is_planner_initialized = 0; //just to make sure.
    if (planner)
      ROS_INFO("Dstar GP has initialized successfully.");
    else
      ROS_ERROR("Dstar GP initialization failed!");
  }

  void DstarGP::publishPlan(std::vector<geometry_msgs::PoseStamped> path)
  {
    nav_msgs::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = ros::Time();
    nav_path.poses = path;
    path_pub.publish(nav_path);
    ROS_INFO("Path of length %ld has been published by DstarGP.", path.size());
  }
  bool DstarGP::makePlan(const geometry_msgs::PoseStamped &start,
                         const geometry_msgs::PoseStamped &goal,
                         std::vector<geometry_msgs::PoseStamped> &plan)
  {
    //For future debugging: since we trust the source of our start and goal, we believe they will
    //be sent on the /map frame. Therefore we don't perform transform here, trading robustness
    //for simplicity.
    //ROS_INFO("make plan starts");

    //wait for map to be updated
    // planner_is_waiting = 1;
    // ROS_INFO("planner is waiting for map.");
    //printf("in makePlan, planner_is_waiting=%d", planner_is_waiting);
    // if (planner_is_waiting == 1)
    // {
    //   ROS_INFO("Planner is waiting for gridmap.");
    //   std::vector<geometry_msgs::PoseStamped> empty_path;
    //   publishPlan(empty_path);
    //   return true;
    // }

    plan_counter++;
    unsigned int map_startx, map_starty, map_goalx, map_goaly;
    costmap->worldToMap(start.pose.position.x, start.pose.position.y, map_startx, map_starty);
    costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, map_goalx, map_goaly);
    if (!is_planner_initialized)
    {
      ROS_INFO("Dstar global planner is fully initialized.");
      planner->init(map_startx, map_starty, map_goalx, map_goaly);
      is_planner_initialized = 1;
    }
    else
    {
      planner->updateStart(map_startx, map_starty);
      planner->updateGoal(map_goalx, map_goaly);
    }

    ROS_INFO("start: %d, %d, goal: %d, %d", map_startx, map_starty, map_goalx, map_goaly);
    ROS_INFO("start: %f, %f, goal: %f, %f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    plan.clear();
    unsigned int i, j, orig_dstar_x, orig_dstar_y;
    //convert gridmap.info.origin into map coordinate.
    costmap->worldToMap(gridmap.info.origin.position.x, gridmap.info.origin.position.y, orig_dstar_x, orig_dstar_y);
    double x1, y1, x2, y2, x3, y3;
    double tx, ty;
    int threshold_now = OBSTACLE_THRESHOLD;

    if (costmap->getCost(orig_dstar_x, orig_dstar_y) > OBSTACLE_THRESHOLD)
    {
      ROS_INFO("Robot is near obstacle");
      //threshold_now = costmap->getCost(orig_dstar_x, orig_dstar_y) + 1;
      //threshold_now = 140;
    }

    int give_up = 0;
    do
    {
      // update with local costmap, deprecated.
      // for (i = 0; i < gridmap.info.height; i++)
      // {
      //   for (j = 0; j < gridmap.info.width; j++)
      //   {
      //     int8_t cost = gridmap.data[i * gridmap.info.width + j];
      //     if (cost > threshold_now)
      //     {
      //       //costmap->mapToWorld(orig_dstar_x+j, orig_dstar_y+i, tx, ty);
      //       //printf("(%f,%f),", tx, ty);
      //       planner->updateCell(orig_dstar_x + j, orig_dstar_y + i, -1);
      //       //printf("(%d, %d), %d", orig_dstar_x + i, orig_dstar_y + j, -1);
      //     }
      //     else
      //     {
      //       //if(cost != 0)ROS_INFO("cost is %f, this is free space");
      //       planner->updateCell(orig_dstar_x + j, orig_dstar_y + i, 1);
      //       //printf("(%d, %d), %d", orig_dstar_x + i, orig_dstar_y + j, 1);
      //     }
      //   }
      // }

      //update with global costmap.
      unsigned int update_size = (unsigned int)(LIDAR_RADIUS / costmap->getResolution());
      ROS_INFO("Updating with threshold %d", threshold_now);
      ROS_INFO("Starting point cost %ld", costmap->getCost(orig_dstar_x, orig_dstar_y));
      for (i = 0; i < update_size; i++)
      {
        for (j = 0; j < update_size; j++)
        {
          unsigned char* grid = costmap->getCharMap();
          int idx = costmap->getIndex(orig_dstar_x + i, orig_dstar_y + j);
          double c = (double)grid[idx];
          //printf("%f", c);
          //double c = (double)costmap->getCost(orig_dstar_x + i, orig_dstar_y + j);
          // if (cost > threshold_now)
          // {
          // if (do_print == 1)
          //   printf("(%d,%d),", orig_dstar_x + i, orig_dstar_y + j);
          //   //printf("%d", cost);
          //   planner->updateCell(orig_dstar_x + i, orig_dstar_y + j, -1);
          // }
          // else
          // {

          //   //printf("(%d,%d)", orig_dstar_x + i, orig_dstar_y + j);
          //   //printf("%d", cost);
          //   planner->updateCell(orig_dstar_x + i, orig_dstar_y + j, 1);
          // }
          
          if (c >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            planner->updateCell(orig_dstar_x + i, orig_dstar_y + j, -1);
            //printf("(%d,%d),", orig_dstar_x + i, orig_dstar_y + j);
          }
          else if (c == costmap_2d::FREE_SPACE)
          {
            planner->updateCell(orig_dstar_x + i, orig_dstar_y + j, 1);
          }
          else
          {
            //printf("(%d, %d),", orig_dstar_x + i, orig_dstar_y + j);
            planner->updateCell(orig_dstar_x + i, orig_dstar_y + j, c / 128);
          }
        }
      }
      //printf("getmap: %f, getindex&cost: %f, updateCell: %f",t_acc[0], t_acc[1], t_acc[2]);
      //ROS_INFO("dstar starts");
      if (!planner->replan())
      {
        ROS_WARN("No path found. Retrying...");
        //do_print = 1;
        //ros::param::set("/move_base/local_costmap/inflation_radius", RETRY_INFLATION);
        threshold_now = RETRY_THRESHOLD;
        give_up++;
      }
      else
      {
        do_print = 0;
        break;
      }
    } while (give_up != 2);
    if (give_up == 2)
    {
      ROS_WARN("No path found.");
      return false;
    }
    else
    {
      //ROS_INFO("dtsar ended");
      list<state> path = planner->getPath();
      //ROS_INFO("Global path got!");
      ROS_INFO("path");
      plan.push_back(start);
      if (DO_SMOOTH_PLAN)
      {
        vector<RealPoint> path_smoothed = SmoothPlan(path);
        for (const auto &n : path_smoothed)
        {
          geometry_msgs::PoseStamped node;
          node.header.seq = plan_counter;
          node.header.stamp = ros::Time::now();
          node.header.frame_id = "map";
          //costmap->mapToWorld(n.x, n.y, node.pose.position.x, node.pose.position.y);
          node.pose.position.x = n.x;
          node.pose.position.y = n.y;
          node.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, n.theta);
          //printf("(%f, %f), ", n.x, n.y);
          plan.push_back(node);
        }
      }
      else
      {
        for (const auto &n : path)
        {
          geometry_msgs::PoseStamped node;
          node.header.seq = plan_counter;
          node.header.stamp = ros::Time::now();
          node.header.frame_id = "map";
          costmap->mapToWorld(n.x, n.y, node.pose.position.x, node.pose.position.y);
          //printf("(%d, %d), ", n.x, n.y);
          //node.pose.position.y = (n.y + 0.5) * costmap->getResolution();
          plan.push_back(node);
        }
      }
    }
    plan.push_back(goal);
    publishPlan(plan);
    //ROS_INFO("Path published.");
    return true;
  }
}