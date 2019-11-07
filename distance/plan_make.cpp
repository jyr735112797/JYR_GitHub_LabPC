#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <plan_make.h>

#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_make");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}

CoveragePlanner::CoveragePlanner()
    : nh_(),
      nh_private_("~"),
      footprint_helper_(new base_local_planner::FootprintHelper()),
      costmap_(new costmap_2d::Costmap2D()),
      costmap_model_(new base_local_planner::CostmapModel(*costmap_)),
      inflation_costmap_(new costmap_2d::Costmap2D()),
      inflation_costmap_model_(new base_local_planner::CostmapModel(*inflation_costmap_))
{
//
    costmap_publisher_ = new costmap_2d::Costmap2DPublisher(&nh_private_, costmap_, map_frame_, std::string("costmap"), true);
    inflation_costmap_publisher_ = new costmap_2d::Costmap2DPublisher(&nh_private_, inflation_costmap_, map_frame_, std::string("inflation_costmap"), true);

    //
    footprint_spec_ = costmap_2d::makeFootprintFromParams(nh_private_); // setup footprint or robot_radius
//    global_planner_ = new navfn::NavfnROS("global_planner", inflation_costmap_, map_frame_);
    global_planner_ = new global_planner::GlobalPlanner("global_planner", inflation_costmap_, map_frame_);


    // Build inflation costmap
    mapGridToCostmap(map_data, inflation_costmap_);
    buildInflationCostmap(costmap_model_, robot_radius_, inflation_costmap_);
    //
    inflation_costmap_publisher_->publishCostmap();

}

bool CoveragePlanner::mapGridToCostmap(const MapParams &map, costmap_2d::Costmap2D* costmap)
{
    if(map.width == 0 || map.height == 0 || map.data.size() == 0 || map.data.size() != (map.width*map.height))
        return false;

    // Create new costmap
    costmap->resizeMap(map.width, map.height, map.resolution, map.origin.position.x, map.origin.position.y);

    boost::unique_lock<boost::recursive_mutex> lock(*(costmap->getMutex()));
//    //Info
//    ROS_INFO("Set static cost.");
//    ROS_INFO_STREAM("Map width = " << map.width);
//    ROS_INFO_STREAM("Map height = " << map.height);
//    ROS_INFO_STREAM("Map data = " << map.data.size());
//    ROS_INFO_STREAM("size = " << sizeof(costmap->getCharMap()));
    //
    unsigned int index = 0;
    for (unsigned int i = 0; i < map.height; ++i)
    {
        for (unsigned int j = 0; j < map.width; ++j)
        {
//            cout << index << " " << std::flush;
            if(index >= map.height*map.width)
            {
                cout << "Index error." << endl;
                return false;
            }

            unsigned char value = map.data[index];
            if(value > 0)
                costmap->getCharMap()[index] = 254; //costmap_2d::LETHAL_OBSTACLE;
            else
                costmap->getCharMap()[index] = 0;   //costmap_2d::FREE_SPACE;
            ++index;
        }
    }

    return true;
}

bool CoveragePlanner::saveCostmap(std::string file_name, costmap_2d::Costmap2D* costmap)
{
    FILE *fp = fopen(file_name.c_str(), "w");

    if (!fp)
    {
        return false;
    }

    int size_x = costmap->getSizeInCellsX();
    int size_y = costmap->getSizeInCellsY();

    fprintf(fp, "P2\n%u\n%u\n%u\n", size_x, size_y, 0xff);
    for (unsigned int iy = 0; iy < size_y; iy++)
    {
        for (unsigned int ix = 0; ix < size_x; ix++)
        {
            unsigned char cost = costmap->getCost(ix, size_y-iy-1);
            fprintf(fp, "%d ", 255-cost);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}


void CoveragePlanner::buildInflationCostmap(base_local_planner::CostmapModel* costmap_model, double distance,
                                           costmap_2d::Costmap2D* inflation_costmap)
{
    std::vector<geometry_msgs::Point> footprint_spec = costmap_2d::makeFootprintFromRadius(distance);
    unsigned int height = inflation_costmap->getSizeInCellsY();
    unsigned int width = inflation_costmap->getSizeInCellsX();
    for(unsigned int y = 0; y < height; y++)
    {
        for(unsigned int x = 0; x < width; x++)
        {
            if(inflation_costmap->getCost(x, y) != 0)
                continue;

            double wx, wy, wa = 0;
            inflation_costmap->mapToWorld(x, y, wx, wy);
            double cost = getFootprintCost(wx, wy, wa, footprint_spec, inflation_costmap, costmap_model);
            if(cost != 0)
            {
                inflation_costmap->setCost(x, y, 254);
            }
        }
    }
}


bool CoveragePlanner::makeGlobalPlan(const geometry_msgs::Point &start,
                                     const geometry_msgs::Point &goal,
                                     double interval,
                                     std::vector<geometry_msgs::Point> &plan)
 {
     plan.clear();
     geometry_msgs::PoseStamped start_pose, goal_pose;
     start_pose.pose.position = start;
     start_pose.pose.orientation.w = 1.0;
     start_pose.header.frame_id = "map";
     goal_pose.pose.position = goal;
     goal_pose.pose.orientation.w = 1.0;
     goal_pose.header.frame_id = "map";
     std::vector<geometry_msgs::PoseStamped> global_plan;
     if(global_planner_->makePlan(start_pose, goal_pose, global_plan))
     {
 //        for(size_t i = 0; i < global_plan.size(); i++)
 //        {
 //            plan.push_back(global_plan[i].pose.position);
 //        }

         if(global_plan.size() < 3)
         {
             plan.push_back(start);
             plan.push_back(goal);
         }
         else
         {
             geometry_msgs::Point last_pose = global_plan[0].pose.position;
             plan.push_back(last_pose);
             for(size_t i = 1; i < (global_plan.size()-1); i++)
             {
                 geometry_msgs::Point pose = global_plan[i].pose.position;
                 double dis = hypot(last_pose.x-pose.x, last_pose.y-pose.y);
                 if(dis >= interval)
                 {
                     last_pose = pose;
                     plan.push_back(last_pose);
                 }

             }
             plan.push_back(global_plan[global_plan.size()-1].pose.position);
         }

         return true;
     }
     else
     {
         plan.push_back(start);
         plan.push_back(goal);
         ROS_WARN_STREAM("Failed to get global plan.");
         return true;
     }
 }


/*
void fillPathRequest(nav_msgs::GetPlan::Request &request)
{
request.start.header.frame_id ="map";
request.start.pose.position.x = 12.378;//初始位置x坐标
request.start.pose.position.y = 28.638;//初始位置y坐标
request.start.pose.orientation.w = 1.0;//方向
request.goal.header.frame_id = "map";
request.goal.pose.position.x = 18.792;//终点坐标
request.goal.pose.position.y = 29.544;
request.goal.pose.orientation.w = 1.0;
request.tolerance = 0.5;//如果不能到达目标，最近可到的约束
}
//路线规划结果回调
void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
{
// Perform the actual path planner call
//执行实际路径规划器
if (serviceClient.call(srv)) {
//srv.response.plan.poses 为保存结果的容器，遍历取出
if (!srv.response.plan.poses.empty()) {
forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
}
}
else {
ROS_WARN("Got empty plan");
}
}
else {
ROS_ERROR("Failed to call service %s - is the robot moving?",
serviceClient.getService().c_str());
}
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "make_plan_node");
ros::NodeHandle nh;
// Init service query for make plan
//初始化路径规划服务，服务名称为"move_base_node/make_plan"
std::string service_name = "move_base_node/make_plan";
//等待服务空闲，如果已经在运行这个服务，会等到运行结束。
while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
ROS_INFO("Waiting for service move_base/make_plan to become available");
}
/*初始化客户端，(nav_msgs/GetPlan)
Allows an external user to ask for a plan to a given pose from move_base without causing move_base to execute that plan.
允许用户从move_base 请求一个plan，并不会导致move_base 执行此plan
*/
/*
 *
 *
ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
if (!serviceClient) {
ROS_FATAL("Could not initialize get plan service from %s",
serviceClient.getService().c_str());
return -1;
}
nav_msgs::GetPlan srv;
//请求服务：规划路线
fillPathRequest(srv.request);
if (!serviceClient) {
ROS_FATAL("Persistent service connection to %s failed",
serviceClient.getService().c_str());
return -1;
}
ROS_INFO("conntect to %s",serviceClient.getService().c_str());
callPlanningService(serviceClient, srv);
}

*/

