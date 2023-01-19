#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include "nav_msgs/Path.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_srvs/Empty.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/metric_tsp_approx.hpp>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>
using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;
using Graph =
    boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS,
                          boost::property<boost::vertex_index_t, int>,
                          boost::property<boost::edge_weight_t, double>,
                          boost::no_property>;

using VertexDescriptor = Graph::vertex_descriptor;



using pairf = std::pair<float, float>;

class TSPsolver
{
public:
  TSPsolver(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle) : nodeHandle_(nodeHandle), pnh_(privateNodeHandle)
  {
    readPearameters();
    status = "initialized";
    last_status = ros::Time::now();
  }
  ~TSPsolver()
  {
    nodeHandle_.shutdown();
  }

private:
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle pnh_;

  std::string status = "created";

  std::string status_topic = "/move_base/status";
  std::string new_goal_topic = "/move_base_simple/goal";

  std::vector<pairf> points_;
  std::vector<pairf> points_solved_;

  ros::Publisher pub_;
  ros::Subscriber sub_;
    ros::Publisher pub_path_;
    ros::Publisher pub_frontier_;

  ros::Time last_status;

  ros::ServiceServer service;

  std::vector<pairf> verticies_to_visit = {};

  // methods

  bool readPearameters()
  {
    pnh_.getParam("status_topic", status_topic);
    pnh_.getParam("new_goal_topic", new_goal_topic);

    std::cout << "status_topic " << status_topic << std::endl;

    // Create a ROS publisher for the output point cloud
    pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>(new_goal_topic, 1);
    pub_path_ = nodeHandle_.advertise<nav_msgs::Path>("tsp_path", 1);
    pub_frontier_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("all_points", 1);
    service = nodeHandle_.advertiseService("plan_tsp", &TSPsolver::solve_tsp_callback, this);

    std::cout << "Node initialized" << std::endl;

    return true;
  }

  void status_cb(const actionlib_msgs::GoalStatusArray &msg)
  {
    // int time_diff = (msg.header.stamp - last_status).toSec();
    // if (time_diff < 5)
    //   return;
    // last_status = msg.header.stamp;

    // std::cout << status << std::endl;

    if(status == "initialized"){
      std::cout << "initialized goal" << std::endl;
      bool result = send_goal(msg);
      status = "following";
      return;
    }

    if (msg.status_list.size() == 0){
      std::cout << " skipping" << std::endl;
      // bool result = send_goal(msg);
      return;
    }

    if ((msg.status_list[0].status == 3) && (status == "following") ){
      std::cout << " reached and sending next goal" << std::endl;
      status = "reached";
      points_solved_.erase(points_solved_.begin());
      bool result = send_goal(msg);
      return;
    }
    if ((msg.status_list[0].status != 3) && (status == "reached")){
      std::cout << " sent next goal and following " << std::endl;
      status = "following";
      bool result = send_goal(msg);
      return;
    }
    // std::cout << "hi " << status << std::endl;
    
  }

  bool publish_pcl(std::vector<pairf> points)
    {
        PointCloudType::Ptr pointCloud(new PointCloudType);
        pointCloud->header.frame_id = "odom";
        pointCloud->header.stamp = ros::Time::now().toNSec() / 1000;

        for (auto &pt : points)
        {
            pcl::PointXYZI p(1);
            p.x = pt.first;
            p.y = pt.second;
            p.z = 0;
            pointCloud->insert(pointCloud->end(), p);
        }

        publish_pcl(pointCloud, pub_frontier_);
        return true;
    }

    bool publish_pcl(PointCloudType::Ptr pointCloud, ros::Publisher pub)
    {

        pcl::PCLPointCloud2 pcl_pc;
        sensor_msgs::PointCloud2 output;
        pcl::toPCLPointCloud2(*pointCloud, pcl_pc);
        pcl_conversions::fromPCL(pcl_pc, output);

        // Publish the data
        pub.publish(output);
        return true;
    }

    bool publish_path(std::vector<pairf>& path_in)
    {
        if (path_in.size() > 0)
        {
            nav_msgs::Path path;
            path.header.frame_id = "odom";
            path.header.stamp = ros::Time::now();

            std::vector<geometry_msgs::PoseStamped> poses;
            int seq = 0;
            for (auto &xy_ind : path_in)
            {
                seq++;
                pairf xy_coordinate = xy_ind;
                geometry_msgs::PoseStamped *pose = new geometry_msgs::PoseStamped();
                pose->header = path.header;
                pose->header.seq = seq;
                geometry_msgs::Pose posee;
                geometry_msgs::Quaternion orientation;
                orientation.w = 0;
                orientation.x = 0;
                orientation.y = 0;
                orientation.z = 0;
                posee.orientation = orientation;
                geometry_msgs::Point point;
                point.x = xy_coordinate.first;
                point.y = xy_coordinate.second;
                point.z = 0;
                posee.position = point;
                pose->pose = posee;
                poses.push_back(*pose);
            }
            path.poses = poses;

            // // Publish the data
            pub_path_.publish(path);
        }
        return true;
    }

  bool send_goal(const actionlib_msgs::GoalStatusArray &msg){
    std::cout << "received callback" << std::endl;
    if (points_solved_.size() == 0){
      std::cout << "no points to lead. last one reached" << std::endl;
      status = "initialized";
      return false;
    }
    // if (status == "reached"){
      std::cout << "sending the goal" << std::endl;
      float x = points_solved_[0].first;
      float y = points_solved_[0].second;

      geometry_msgs::PoseStamped goalMsg;
      goalMsg.header.frame_id = "odom";
      goalMsg.header.stamp=ros::Time::now();
      // goalMsg.header.seq=0;
      // goalMsg.goal_id.id = "tsp";
      // goalMsg.goal_id.stamp = ros::Time::now();
      goalMsg.pose.position.x = x;
      goalMsg.pose.position.y = y;
      goalMsg.pose.position.z = 0;
      goalMsg.pose.orientation.x = 0;
      goalMsg.pose.orientation.y = 0;
      goalMsg.pose.orientation.z = 0;
      goalMsg.pose.orientation.w = 1;
      // std::cout << "publishing "
      pub_.publish(goalMsg);
      // status = "following";
    // }
      // std::cout << "here" << std::endl;
    return true;
  }

  bool solve_tsp_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
  {
    std::vector<pairf> points{
        {8., 5.}, // these can be randomized
        {11., 10.},
        {7., 14.},
        {2., 15.},
    };
    points_ = points;
    Graph graph;
    for (auto i = 0u; i < points.size(); ++i)
    {
      add_vertex(i, graph);
    }

    for (auto i = 0u; i < points.size(); ++i)
    {
      auto va = vertex(i, graph);

      // undirected, so only need traverse higher vertices for connections
      for (auto j = i + 1; j < points.size(); ++j)
      {
        auto vb = vertex(j, graph);

        auto const ax = points.at(i).first;
        auto const ay = points.at(i).second;
        auto const bx = points.at(j).first;
        auto const by = points.at(j).second;
        auto const dx = bx - ax;
        auto const dy = by - ay;

        add_edge(va, vb, sqrt(dx * dx + dy * dy), graph); // weight is euclidean distance
      }
    }

    print_graph(graph);

    std::vector<VertexDescriptor> tsp_path(num_vertices(graph)); // VertexDescriptor is adjacency_list::vertex_descriptor
    metric_tsp_approx_tour(graph, back_inserter(tsp_path));

    auto idmap = get(boost::vertex_index, graph);
    for (auto vd : tsp_path)
    {
      if (vd != graph.null_vertex())
      {
        auto x = points.at(idmap[vd]).first;
        auto y = points.at(idmap[vd]).second;
        points_solved_.push_back(pairf(x, y));

        std::cout << " {" << x << "," << y << "}";
      }
    }

    // status = "following";

    sub_ = nodeHandle_.subscribe(status_topic, 1, &TSPsolver::status_cb, this);
    publish_pcl(points_);
    publish_path(points);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tsp");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TSPsolver mapper(nh, pnh);

  // Spin
  ros::spin();
}