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

    service = nodeHandle_.advertiseService("plan_tsp", &TSPsolver::solve_tsp_callback, this);

    std::cout << "Node initialized" << std::endl;

    return true;
  }

  void status_cb(const actionlib_msgs::GoalStatusArray &msg)
  {
    int time_diff = (msg.header.stamp - last_status).toSec();
    if (time_diff < 5)
      return;
    last_status = msg.header.stamp;

    if(status == "initialized"){
      bool result = send_goal(msg);
      return;
    }

    if (msg.status_list.size() == 0)
      bool result = send_goal(msg);
      return;

    if ((msg.status_list[0].status == 3) && (status == "following")){
      status = "reached";
      points_solved_.erase(points_solved_.begin());
      bool result = send_goal(msg);
      return;
    }
    if ((msg.status_list[0].status != 3) && (status == "reached")){
      status = "following";
      bool result = send_goal(msg);
      return;
    }
    // bool result = send_goal(msg);
    
  }

  // bool send_goal(const actionlib_msgs::GoalStatusArray &msg){
  //   std::cout << "received callback" << std::endl;
  //   if (points_solved_.size() == 0){
  //     std::cout << "no points to lead. last one reached" << std::endl;
  //     status = "initialized";
  //     return false;
  //   }
  //   // if (status == "reached"){
  //     std::cout << "sending the goal" << std::endl;
  //     float x = points_solved_[0].first;
  //     float y = points_solved_[0].second;

  //     move_base_msgs::MoveBaseActionGoal goalMsg;
  //     goalMsg.header.frame_id = "odom";
  //     goalMsg.header.stamp=ros::Time::now();
  //     goalMsg.header.seq=0;
  //     goalMsg.goal_id.id = "tsp";
  //     goalMsg.goal_id.stamp = ros::Time::now();
  //     goalMsg.goal.target_pose.header = msg.header;
  //     goalMsg.goal.target_pose.header.frame_id = "odom";
  //     goalMsg.goal.target_pose.pose.position.x = x;
  //     goalMsg.goal.target_pose.pose.position.y = y;
  //     goalMsg.goal.target_pose.pose.position.z = 0;
  //     goalMsg.goal.target_pose.pose.orientation.x = 0;
  //     goalMsg.goal.target_pose.pose.orientation.y = 0;
  //     goalMsg.goal.target_pose.pose.orientation.z = 0;
  //     goalMsg.goal.target_pose.pose.orientation.w = 1;
  //     // std::cout << "publishing "
  //     pub_.publish(goalMsg);
  //     // status = "following";
  //   // }
  //     // std::cout << "here" << std::endl;
  //   return true;
  // }
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