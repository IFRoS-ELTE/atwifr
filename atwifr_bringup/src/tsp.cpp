#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include "nav_msgs/Path.h"
// #include <std_srvs/SetBoolResponse.h>
// #include <utility>

using pairf = std::pair<float, float>;

class ElevationMapper{
  public:
    ElevationMapper(ros::NodeHandle nodeHandle, ros::NodeHandle privateNodeHandle): nodeHandle_(nodeHandle), pnh_(privateNodeHandle){
      readPearameters();
      
    }
    ~ElevationMapper(){
      nodeHandle_.shutdown();
    }

  private:
    ros::NodeHandle nodeHandle_;
    ros::NodeHandle pnh_;
    
    std::string path_topic_ = "path";
    std::string points_out_topic_ = "points_out";

    
    std::vector<pairf> points_;



    // tf::TransformListener transformListener_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    ros::ServiceServer service;

    //methods

    bool readPearameters(){
      pnh_.getParam("path_topic", path_topic_);
      pnh_.getParam("points_out_topic", points_out_topic_);

      std::cout << "path_topic " << path_topic_ << std::endl;

    //   sub_ = nodeHandle_.subscribe(path_topic_, 1, &ElevationMapper::cloud_cb, this);

      // Create a ROS publisher for the output point cloud
      pub_ = nodeHandle_.advertise<nav_msgs::Path>(points_out_topic_, 1);

      service = nodeHandle_.advertiseService("plan_salesman", &ElevationMapper::frontrier_cb, this);

      std::cout << "Node initialized" << std::endl;

      return true;
    }

    bool frontrier_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
   
  
    }



};

    




int
main (int argc, char** argv)
{
  ros::init (argc, argv, "my_tsp");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ElevationMapper mapper(nh, pnh);

  // Spin
  ros::spin ();
}