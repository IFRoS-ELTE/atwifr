#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_gt_publisher");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_gt", 10);
	tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener transformListener_(tf_buffer_);


	// initial position
	double x = 0.0; 
	double y = 0.0;
	double z = 0.0;
	double xa = 0.0; 
	double ya = 0.0;
	double za = 0.0;

	// velocity
	double vx = 0.0;
	double vy = 0.0;
	double vz = 0.0;
	double vxa = 0.0;
	double vya = 0.0;
	double vza = 0.0;

	ros::Time current_time;
	ros::Time last_time;
	ros::Time last_vel_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);

	// const double degree = M_PI/180;

	// message declarations

	nav_msgs::Odometry prev_odom;

	bool init = true;

	while (ros::ok()) {
		current_time = ros::Time::now(); 

		geometry_msgs::TransformStamped transformTf;
        ros::Time timeStamp;
        timeStamp.fromSec(0.0);
        try
        {
            transformTf = tf_buffer_.lookupTransform("odom", "base_footprint", timeStamp, ros::Duration(5.0));
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            std::cerr << "Failed to get a transform" << std::endl;
            continue;
        }
        float x_coord = transformTf.transform.translation.x;
        float y_coord = transformTf.transform.translation.y;
        float z_coord = transformTf.transform.translation.z;

		tf::Quaternion q(
			transformTf.transform.rotation.x,
			transformTf.transform.rotation.y,
			transformTf.transform.rotation.z,
			transformTf.transform.rotation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		// float droll = roll - xa;
		// float dpitch = pitch - ya;
		float dyaw = yaw - za;

		double dx_glob = x_coord - x;
		double dy_glob = y_coord - y;
		double dx =   std::cos(yaw)*dx_glob + std::sin(yaw) * dy_glob;
		double dy = - std::sin(yaw)*dx_glob + std::cos(yaw) * dy_glob;
		// double dz = z_coord - z;

		// xa = roll;
		// ya = pitch;
		
		// z = z_coord;


		// transformTf.transform.rotation.
		ros::Time current_time = transformTf.header.stamp;

		double dvt = (current_time - last_vel_time).toSec();
		if (!init && dvt > 0.2){
			double dt = (current_time - last_vel_time).toSec();
			double  vx_cur = dx/dt;
			double  vy_cur = dy/dt;
			// double  vz_cur = dz/dt;
			// double vxa_cur = droll/dt;
			// double vya_cur = dpitch/dt;
			double vza_cur = dyaw/dt;
			// TODO: add some filtering
			vx = 0.8*vx + 0.2*vx_cur;
			vy = 0.8*vy + 0.2*vy_cur;
			// vz = vz_cur;
			// vxa = vxa_cur;
			// vya = vya_cur;
			vza = 0.8*vza + 0.2*vza_cur;
			last_vel_time = current_time;

			x = 0.8*x + 0.2*x_coord;
			y = 0.8*y + 0.2*y_coord;
			za = 0.8*za + 0.2*yaw;
		} else {
			init = false;
			x = x_coord;
			y = y_coord;
			za = yaw;
		}

		// double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		// double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		// double delta_th = vth * dt;

		// x += delta_x;
		// y += delta_y;
		// th += delta_th;

		// geometry_msgs::Quaternion odom_quat;	
		// odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

		// update transform
		// odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

		//filling the odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";

		// position
		odom.pose.pose.position.x = x_coord;
		odom.pose.pose.position.y = y_coord;
		odom.pose.pose.position.z = z_coord;
		odom.pose.pose.orientation = transformTf.transform.rotation;

		//velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.linear.z = vz;
		odom.twist.twist.angular.x = vxa;
		odom.twist.twist.angular.y = vya;
		odom.twist.twist.angular.z = vza;

		last_time = current_time;

		// publishing the odometry and the new tf
		// broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);

		loop_rate.sleep();
	}
	return 0;
}