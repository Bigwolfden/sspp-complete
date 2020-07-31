#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sspp/sspp_srv.h>

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //SSPP
  sspp::sspp_srv planningService;
  geometry_msgs::Pose start,end;

  //Get the starting and ending positions from the ROS params
  nh_private.param("start_x", start.position.x, start.position.x);
  nh_private.param("start_y", start.position.y, start.position.y);
  nh_private.param("start_z", start.position.z, start.position.z);
  nh_private.param("end_x", end.position.x, end.position.x);
  nh_private.param("end_y", end.position.y, end.position.y);
  nh_private.param("end_z", end.position.z, end.position.z);

  ROS_INFO("Started waypoint_publisher.");
 
  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  //Wait for path Planner serivce to be ready
  ros::service::waitForService("/sspp_planner",ros::Duration(10.0));
  ROS_INFO("Planning Service Provider Ready");

  ros::Rate rate(20.0);
  
  while (ros::ok())
  {
    /*
    start.position.x = -2.12;
    start.position.y =  1.2;
    start.position.z =  0.2;

    end.position.x = 2;
    end.position.y = 2;
    end.position.z = 1;
    */
    planningService.request.header.stamp = ros::Time::now();
    planningService.request.header.seq = 1;
    planningService.request.header.frame_id = "world";
    planningService.request.start = start;
    planningService.request.end   = end;
    planningService.request.grid_start = start;

    if(ros::service::call("/sspp_planner", planningService))
    {
      ROS_INFO("Path Found");
      for (int i = 0; i < planningService.response.path.size(); i++)
      {
        std::cout<<"Path x:"<<planningService.response.path[i].position.x
                <<" y:"<<planningService.response.path[i].position.y
                <<" z:"<<planningService.response.path[i].position.z
                <<"\n";

        tf::Pose pose;
        tf::poseMsgToTF(planningService.response.path[i], pose);
        double yaw = tf::getYaw(pose.getRotation());
        std::cout << "Yaw: " << yaw << '\n';
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
        //Add the current waypoint to the vector
        waypoints.push_back(WaypointWithTime(5, planningService.response.path[i].position.x, planningService.response.path[i].position.y, planningService.response.path[i].position.z, yaw));
      }
      tf::Pose pose;
      tf::poseMsgToTF(end, pose);
      double yaw = tf::getYaw(pose.getRotation());
      //Add the ending waypoint too
      waypoints.push_back(WaypointWithTime(5, end.position.x, end.position.y, end.position.z, yaw));
      break;
    }
    else
    {
      ROS_INFO("No Path Found or planner not ready!");
      ros::Duration(1.0).sleep();
    }
    ros::spinOnce();
    rate.sleep();
  }

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  //ros::Duration(30).sleep();

  ROS_INFO("Start publishing waypoints.");

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(waypoints.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }
  wp_pub.publish(msg);

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
