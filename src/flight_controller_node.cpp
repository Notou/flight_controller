
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <drone_operation_msgs/flightAction.h>
#include <bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <mav_msgs/CommandTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class FlightController{
private:
	typedef actionlib::ActionServer<drone_operation_msgs::flightAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	FlightController(ros::NodeHandle &n) :
	node_(n),
	action_server_(node_, "flight_action",
	boost::bind(&FlightController::goalCB, this, _1),
	boost::bind(&FlightController::cancelCB, this, _1),
	false),
	has_active_goal_(false),
	move_group(PLANNING_GROUP),
	joint_group_positions(7,0) //Init the vector that will contain the goal position. 7 values: 3 for position and 4 for orientation (quaternion)
	{
		ROS_INFO("\n\n Node ready \n\n");
		created=0;
		command_pub = node_.advertise<mav_msgs::CommandTrajectory>("/quad/command/trajectory", 10);
		goal_pub = node_.advertise<geometry_msgs::Pose>("/feedback/currentGoal", 10);
		odometry_sub = node_.subscribe("/quad/odometry", 1, &FlightController::OdometryCallback, this);
    image_sub = node_.subscribe("/left_rgb_rect/image_rect_color", 1, &FlightController::PhotoCallback, this);
    depth_sub = node_.subscribe("/depth_map/image", 1, &FlightController::DepthCallback, this);
    range_sub = node_.subscribe("/ultrasound", 1, &FlightController::RangeCallback, this);
    emergency_sub = node_.subscribe("/emergency", 1, &FlightController::EmergencyCallback, this);
		landed_sub = node_.subscribe("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", 1, &FlightController::LandedCallback, this);
		ROS_INFO("\n\n Node ready \n\n");
		node_.getParam("flight_controller/distance_to_emplacement", distance_to_emplacement);
		node_.getParam("flight_controller/workspace_low_point/X", workspace_min_x);
		node_.getParam("flight_controller/workspace_low_point/Y", workspace_min_y);
		node_.getParam("flight_controller/workspace_low_point/Z", workspace_min_z);
		node_.getParam("flight_controller/workspace_high_point/X", workspace_max_x);
		node_.getParam("flight_controller/workspace_high_point/Y", workspace_max_y);
		node_.getParam("flight_controller/workspace_high_point/Z", workspace_max_z);
		node_.getParam("flight_controller/waiting_time", waiting_time);
		node_.getParam("flight_controller/planner", planner_id);
		node_.getParam("flight_controller/lower_destination_point", lower_destination_point);
		ROS_INFO("\n\n Node ready \n\n");
		action_server_.start();
    landed = true;
		poseTaken = photoTaken = depthTaken = rangeTaken = snapshot = false;

		command.position.x = 0.0;	// m
		command.position.y = 0.0;	// m
		command.position.z = 0.0;	// m
		command.yaw = 0.0;	// rad

		// Hack for sending button statuses
		command.snap.x = 0.0;	// takeoff
		command.snap.y = 0.0;	// land
		command.snap.z = 0.0;	// 3D navigation
		command.jerk.x = 0.0;	// enable GPS
		command.jerk.y = 0.0;	// enable mission

		ROS_INFO("\n\n Node ready \n\n");
	}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Subscriber odometry_sub;
  ros::Subscriber image_sub;
  ros::Subscriber depth_sub;
  ros::Subscriber range_sub;
  ros::Subscriber emergency_sub;
	ros::Subscriber landed_sub;

	ros::Publisher goal_pub;
	ros::Publisher command_pub;
	mav_msgs::CommandTrajectory command;

	pthread_t trajectoryExecutor;
	int created;

	bool has_active_goal_;
	GoalHandle active_goal_;
	geometry_msgs::PoseArray_<std::allocator<void> > toExecute;

	std::string planner_id;
	double distance_to_emplacement = 1.5; //In meters
	double workspace_min_x, workspace_min_y, workspace_min_z, workspace_max_x, workspace_max_y, workspace_max_z;
	double waiting_time = 8;
	double lower_destination_point = 0;
  bool landed, flying;
  drone_operation_msgs::flightFeedback feedback;
  bool snapshot, photoTaken, depthTaken, poseTaken, rangeTaken;

	std::string PLANNING_GROUP = "Quad_base";
	std::vector<double> joint_group_positions;
	moveit::planning_interface::MoveGroupInterface move_group;


	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(created){
				printf("Stop thread \n");
				pthread_cancel(trajectoryExecutor);

				move_group.stop();
				backToBase();
				land();
				created=0;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_){
			gh.setRejected();
		}
		else{
			gh.setAccepted();
			active_goal_ = gh;
			has_active_goal_ = true;
			toExecute = gh.getGoal()->objectives;

			if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
				created=1;
				printf("Thread for trajectory execution created \n");
			} else {
				printf("Thread creation failed! \n");
			}
		}
	}

	static void* threadWrapper(void* arg) {
		FlightController * mySelf=(FlightController*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    if (snapshot && !poseTaken) {
      //take pose
      feedback.pose = odometry_msg->pose.pose;
      ROS_INFO("odom snapshot taken.");
      poseTaken = true;
    }
	}

  void PhotoCallback(const sensor_msgs::ImageConstPtr& image_msg){
    if (snapshot && !photoTaken) {
      //take photo
      feedback.snapshot = *image_msg;
      ROS_INFO("image snapshot taken.");
      photoTaken = true;
    }
  }

  void DepthCallback(const sensor_msgs::ImageConstPtr& image_msg){
    if (snapshot && !depthTaken) {
      //take depth
      feedback.depthmap = *image_msg;
      ROS_INFO("depth snapshot taken.");
      depthTaken = true;
    }
  }

  void RangeCallback(const sensor_msgs::RangeConstPtr& range_msg){
    if (snapshot && !rangeTaken) {
      //take range
      feedback.distance = *range_msg;
      ROS_INFO("Range snapshot taken. Value=%f", feedback.distance.range);
			feedback.distance.range = feedback.distance.range - 0.05;
			ROS_INFO("Correcting range. New value=%f", feedback.distance.range);
      rangeTaken = true;
    }
  }

  void EmergencyCallback(const std_msgs::EmptyConstPtr& empty_ptr){
		if (has_active_goal_) {
			// Stops the controller.
			if(created){
				printf("Stop thread \n");
				pthread_cancel(trajectoryExecutor);

				move_group.stop();
				land();
				created=0;
			}

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
  }

	void LandedCallback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChangedPtr& state_ptr){
		if (state_ptr->state == bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_landed) {
			landed = true;
		}else{
			landed = false;
		}
		if (state_ptr->state == bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_flying || state_ptr->state == bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_hovering) {
			flying = true;
		}else{
			flying = false;
		}
	}

	void executeTrajectory(){
		if(toExecute.poses.size()>0){
      if (landed) {
				bool errorHasOccured = false;
				bool success = false;

				geometry_msgs::Pose_<std::allocator<void> > waypoint;
				waypoint.position.z = 1;
				waypoint.position.x = 1.5;
				goal_pub.publish(waypoint);
				takeOff();
				ros::Duration(waiting_time).sleep();
				//fullRotation();

				move_group.allowReplanning(true);
				move_group.setPlannerId(planner_id);
				ROS_INFO("Workspace set between [%f, %f, %f] and [%f, %f, %f]", workspace_min_x, workspace_min_y, workspace_min_z, workspace_max_x, workspace_max_y, workspace_max_z);
				move_group.setWorkspace(workspace_min_x, workspace_min_y, workspace_min_z, workspace_max_x, workspace_max_y, workspace_max_z);

        for(int k=0; k<toExecute.poses.size(); k++){
          geometry_msgs::Pose_<std::allocator<void> > waypoint=toExecute.poses[k];
					waypoint.position.z -= lower_destination_point;
					//Set target
					joint_group_positions = setPositions(joint_group_positions, waypoint);
					move_group.setJointValueTarget(joint_group_positions);
					double yawAngle = atan2(-waypoint.orientation.y,-waypoint.orientation.x);
					double yshift = 0.1 * cos(yawAngle);
					double xshift = 0.1 * sin(yawAngle);
					waypoint.position.x -= xshift;
					waypoint.position.y -= yshift;
					goal_pub.publish(waypoint);
					ROS_INFO("Going to point: x:%f, y:%f, z:%f", joint_group_positions[0], joint_group_positions[1], joint_group_positions[2]);
					success = (bool)move_group.move();
					ROS_INFO("Movement %s", success ? "SUCCEDED" : "FAILED");
					if(!success){
						ROS_ERROR("Movement has failed. Returning to base");
						errorHasOccured = true;
						break;
					}
					ros::Duration(1).sleep(); //Wait for drone to stabilise
          //Taking a snapshot
          snapshot = true;
          while(!(poseTaken && photoTaken && depthTaken && rangeTaken)){
            ros::Duration(0.05).sleep();
          }
          poseTaken = photoTaken = depthTaken = rangeTaken = snapshot = false;
					ROS_INFO("Snapshot Taken");
					feedback.objective = waypoint;
          active_goal_.publishFeedback(feedback);
					ros::Duration(0.1).sleep();
        }
				//fullRotation();
				success = backToBase();
				land();
				if (!success) {
					errorHasOccured = true;
				}

				if (errorHasOccured) {
					active_goal_.setAborted();
				}else{
					active_goal_.setSucceeded();
					ROS_INFO("Flight successful");
				}
      }else{
				active_goal_.setAborted();
				has_active_goal_=false;
				created=0;
			}
		}
		has_active_goal_=false;
		created=0;
	}

	bool backToBase(){
		//Get back to origin point
		joint_group_positions[0] = 0;  // x
		joint_group_positions[1] = 0;  // y
		joint_group_positions[2] = 1;  // z
		joint_group_positions[3] = 0;  // roll
		joint_group_positions[4] = 0;  // pitch
		joint_group_positions[5] = 0;  // yaw
		joint_group_positions[6] = 1;
		move_group.setJointValueTarget(joint_group_positions);

		bool success = (bool)move_group.move();
		ROS_INFO("Return to base plan %s. Landing", success ? "" : "FAILED");

		return success;
	}

	void land(){
		command.snap.x = 0.0;	// takeoff
		command.snap.y = 1;	// land
		command.snap.z = 1;	// 3D navigation
		command_pub.publish(command);
		ROS_INFO("Landing");
		waitForLanded();
		ROS_INFO("Landed");
	}

	void takeOff(){
		command.snap.x = 1;	// takeoff
		command.snap.y = 0.0;	// land
		command.snap.z = 1;	// 3D navigation
		command_pub.publish(command);
		ROS_INFO("Taking off");
		waitForFlying();
		ROS_INFO("Flying");
	}

	void waitForFlying(){
		while(ros::ok()){
			ros::Duration(0.3).sleep();
			if (flying) {
				return;
			}
		}
	}

	void waitForLanded(){
		while(ros::ok()){
			ros::Duration(0.3).sleep();
			if (landed) {
				return;
			}
		}
	}

	void fullRotation(){ //Rotate one full turn
		command.snap.x = 0;	// takeoff
		command.snap.y = 0.0;	// land
		command.snap.z = 0;	// 3D navigation
		command.yaw = 1;
		command_pub.publish(command);

		ros::Duration(3).sleep();

		command.snap.x = 0;	// takeoff
		command.snap.y = 0.0;	// land
		command.snap.z = 0;	// 3D navigation
		command.yaw = 0;
		command_pub.publish(command);
	}

	std::vector<double> setPositions(std::vector<double> joint_group_positions, geometry_msgs::Pose waypoint){
		tf::Quaternion q;

		geometry_msgs::Quaternion qMsg;
		double yawAngle = atan2(-waypoint.orientation.y,-waypoint.orientation.x);

		joint_group_positions[0] = waypoint.position.x + waypoint.orientation.x * distance_to_emplacement;  // x
		joint_group_positions[1] = waypoint.position.y + waypoint.orientation.y * distance_to_emplacement;  // y
		joint_group_positions[2] = waypoint.position.z + waypoint.orientation.z * distance_to_emplacement;  // z
		//q.setEuler(-waypoint.orientation.x, -waypoint.orientation.y, -waypoint.orientation.z);
		//tf::quaternionTFToMsg(q,qMsg);
		ROS_INFO("Conversion 2d to angle: %f %f  => %f", -waypoint.orientation.x, -waypoint.orientation.y, yawAngle);
		qMsg = tf::createQuaternionMsgFromYaw(yawAngle);
		joint_group_positions[3] = qMsg.x;  // roll
		joint_group_positions[4] = qMsg.y;  // pitch
		joint_group_positions[5] = qMsg.z;  // yaw
		joint_group_positions[6] = qMsg.w;

		return joint_group_positions;
	}


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "flight_controller_node");
	ros::NodeHandle node;//("~");
	FlightController control(node);

	ros::spin();

	return 0;
}
