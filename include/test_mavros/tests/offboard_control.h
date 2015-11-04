/**
 * @brief Offboard control test
 * @file offboard_control.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Andre Nguyen <andre-phu-van.nguyen@polymtl.ca>
 *
 * @addtogroup sitl_test
 * @{
 */
/*
 * Copyright 2015 Nuno Marques, Andre Nguyen.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <array>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <test_mavros/sitl_test/sitl_test.h>


#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>

#include <boost/thread.hpp>

#include <tf/transform_datatypes.h>


typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

namespace testsetup {
/**
 * @brief Offboard controller tester
 *
 * Tests offboard position, velocity and acceleration control
 *
 */

typedef enum {
	POSITION,
	VELOCITY,
	ACCELERATION
} control_mode;

typedef enum {
	SQUARE,
	CIRCLE,
	EIGHT,
	ELLIPSE
} path_shape;

class OffboardControl {
public:
	OffboardControl() :
		nh_sp(test.nh),
		local_pos_sp_pub(nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10)),
		vel_sp_pub(nh_sp.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10)),
		local_pos_sub(nh_sp.subscribe("/odometry/filtered", 10, &OffboardControl::local_pos_cb, this)),		
		//local_pos_sub(nh_sp.subscribe("/gazebo/model_states", 10, &OffboardControl::local_pos_cb, this)),
		//modelClient(nh_sp.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state")),
		threshold(threshold_definition())
	{ };

	void init() {
		/**
		 * @brief Setup of the test conditions
		 */
		test.setup(nh_sp);
		rate = test.rate;
		use_pid = test.use_pid;
		num_of_tests = test.num_of_tests;



		if (use_pid) {
			/**
			 * @note some of these are based on values defaulted @ https://bitbucket.org/enddl22/ardrone_side_project/
			 * tweaks to them so to get a better velocity response are welcomed!
			 */

			// Linear velocity PID gains and bound of integral windup
			nh_sp.param("linvel_p_gain", linvel_p_gain, 0.4);
			nh_sp.param("linvel_i_gain", linvel_i_gain, 0.05);
			nh_sp.param("linvel_d_gain", linvel_d_gain, 0.12);
			nh_sp.param("linvel_i_max", linvel_i_max, 0.1);
			nh_sp.param("linvel_i_min", linvel_i_min, -0.1);

			// Yaw rate PID gains and bound of integral windup
			nh_sp.param("yawrate_p_gain", yawrate_p_gain, 0.011);
			nh_sp.param("yawrate_i_gain", yawrate_i_gain, 0.00058);
			nh_sp.param("yawrate_d_gain", yawrate_d_gain, 0.12);
			nh_sp.param("yawrate_i_max", yawrate_i_max, 0.005);
			nh_sp.param("yawrate_i_min", yawrate_i_min, -0.005);

			// Setup of the PID controllers
			pid.setup_linvel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min, nh_sp);
			pid.setup_yawrate_pid(yawrate_p_gain, yawrate_i_gain, yawrate_d_gain, yawrate_i_max, yawrate_i_min, nh_sp);
		}

		/**
		 * @brief Setpoint control mode selector
		 *
		 * Available modes:
		 * - position
		 * - velocity
		 * - acceleration
		 */
		std::string mode_;
		nh_sp.param<std::string>("mode", mode_, "position");

		/**
		 * @brief Setpoint path shape selector
		 *
		 * Available shapes:
		 * - square
		 * - circle
		 * - eight
		 * - ellipse (in 3D space)
		 */
		std::string shape_;
		nh_sp.param<std::string>("shape", shape_, "square");

		if (mode_ == "position")
			mode = POSITION;
		else if (mode_ == "velocity")
			mode = VELOCITY;
		else if (mode_ == "acceleration")
			mode = ACCELERATION;
		else {
			ROS_ERROR_NAMED("sitl_test", "Control mode: wrong/unexistant control mode name %s", mode_.c_str());
			return;
		}

		if (shape_ == "square")
			shape = SQUARE;
		else if (shape_ == "circle")
			shape = CIRCLE;
		else if (shape_ == "eight")
			shape = EIGHT;
		else if (shape_ == "ellipse")
			shape = ELLIPSE;
		else {
			ROS_ERROR_NAMED("sitl_test", "Path shape: wrong/unexistant path shape name %s", shape_.c_str());
			return;
		}
	}

	/* -*- main routine -*- */

	void spin(int argc, char *argv[]) {
		init();
		ros::Rate loop_rate(rate);

		ROS_INFO("SITL Test: Offboard control test running!");

		if (mode == POSITION) {
			ROS_INFO("Position control mode selected.");
		}
		else if (mode == VELOCITY) {
			ROS_INFO("Velocity control mode selected.");
		}
		else if (mode == ACCELERATION) {
			ROS_INFO("Acceleration control mode selected.");
			ROS_ERROR_NAMED("sitl_test", "Control mode: acceleration control mode not supported in PX4 current Firmware.");
			/**
			 * @todo: lacks firmware support, for now
			 */
			return;
		}

		if (shape == SQUARE) {
			ROS_INFO("Test option: square-shaped path...");
			square_path_motion(loop_rate, mode);
		}
		else if (shape == CIRCLE) {
			ROS_INFO("Test option: circle-shaped path...");
			circle_path_motion(loop_rate, mode);
		}
		else if (shape == EIGHT) {
			ROS_INFO("Test option: eight-shaped path...");
			eight_path_motion(loop_rate, mode);
		}
		else if (shape == ELLIPSE) {
			ROS_INFO("Test option: ellipse-shaped path...");
			ellipse_path_motion(loop_rate, mode);
		}
	}

private:
	TestSetup test;
	pidcontroller::PIDController pid;

	double rate;
	bool use_pid;
	int num_of_tests; //TODO: find a way to use this...

	double linvel_p_gain;
	double linvel_i_gain;
	double linvel_d_gain;
	double linvel_i_max;
	double linvel_i_min;

	double yawrate_p_gain;
	double yawrate_i_gain;
	double yawrate_d_gain;
	double yawrate_i_max;
	double yawrate_i_min;

	control_mode mode;
	path_shape shape;

	ros::NodeHandle nh_sp;
	ros::Publisher local_pos_sp_pub;
	ros::Publisher vel_sp_pub;
	ros::Subscriber local_pos_sub;

	ros::ServiceClient modelService;

	//geometry_msgs::PoseStamped localpos, ps;
	geometry_msgs::PoseStamped ps;
	nav_msgs::Odometry localpos;
	gazebo_msgs::ModelState localModelState;
	geometry_msgs::Twist vs;

	Eigen::Vector3d current;

	std::array<double, 100> threshold;

	/* -*- helper functions -*- */

	/**
	 * @brief Defines single position setpoint
	 */
	Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z){
		/** @todo Give possibility to user define amplitude of movement (square corners coordinates)*/
		return Eigen::Vector3d(tr_x * 2.0f, tr_y * 2.0f, tr_z * 1.0f);	// meters
	}

	/**
	 * @brief Defines circle path
	 */
	Eigen::Vector3d circle_shape(double angle){
		/** @todo Give possibility to user define amplitude of movement (circle radius)*/
		double r = 5.0f;	// 5 meters radius

		return Eigen::Vector3d(r * cos(angles::from_degrees(angle)),
				r * sin(angles::from_degrees(angle)),
				0.0f);
	}
     
	/**
	 * @brief Defines Gerono lemniscate path
	 */
	Eigen::Vector3d eight_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (vertical tangent size)*/
		double a = 5.0f;	// vertical tangent with 5 meters size

		return Eigen::Vector3d(a * cos(angles::from_degrees(angle)),
				a * sin(angles::from_degrees(angle)) * cos(angles::from_degrees(angle)),
				1.0f);
	}

	/**
	 * @brief Defines ellipse path
	 */
	Eigen::Vector3d ellipse_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (tangent sizes)*/
		double a = 5.0f;	// major axis
		double b = 2.0f;	// minor axis

		// rotation around y-axis
		return Eigen::Vector3d(a * cos(angles::from_degrees(angle)),
				0.0f,
				2.5f + b * sin(angles::from_degrees(angle)));
	}

	/**
	 * @brief Square path motion routine
	 */
	void square_path_motion(ros::Rate loop_rate, control_mode mode){
		uint8_t pos_target = 1;

		ROS_INFO("Testing...");

		while (ros::ok()) {
			wait_and_move(ps);

			// motion routine
			switch (pos_target) {
			case 1:
				tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
				break;
			case 2:
				tf::pointEigenToMsg(pos_setpoint(-1, 1, 1), ps.pose.position);
				break;
			case 3:
				tf::pointEigenToMsg(pos_setpoint(-1, -1, 1), ps.pose.position);
				break;
			case 4:
				tf::pointEigenToMsg(pos_setpoint(1, -1, 1), ps.pose.position);
				break;
			case 5:
				tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
				break;
			default:
				break;
			}

			if (pos_target == 6) {
				ROS_INFO("Test complete!");
				ros::shutdown();
			}
			else
				++pos_target;

			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	/**
	 * @brief Circle path motion routine
	 */
	void circle_path_motion(ros::Rate loop_rate, control_mode mode){
		ROS_INFO("Testing...");
		ros::Time last_time = ros::Time::now();
		
		while (ros::ok()) {
			tf::pointMsgToEigen(localpos.pose.pose.position, current);

			
			// starting point
			/*
			if (mode == POSITION) {
				tf::pointEigenToMsg(Eigen::Vector3d(5.0f, 0.0f, 1.0f), ps.pose.position);
				local_pos_sp_pub.publish(ps);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(
								Eigen::Vector3d(5.0f, 0.0f, 0.0f), current, last_time), vs.linear);
				else
					tf::vectorEigenToMsg(Eigen::Vector3d(5.0f - current.x(), -current.y(), 0.0f - current.z()), vs.linear);	
				vel_sp_pub.publish(vs);

				ROS_INFO("Loop Pose: Pose [%f, %f, %f]", localpos.pose.pose.position.x, localpos.pose.pose.position.y, localpos.pose.pose.position.z);
				ROS_INFO("publish vel_sp_sub: linear [%f, %f, %f], angular [%f, %f, %f]", vs.linear.x, vs.linear.y, vs.linear.z, vs.angular.x, vs.angular.y, vs.angular.z);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}
			*/
			// -----------------------------------------------//
			double startPointX = 5.0f;
			double startPointY = 0.0f;

			double robot_current_roll, robot_current_pitch, robot_current_yaw;
			Eigen::Vector3d robot_current_position = current;
			tf::Quaternion q(localpos.pose.pose.orientation.x,localpos.pose.pose.orientation.y, localpos.pose.pose.orientation.z, localpos.pose.pose.orientation.w);
			tf::Matrix3x3 m(q);					
			m.getRPY(robot_current_roll, robot_current_pitch, robot_current_yaw);

			ROS_INFO("Move to the starting point!");
			ROS_INFO("Starting Point is : [%f, %f]", startPointX, startPointY);
			ROS_INFO("Robot Current Position: [%f, %f, %f]", robot_current_position.x(), robot_current_position.y(), robot_current_yaw);
			double startTheta = atan((startPointY - robot_current_position.y())/(startPointX - robot_current_position.x()));
			ROS_INFO("Starting Theta is : [%f]", startTheta);

						
			bool flag_turning = false;
			double tmpSin;
			double tmpCos;
			double tmpX;
			double tmpY;

			double errStartTheta;

			bool flag_moving = false;
			robot_current_position = current;
			double distanceToStart = sqrt((startPointX - robot_current_position.x())*(startPointX - robot_current_position.x())+(startPointY - robot_current_position.y())*(startPointY - robot_current_position.y()));
			ROS_INFO("Distance is %f", distanceToStart);

			gazebo_msgs::GetModelState getmodelstate;
			ros::NodeHandle n;
			ros::ServiceClient modelClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

			while(ros::ok() && (!flag_turning || !flag_moving ))
			{

				getmodelstate.request.model_name = "mobile_base";
				if (modelClient.call(getmodelstate))
				{
					localpos.pose.pose.position.x = getmodelstate.response.pose.position.x;
					localpos.pose.pose.position.y = getmodelstate.response.pose.position.y;
					localpos.pose.pose.position.z = getmodelstate.response.pose.position.z;
					localpos.pose.pose.orientation.x = getmodelstate.response.pose.orientation.x;
					localpos.pose.pose.orientation.y = getmodelstate.response.pose.orientation.y;
					localpos.pose.pose.orientation.z = getmodelstate.response.pose.orientation.z;
					localpos.pose.pose.orientation.w = getmodelstate.response.pose.orientation.w;
				}

				tf::pointMsgToEigen(localpos.pose.pose.position, current);
				robot_current_position = current;
				ROS_INFO("---------------------");
				ROS_INFO("Starting Point is : [%f, %f]", startPointX, startPointY);
				ROS_INFO("Robot Current Position: [%f, %f]", robot_current_position.x(), robot_current_position.y());
				

				robot_current_position = current;


				tmpSin = sin((startPointY - robot_current_position.y())/(startPointX - robot_current_position.x()));
				tmpCos = cos((startPointY - robot_current_position.y())/(startPointX - robot_current_position.x()));
			
				tmpX = startPointX - robot_current_position.x();
				tmpY = startPointY - robot_current_position.y();

				startTheta = atan((startPointY - robot_current_position.y())/(startPointX - robot_current_position.x()));
				
				ROS_INFO("Before Start Theta: [%f]", startTheta);
				startTheta = fabs(startTheta);
				if (tmpX > 0)
				{
					if (tmpY < 0)
					{
						startTheta = -startTheta;
					}					
				}
				else
				{
					if (tmpY < 0)
					{
						startTheta = - 3.1415926 + startTheta;
					}
					else
						startTheta = 3.1415926 - startTheta;

				}
				
				/*
				if (startPointX < robot_current_position.x())
				{
						startTheta = - 3.1415926 + abs(startTheta);
						if (startPointY > robot_current_position.y())
						{
							startTheta = - (3.1415926 + abs(atan((startPointY - robot_current_position.y())/(startPointX - robot_current_position.x()))));
						}
				}*/
				tf::Quaternion q(localpos.pose.pose.orientation.x,localpos.pose.pose.orientation.y, localpos.pose.pose.orientation.z, localpos.pose.pose.orientation.w);
				tf::Matrix3x3 m(q);					
				m.getRPY(robot_current_roll, robot_current_pitch, robot_current_yaw);
				
				errStartTheta = startTheta - robot_current_yaw;

				ROS_INFO("Start Theta, Robot Theta,: [%f, %f]", startTheta, robot_current_yaw);
				ROS_INFO("Theta Error is : [%f]", errStartTheta);

				if (fabs(errStartTheta) < 0.2)
				{
					flag_turning = true;
					vs.angular.z = 0.0;
				}
				else
				{
					flag_turning = false;
					if ( errStartTheta > 0)
					{
						vs.angular.z = 0.1;
						if (errStartTheta > 3.1415926)
							vs.angular.z = -0.1;
					}
					else
					{
						vs.angular.z = -0.1;
						if (errStartTheta < -3.1415926)
							vs.angular.z = 0.1;	
					}
				}
				
				
				distanceToStart = sqrt((startPointX - robot_current_position.x())*(startPointX - robot_current_position.x())+(startPointY - robot_current_position.y())*(startPointY - robot_current_position.y()));
				ROS_INFO("Distance Error is : [%f]", distanceToStart);
				if (fabs(distanceToStart) < 0.2)
				{
					flag_moving = true;
					flag_turning = true;
					vs.linear.x = 0;
				}
				else
				{
					flag_moving = false;
					flag_turning = false;
					if (fabs(errStartTheta) < 0.2 )
						vs.linear.x = 0.4;
				}
				
				//vs.linear.x = 0;
				vs.linear.y = 0;
				vs.linear.z = 0;
				vel_sp_pub.publish(vs);
				ROS_INFO("publish vel_sp_sub: linear [%f, %f, %f], angular [%f, %f, %f]", vs.linear.x, vs.linear.y, vs.linear.z, vs.angular.x, vs.angular.y, vs.angular.z);
				loop_rate.sleep();
				ros::spinOnce();
			}
			
				

			/*while(ros::ok() && !flag_moving)
			{
				getmodelstate.request.model_name = "mobile_base";
				if (modelClient.call(getmodelstate))
				{
					localpos.pose.pose.position.x = getmodelstate.response.pose.position.x;
					localpos.pose.pose.position.y = getmodelstate.response.pose.position.y;
					localpos.pose.pose.position.z = getmodelstate.response.pose.position.z;
					localpos.pose.pose.orientation.x = getmodelstate.response.pose.orientation.x;
					localpos.pose.pose.orientation.y = getmodelstate.response.pose.orientation.y;
					localpos.pose.pose.orientation.z = getmodelstate.response.pose.orientation.z;
					localpos.pose.pose.orientation.w = getmodelstate.response.pose.orientation.w;
				}				
				tf::pointMsgToEigen(localpos.pose.pose.position, current);
				robot_current_position = current;
				distanceToStart = sqrt((startPointX - robot_current_position.x())*(startPointX - robot_current_position.x())+(startPointY - robot_current_position.y())*(startPointY - robot_current_position.y()));
				ROS_INFO("Robot Current Pose: [%f, %f]", robot_current_position.x(), robot_current_position.y());
				ROS_INFO("Distance Error is : [%f]", distanceToStart);
				
				
				vs.linear.x = 1;
				vs.linear.y = 0;
				vs.linear.z = 0;
				vs.angular.z = 0;
				vel_sp_pub.publish(vs);
				if (fabs(distanceToStart) < 0.1)
				{
					flag_moving = true;
				}
				loop_rate.sleep();
				ros::spinOnce();
			}
			ROS_INFO("Turn Husky towards next point!");
			*/

			//wait_and_move(ps);
			ROS_INFO("Adjust Direction.");

			double delta_theta = 0.1;
			double theta =0;
			flag_turning = false;

			while(ros::ok() && !flag_turning)
			{
				getmodelstate.request.model_name = "mobile_base";
				if (modelClient.call(getmodelstate))
				{
					localpos.pose.pose.position.x = getmodelstate.response.pose.position.x;
					localpos.pose.pose.position.y = getmodelstate.response.pose.position.y;
					localpos.pose.pose.position.z = getmodelstate.response.pose.position.z;
					localpos.pose.pose.orientation.x = getmodelstate.response.pose.orientation.x;
					localpos.pose.pose.orientation.y = getmodelstate.response.pose.orientation.y;
					localpos.pose.pose.orientation.z = getmodelstate.response.pose.orientation.z;
					localpos.pose.pose.orientation.w = getmodelstate.response.pose.orientation.w;
				}				
				tf::pointMsgToEigen(localpos.pose.pose.position, current);
				tf::Quaternion q(localpos.pose.pose.orientation.x,localpos.pose.pose.orientation.y, localpos.pose.pose.orientation.z, localpos.pose.pose.orientation.w);
				tf::Matrix3x3 m(q);
				m.getRPY(robot_current_roll, robot_current_pitch, robot_current_yaw);

				Eigen::Vector3d ref_current_position(circle_shape(theta+delta_theta));
				Eigen::Vector3d ref_next_position(circle_shape(theta + delta_theta+delta_theta));
				double ref_theta = atan((ref_next_position.y() - ref_current_position.y())/(ref_next_position.x() - ref_current_position.x()));
				tmpX = ref_next_position.x() - ref_current_position.x();
				tmpY = ref_next_position.y() - ref_current_position.y();
				ref_theta = fabs(ref_theta);
				if (tmpX > 0)
				{
					if (tmpY < 0)
					{
						ref_theta = - ref_theta;
					}					
				}
				else
				{
					if (tmpY < 0)
					{
						ref_theta = -3.1415926 + ref_theta;
					}
					else
						ref_theta = 3.1415926 - ref_theta;

				}
				double errRobotTheta = ref_theta - robot_current_yaw;  // theta error

				ROS_INFO("Ref Theta, Robot Theta: [%f, %f]", ref_theta, robot_current_yaw);

				double theta_err = errRobotTheta;
				if (fabs(theta_err) < 0.1)
				{
					vs.angular.z = 0.0;
					flag_turning = true;
				}
				else
				{
					if ( theta_err > 0)
					{
						vs.angular.z = 0.2;
						if (theta_err > 3.1415926)
							vs.angular.z = -0.2;
					}
					else
					{
						vs.angular.z = -0.2;
						if (theta_err < -3.1415926)
							vs.angular.z = 0.2;	
					}
					
				}
				vs.linear.x = 0;
				vs.linear.y = 0;
				vs.linear.z = 0;
				vel_sp_pub.publish(vs);


				loop_rate.sleep();
				ros::spinOnce();
			}
			ROS_INFO("Arrived and move along the circle.");

			// motion routine
			
			//for (int theta = 0; theta <= 360; theta = theta + delta_theta) {
			for (int count = 0; count <= 360/delta_theta; count++) {
				theta += delta_theta;

				getmodelstate.request.model_name = "mobile_base";
				if (modelClient.call(getmodelstate))
				{
					localpos.pose.pose.position.x = getmodelstate.response.pose.position.x;
					localpos.pose.pose.position.y = getmodelstate.response.pose.position.y;
					localpos.pose.pose.position.z = getmodelstate.response.pose.position.z;
					localpos.pose.pose.orientation.x = getmodelstate.response.pose.orientation.x;
					localpos.pose.pose.orientation.y = getmodelstate.response.pose.orientation.y;
					localpos.pose.pose.orientation.z = getmodelstate.response.pose.orientation.z;
					localpos.pose.pose.orientation.w = getmodelstate.response.pose.orientation.w;
				}				
				tf::pointMsgToEigen(localpos.pose.pose.position, current);
				robot_current_position = current;
				ROS_INFO("--------------------------");
				ROS_INFO("Current Theta = %f", theta);
								
				
				

				if (mode == POSITION) {
					tf::pointEigenToMsg(circle_shape(theta), ps.pose.position);
					local_pos_sp_pub.publish(ps);
				}
				else if (mode == VELOCITY) {
					if (use_pid)
						tf::vectorEigenToMsg(pid.compute_linvel_effort(circle_shape(theta), current, last_time), vs.linear);
					else
						tf::vectorEigenToMsg(circle_shape(theta) - current, vs.linear);

					// ----------------- //
					Eigen::Vector3d ref_current_position(circle_shape(theta));
					Eigen::Vector3d ref_next_position(circle_shape(theta + delta_theta));

					double ref_theta = atan((ref_next_position.y() - ref_current_position.y())/(ref_next_position.x() - ref_current_position.x()));

					tmpSin = sin((ref_next_position.y() - ref_current_position.y())/(ref_next_position.x() - ref_current_position.x()));
					tmpCos = cos((ref_next_position.y() - ref_current_position.y())/(ref_next_position.x() - ref_current_position.x()));

					tmpX = ref_next_position.x() - ref_current_position.x();
					tmpY = ref_next_position.y() - ref_current_position.y();
					ref_theta = fabs(ref_theta);
					if (tmpX > 0)
					{
						if (tmpY < 0)
						{
							ref_theta = - ref_theta;
						}					
					}
					else
					{
						if (tmpY < 0)
						{
							ref_theta = -3.1415926 + ref_theta;
						}
						else
							ref_theta = 3.1415926 - ref_theta;

					}


					/*
					if (ref_next_position.x() < ref_current_position.x())
					{
						ref_theta = - 3.1415926 + abs(ref_theta);
						if (ref_next_position.y() > ref_current_position.y())
						{
							ref_theta = - (3.1415926 + abs(atan((ref_next_position.y() - ref_current_position.y())/(ref_next_position.x() - ref_current_position.x()))));
						}
					}*/
					
					//----- Get Robot Current Orientation ----- //	
					tf::Quaternion q(localpos.pose.pose.orientation.x,localpos.pose.pose.orientation.y, localpos.pose.pose.orientation.z, localpos.pose.pose.orientation.w);
					tf::Matrix3x3 m(q);
					m.getRPY(robot_current_roll, robot_current_pitch, robot_current_yaw);
					ROS_INFO("Robot Current Pose [X = %f, Y = %f, Yaw = %f]", robot_current_position.x(),robot_current_position.y(), robot_current_yaw);
					// ----- Calculate Error Between Reference Pose and Robot Current Pose ----//

					Eigen::Vector3d errRobotPosition = ref_current_position - robot_current_position;  // X, Y error
					ROS_INFO("Reference Target Pose [X = %f, Y = %f, Yaw = %f]", circle_shape(theta).x(), circle_shape(theta).y(), ref_theta);

					double errRobotTheta = ref_theta - robot_current_yaw;  // theta error
					
					// --- Transfer to Body Frame ----- //

					double distance_err = sqrt(errRobotPosition.x()*errRobotPosition.x() + errRobotPosition.y()*errRobotPosition.y());
					double theta_err = errRobotTheta;
					ROS_INFO("Error [distance = %f, theta = %f]", distance_err, theta_err);

					//theta_err = atan(errRobotPosition.y() / errRobotPosition.x());

					double P_distance = 0.5;
					double P_theta = 0.8;

					vs.linear.x = P_distance * distance_err;
					vs.linear.y = 0;
					vs.linear.z = 0;

					if (fabs(theta_err) < 0.05)
					{
						vs.angular.z = 0.0;
					}
					else
					{
						if ( theta_err > 0)
						{
							vs.angular.z = P_theta * theta_err;
							if (theta_err > 3.1415926)
								vs.angular.z = -P_theta * theta_err;
						}
						else
						{
							vs.angular.z = P_theta * theta_err;
							if (theta_err < -3.1415926)
								vs.angular.z = -P_theta * theta_err;	
						}
						
					}
									



					//-------------------//	

					vel_sp_pub.publish(vs);					
					ROS_INFO("publish vel_sp_sub: linear [%f, %f, %f], angular [%f, %f, %f]", vs.linear.x, vs.linear.y, vs.linear.z, vs.angular.x, vs.angular.y, vs.angular.z);
				}
				else if (mode == ACCELERATION) {
					// TODO
					return;
				}
				if (theta == 360.0) {
					ROS_INFO("Test complete!");
					ros::shutdown();
				}
				last_time = ros::Time::now();
				loop_rate.sleep();
				ros::spinOnce();
			}
		}
	}

	/**
	 * @brief Eight path motion routine
	 */
	void eight_path_motion(ros::Rate loop_rate, control_mode mode){
		ROS_INFO("Testing...");
		ros::Time last_time = ros::Time::now();

		while (ros::ok()) {
			tf::pointMsgToEigen(localpos.pose.pose.position, current);

			// starting point
			if (mode == POSITION) {
				tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 1.0f), ps.pose.position);
				local_pos_sp_pub.publish(ps);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(
								Eigen::Vector3d(0.0f, 0.0f, 1.0f), current, last_time), vs.linear);
				else
					tf::vectorEigenToMsg(Eigen::Vector3d(-current.x(), -current.y(), 1.0f - current.z()), vs.linear);
				vel_sp_pub.publish(vs);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}

			wait_and_move(ps);

			// motion routine
			for (int theta = -180; theta <= 180; theta++) {
				tf::pointMsgToEigen(localpos.pose.pose.position, current);

				if (mode == POSITION) {
					tf::pointEigenToMsg(eight_shape(theta), ps.pose.position);
					local_pos_sp_pub.publish(ps);
				}
				else if (mode == VELOCITY) {
					if (use_pid)
						tf::vectorEigenToMsg(pid.compute_linvel_effort(eight_shape(theta), current, last_time), vs.linear);
					else
						tf::vectorEigenToMsg(eight_shape(theta) - current, vs.linear);
					vel_sp_pub.publish(vs);
				}
				else if (mode == ACCELERATION) {
					// TODO
					return;
				}
				if (theta == 180) {
					ROS_INFO("Test complete!");
					ros::shutdown();
				}
				last_time = ros::Time::now();
				loop_rate.sleep();
				ros::spinOnce();
			}
		}
	}

	/**
	 * @brief Ellipse path motion routine
	 */
	void ellipse_path_motion(ros::Rate loop_rate, control_mode mode){
		ROS_INFO("Testing...");
		ros::Time last_time = ros::Time::now();

		while (ros::ok()) {
			tf::pointMsgToEigen(localpos.pose.pose.position, current);

			// starting point
			if (mode == POSITION) {
				tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 2.5f), ps.pose.position);
				local_pos_sp_pub.publish(ps);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(
								Eigen::Vector3d(0.0f, 0.0f, 2.5f), current, last_time), vs.linear);
				else
					tf::vectorEigenToMsg(Eigen::Vector3d(-current.x(), -current.y(), 2.5f - current.z()), vs.linear);
				vel_sp_pub.publish(vs);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}

			wait_and_move(ps);

			// motion routine
			for (int theta = 0; theta <= 360; theta++) {
				tf::pointMsgToEigen(localpos.pose.pose.position, current);

				if (mode == POSITION) {
					tf::pointEigenToMsg(ellipse_shape(theta), ps.pose.position);
					local_pos_sp_pub.publish(ps);
				}
				else if (mode == VELOCITY) {
					if (use_pid)
						tf::vectorEigenToMsg(pid.compute_linvel_effort(ellipse_shape(theta), current, last_time), vs.linear);
					else
						tf::vectorEigenToMsg(ellipse_shape(theta) - current, vs.linear);
					vel_sp_pub.publish(vs);
				}
				else if (mode == ACCELERATION) {
					// TODO
					return;
				}
				if (theta == 360) {
					ROS_INFO("Test complete!");
					ros::shutdown();
				}
				last_time = ros::Time::now();
				loop_rate.sleep();
				ros::spinOnce();
			}
		}
	}

	/**
	 * @brief Defines the accepted threshold to the destination/target position
	 * before moving to the next setpoint.
	 */
	void wait_and_move(geometry_msgs::PoseStamped target){
		ros::Rate loop_rate(rate);
		ros::Time last_time = ros::Time::now();
		bool stop = false;

		Eigen::Vector3d dest;

		double distance;
		double err_th = threshold[rand() % threshold.size()];

		ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", err_th);

		while (ros::ok() && !stop) {
			tf::pointMsgToEigen(target.pose.position, dest);
			tf::pointMsgToEigen(localpos.pose.pose.position, current);

			distance = sqrt((dest - current).x() * (dest - current).x() +
					(dest - current).y() * (dest - current).y() +
					(dest - current).z() * (dest - current).z());

			if (distance <= err_th)
				stop = true;

			if (mode == POSITION) {
				local_pos_sp_pub.publish(target);
			}
			else if (mode == VELOCITY) {
				if (use_pid)
					tf::vectorEigenToMsg(pid.compute_linvel_effort(dest, current, last_time), vs.linear);
				else
					tf::vectorEigenToMsg(dest - current, vs.linear);
				vel_sp_pub.publish(vs);
			}
			else if (mode == ACCELERATION) {
				// TODO
				return;
			}
			last_time = ros::Time::now();
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	/**
	 * @brief Gaussian noise generator for accepted position threshold
	 */
	std::array<double, 100> threshold_definition(){
		std::random_device rd;
		std::mt19937 gen(rd());
		std::array<double, 100> th_values;

		std::normal_distribution<double> th(0.1f,0.05f);

		for (auto &value : th_values) {
			value = th(gen);
		}
		return th_values;
	}

	/* -*- callbacks -*- */

	//void local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg){
	//void local_pos_cb(const OdomConstPtr& msg){
	void local_pos_cb(const gazebo_msgs::ModelStateConstPtr &msg){
		//localpos = *msg;
		//localModelState = *msg;
		//ROS_INFO("Callback!");
		/*if (msg.model_name == "mobile_base")
		{
			localpos.pose.pose.position.x = msg.pose.position.x;
			localpos.pose.pose.position.y = msg.pose.position.y;
			localpos.pose.pose.position.z = msg.pose.position.z;
			localpos.pose.pose.orientation.x = msg.pose.orientation.x;
			localpos.pose.pose.orientation.y = msg.pose.orientation.y;
			localpos.pose.pose.orientation.z = msg.pose.orientation.z;
			ROS_INFO("Callback Current Pose: Pose [%f, %f, %f]", localpos.pose.pose.position.x, localpos.pose.pose.position.y, localpos.pose.pose.position.z);
			ROS_INFO("Callback Current Orientation: Orientation [%f, %f, %f, %f]", localpos.pose.pose.orientation.x, localpos.pose.pose.orientation.y, localpos.pose.pose.orientation.z, localpos.pose.pose.orientation.w);
		}*/
		
	}
};
};	// namespace testsetup

