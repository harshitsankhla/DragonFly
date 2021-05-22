
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

// Callback function for copter state
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;

}

// Callback function for copter location
geometry_msgs::PoseStamped local_position;
void location_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	local_position = *msg;
}


// Parameters
int offset = 2;
int goal_counter = 0;
float positions[][3] = {{0, 0, 5},{0, 10, 5}, {10, 10, 5}, {20, 10, 5}};
int waypoint_count = sizeof(positions)/ sizeof(positions[0]);

geometry_msgs::PoseStamped pose;


// Function to check is quadcopter is within waypoint offset
bool is_at_position( float x, float y, float z, float offset)
{
	float dist_to_pos = sqrt(
	pow( local_position.pose.position.x - x, 2) +
	pow( local_position.pose.position.y - y, 2) +
	pow( local_position.pose.position.z - z, 2));
	if (dist_to_pos < offset && (goal_counter < waypoint_count))
	{
		cout<<"Waypoint reached"<<endl;
	}
	return dist_to_pos < offset;
}



// Fucntion to set waypoint
geometry_msgs::PoseStamped reach_position()
{
	float des_x = positions[goal_counter][0];
	float des_y = positions[goal_counter][1];
	float des_z = positions[goal_counter][2];
	// cout<< "dest: " << des_x << " " << des_y << " " << des_z << endl;

	if (is_at_position(des_x, des_y, des_z, offset) && (goal_counter < waypoint_count -1)) 
	{
		goal_counter = goal_counter + 1;
		// cout<< "waypoint_count"<<waypoint_count<<endl;
		// cout<< "goal_counter"<<goal_counter<<endl;
		cout<<"going to point: "<< positions[goal_counter][0]<<positions[goal_counter][1]<<positions[goal_counter][2]<<endl;
		pose.pose.position.x = positions[goal_counter][0];
		pose.pose.position.y = positions[goal_counter][1];
		pose.pose.position.z = positions[goal_counter][2];
	}
	// cout<< "pose sent: " << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z << endl;
	return pose;
}



int main(int argc, char **argv)
{
	// Initalization
	pose.pose.position.x = positions[0][0];
	pose.pose.position.y = positions[0][1];
	pose.pose.position.z = positions[0][2];
	cout<<"Inital point: "<< positions[0][0]<<positions[0][1]<<positions[0][2]<<endl;
	ros::init( argc, argv, "waypoint_node");
	ros::NodeHandle nh;


	// If using position setpoints in offboard mode, then velocity of quad is equal to the maximum set in MPC_XY_CRUISE parameter (available in QGC). The FCU applies this maximum velocity to the quad until setpoint is achieved.

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber localtion_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 5, location_cb );
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	// the setpoint publishing rate must be faster that 2Hz
	ros::Rate rate(20.0);

	// wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("FCU connected");

	


	// send a few setpoints before starting
	for( int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;


	ros::Time last_request = ros::Time::now();
	// cout<<" Starting main loop####################################################"<<endl;
	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
			{
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if( !current_state.armed && (ros::Time::now() -last_request > ros::Duration(5.0)))
			{
				if( arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}


		local_pos_pub.publish(reach_position());

		ros::spinOnce();
		rate.sleep();

	}
	return 0;


}