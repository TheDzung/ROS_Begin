#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "math.h"
#include "geometry_msgs/Pose2D.h"

void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg);
void DesPoseCallback(const turtlesim::Pose::ConstPtr& msg);
float GetErrorLin(turtlesim::Pose curpose, turtlesim::Pose despose);
float GetErrorAng(turtlesim::Pose curpose, turtlesim::Pose despose);

geometry_msgs::Twist Vel;
turtlesim::Pose CurPose;
turtlesim::Pose DesPose;
float ErrorLin=0;
float ErrorAng=0;

bool STOP= true;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveturtle");	
	ros::NodeHandle n;

	ros::Publisher Twist_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
	ros::Subscriber CurPose_sub = n.subscribe("/turtle1/pose", 5, CurPoseCallback);	
	ros::Subscriber DesPose_sub = n.subscribe("/turtle/DesPosition",5,DesPoseCallback);	
	ros::Rate loop_rate(10);
	
	while (ros::ok() && n.ok() )
	{	
		ros::spinOnce();
		if(STOP==false)
		{
			ErrorLin = GetErrorLin(CurPose,DesPose);
			ErrorAng = GetErrorAng(CurPose,DesPose);
			if(ErrorLin > 0)
			{
				Vel.linear.x = 0.2*ErrorLin;
			}
			else
			{
				Vel.linear.x = 0;
			}
			Vel.angular.z = 0.5*ErrorAng;
			Twist_pub.publish(Vel);
		}

		loop_rate.sleep();
	}
}

void CurPoseCallback(const turtlesim::Pose::ConstPtr& msg)
{	
	
	
		CurPose.x = msg->x;
		CurPose.y = msg->y;
		CurPose.theta = msg->theta;
		ROS_INFO("Pos x=%f : Pos y =%f : Error linear =%f : Error angular =%f",CurPose.x,CurPose.y,ErrorLin,ErrorAng);
	
}

void DesPoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	STOP=false;
	DesPose.x = msg->x;
	DesPose.y = msg->y;

}

float GetErrorAng(turtlesim::Pose curpose, turtlesim::Pose despose)
{
	float Ex = despose.x - curpose.x;
	float Ey = despose.y - curpose.y;
	float dest = atan2f(Ey,Ex);
	
	float Et = dest - curpose.theta;

	return Et;  
}

float GetErrorLin(turtlesim::Pose curpose, turtlesim::Pose despose)
{
	float Ex= despose.x - curpose.x;
	float Ey= despose.y - curpose.y;
	float Et= GetErrorAng(curpose,despose);

	float Etx = hypotf(Ex,Ey)*cos(Et);

	return  Etx;
}
