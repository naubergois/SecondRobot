#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>


bool pickup=false;
bool finish=false;
bool finishDrop=false;
float x_goal=-5.71;
float y_goal=-1.53;
float x_second_goal=1.75;
float y_second_goal=4.42;

float error=0.8;
bool drop=false;


void callback_method(const nav_msgs::Odometry::ConstPtr& msg)

{
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float diffx=std::abs(x_goal -x);
	float diffy=std::abs(y_goal -y);

	ROS_INFO("callback");
	ROS_INFO("x %f",diffx);
	ROS_INFO("y %f",diffy);
	if ((diffx < error) && (diffy <error))
	   {
	        ROS_INFO("pickup"); 

	     	pickup=true;
	   }else{
		ROS_INFO("no pickup");
		pickup = false;
	   }


	if ((std::abs(x_second_goal -x) < error) && (std::abs(y_second_goal -y) <error)  )
	   { 
		if(pickup)
	  	 {
	     		drop=true;
		}	
	   }else{
		drop = false;
	   }

}


int main( int argc, char** argv )
{
  ROS_INFO("init add marker");
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ROS_INFO("set callback");

  ros::Subscriber subs = n.subscribe("odom", 1000,  callback_method);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

 
    ROS_INFO("ROS ok");
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    ROS_INFO("init add marker");

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_goal;
    marker.pose.position.y = y_goal;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);

    
    ROS_INFO("enter ifs");

    while (ros::ok())
    {
           ROS_INFO("ros while ok");   
	   if(pickup)
	   {
		if(!finish){
	     		marker.action = visualization_msgs::Marker::DELETE;
	     		marker_pub.publish(marker);
	     		ROS_INFO("Remove pickup");
	     		finish = true;
			pickup=true;
		}
	    }


	    ROS_INFO("if drop"); 
	    if(drop){
		  if(!finishDrop){              
		 	marker.pose.position.x = x_second_goal;
		 	marker.pose.position.y = y_second_goal;
		 	marker.action = visualization_msgs::Marker::ADD;
			marker_pub.publish(marker);
			finishDrop=true;
		        x_goal=x_second_goal;
		        y_goal=y_second_goal;
		  }
			
			
	    }  

	    ROS_INFO("test subscribers"); 

	    // Publish the marker
	    while (marker_pub.getNumSubscribers() < 1)
	    {
	      if (!ros::ok())
	      {
		return 0;
	      }
	      ROS_WARN_ONCE("Please create a subscriber to the marker");
	      sleep(1);
	    }
	    
    	    marker_pub.publish(marker);
		
            ros::spinOnce();
    	    ROS_INFO("Marker displayed");

    
   }
   return 0;
}
