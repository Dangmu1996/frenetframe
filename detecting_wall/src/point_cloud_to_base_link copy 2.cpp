#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Point32.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std;


class PoseDrawer
{
public:
    PoseDrawer()
    {
        pc_sub_=nh_.subscribe<sensor_msgs::PointCloud>("/local/waypoints", 10, &PoseDrawer::transformPointCloudCB, this);
    }
    

private:
    ros::Subscriber pc_sub_;
    std::string REFERENCE_FRAME = "world";
    ros::NodeHandle nh_;

    bool getTransform(const std::string &refFrame, const std::string &childFrame, tf::StampedTransform &transform)
    {
        std::string errMsg;
		tf::TransformListener tf_listener;
        if (!tf_listener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.2), ros::Duration(0.01), &errMsg))
        {
			ROS_ERROR_STREAM("Pointcloud transform | Unable to get pose from TF: " << errMsg);
			return false;
		}
        else
        {
				try
                {
					tf_listener.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
				}
				catch (const tf::TransformException &e) 
                {
					ROS_ERROR_STREAM(
							"Pointcloud transform | Error in lookupTransform of " << childFrame << " in " << refFrame);
					return false;
				}
		}
		return true;

    }

    void transformPointCloudCB(const sensor_msgs::PointCloud::ConstPtr & pc)
    {
        tf::StampedTransform tf_between_frames;
        getTransform(REFERENCE_FRAME, pc->header.frame_id, tf_between_frames);
    	geometry_msgs::TransformStamped tf_between_frames_geo;
		tf::transformStampedTFToMsg(tf_between_frames, tf_between_frames_geo);
        sensor_msgs::PointCloud2 cloud_in2;
		sensor_msgs::convertPointCloudToPointCloud2(*pc, cloud_in2); // convert from pointcloud to pointcloud2
		sensor_msgs::PointCloud2 cloud_in_transformed;
		tf2::doTransform(cloud_in2, cloud_in_transformed, tf_between_frames_geo);
        sensor_msgs::PointCloud cloud_out;
		sensor_msgs::convertPointCloud2ToPointCloud(cloud_in_transformed, cloud_out);
        cout<<"1"<<endl;
        cout<<cloud_out.points[0].x<<endl;
    
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_to_world_link");

    PoseDrawer pd;
    ros::spin();

    
    return 0;
}