#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include <limits>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

/*
transform_tolerance: 0.01
            min_height: -0.3
            max_height: 1.0

            angle_min: -3.14 # -M_PI
            angle_max: 3.14 # M_PI
            angle_increment: 0.0087 # M_PI/360.0
            scan_time: 0.3333
            range_min: 0.45
            range_max: 10.0
            use_inf: true
            inf_epsilon: 1.0

            # Concurrency level, affects number of pointclouds queued for processing and number of threads used
            # 0 : Detect number of cores
            # 1 : Single threaded
            # 2->inf : Parallelism level
            concurrency_level: 1
            #SIZE 722
*/

#define RIGHT_START 140
#define RIGHT_END 271
#define LEFT_START 451
#define LEFT_END 582
#define RADIAN_INC 0.0087

using namespace std;

class Waypoints
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber scan_sub_;
    Eigen::Vector3f left_line_;
    Eigen::Vector3f right_line_;

    sensor_msgs::PointCloud way_points_;
    ros::Publisher points_pub_;

    ros::Rate rate_;

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr & scan_msg)
    {
        float x_temp, y_temp;
        vector<float> right_x_vec;
        vector<float> right_y_vec;
        vector<float> left_x_vec;
        vector<float> left_y_vec;
    
        for(int i=0; i<scan_msg->ranges.size(); i++)
        {
            if(scan_msg->ranges[i]!=INFINITY)
            {
                x_temp=scan_msg->ranges[i]*cos(i*RADIAN_INC)*(-1);
                y_temp=scan_msg->ranges[i]*sin(i*RADIAN_INC)*(-1);
                if(140<=i && i<=271)
                {
                    right_x_vec.push_back(x_temp);
                    right_y_vec.push_back(y_temp);
                }
                else if(451<= i && i<=582)
                {
                    left_x_vec.push_back(x_temp);
                    left_y_vec.push_back(y_temp);
                }
                else
                {   
                }
            }
        }

        right_line_=linearFitting(right_x_vec, right_y_vec);
        left_line_=linearFitting(left_x_vec, left_y_vec);
       
    }

    Eigen::Vector3f linearFitting(vector<float> xvec, vector<float> yvec)
    {
        Eigen::VectorXf X_SQUARE;
        Eigen::VectorXf X;
        Eigen::VectorXf Y;
        Eigen::VectorXf SIGMA;
        Eigen::MatrixXf A;
        Eigen::Matrix3f temp;

        X_SQUARE=Eigen::VectorXf(xvec.size());
        X=Eigen::VectorXf(xvec.size());
        Y=Eigen::VectorXf(yvec.size());
        SIGMA=Eigen::VectorXf(xvec.size());

        //cout<<"1"<<endl;

        for(int ii=0; ii<xvec.size(); ii++)
        {
            X_SQUARE[ii]=xvec[ii]*xvec[ii];
            X[ii]=xvec[ii];
            Y[ii]=yvec[ii];
            SIGMA[ii]=1;
        }

        //cout<<"2"<<endl;

        A=Eigen::MatrixXf(xvec.size(),3);
        A.col(0)<<X_SQUARE;
        A.col(1)<<X;
        A.col(2)<<SIGMA;

        //cout<<"3"<<endl;

        temp=A.transpose()*A;

        return temp.inverse()*A.transpose()*Y;
    }

public:
    Waypoints()
    :  rate_(10)
    {
        this->scan_sub_=nh_.subscribe("/projected/scan", 10, &Waypoints::laserCallback, this);
        this->points_pub_=nh_.advertise<sensor_msgs::PointCloud>("/local/waypoints", 10);
        way_points_.header.frame_id="velodyne";
    }

    void publishWaypoints()
    {
        way_points_.header.stamp=ros::Time::now();
        geometry_msgs::Point32 wp_;
        for(float i=1.0; i<=10; i=i+0.1)
        {
            wp_.x=i;
            wp_.y=i*i*(left_line_[0]+right_line_[0])/2 + i*(left_line_[1]+right_line_[1])/2+(right_line_[2]+left_line_[2])/2;
            wp_.z=0;
            way_points_.points.push_back(wp_);
        }

        points_pub_.publish(way_points_);
        way_points_.points.clear();
        rate_.sleep();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "find_waypoint");

    Waypoints wp;
    while(ros::ok())
    {
        wp.publishWaypoints();
        ros::spinOnce();
    }
    
    return 0;
}