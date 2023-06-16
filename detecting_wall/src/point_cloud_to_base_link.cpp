#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>

using namespace std;

class CublicSpline1D //a=dt^3+ct^2+b*t+a
{
private:
    size_t dimension;
    vector<float> a_, b_, c_, d_;
    vector<float> x_,y_;

    Eigen::MatrixXf calA(vector<float> h)
    {
        Eigen::MatrixXf A=Eigen::MatrixXf::Zero(dimension, dimension);
        A[0, 0] = 1.0;
        for(size_t i=0; i<dimension-1; i++)
        {
            if(i != dimension -2)
            {
                A(i+1, i+1) = 2*(h[i]+h[i+1]);
            }
            A(i+1, i) = h[i];
            A(i, i+1) = h[i];
        }
        A(0, 1) = 0.0;
        A(dimension -1, dimension -2) = 0.0;
        A(dimension -1, dimension -1) = 1.0;
        return A;
    }

    Eigen::VectorXf calB(vector<float> h)
    {
        Eigen::VectorXf B=Eigen::VectorXf::Zero(dimension);
        for(size_t i=0; i<dimension -2; i++)
        {
            B[i+1] = 3*(a_[i+2]- a_[i+1])/h[i+1] - 3*(a_[i+1]- a_[i])/h[i]; 
        }

        return B;
    }

    int searchIndex(float x)
    {
        int i=0;
        while(i < dimension -2)
        {
            if(x_[i] < x && x <= x_[i+1])
            {
                break;
            }
            i++;
        }
        return i;
    }

public:
    CublicSpline1D() = default;
    void getSXY(vector<float> x, vector<float> y)
    {
        x_.clear();
        y_.clear();
        a_.clear();
        b_.clear();
        c_.clear();
        d_.clear();

        vector<float> h;
        for(int i=0; i<x.size()-1; i++)
        {
            h.push_back(x[i+1]-x[i]);
        }

        
        dimension = x.size();
        x_ = x;
        y_ = y;

        for(auto &i : y)
        {
            a_.push_back(i);
        }

        Eigen::MatrixXf A = this->calA(h);
        Eigen::VectorXf B = this->calB(h);

        Eigen::VectorXf C = A.inverse() * B;
        for(int i=0; i<C.rows(); i++)
        {
            c_.push_back(C[i]);
        }

        for(size_t i=0; i<dimension-1; i++)
        {
            float d_temp = (c_[i+1] - c_[i]) / (3*h[i]);
            float b_temp = 1/h[i] *(a_[i+1]-a_[i]) - h[i]/3* (2*c_[i]+c_[i+1]);
            d_.push_back(d_temp);
            b_.push_back(b_temp);
        }
    }

    float calcPosition(float x)
    {
        int i = this->searchIndex(x);
        float dx = x - x_[i];
        float position = a_[i] + b_[i] * dx + c_[i]*dx*dx + d_[i] * dx * dx * dx;
        return position;
    }

    float clacFirstDerivative(float x)
    {
        int i = this->searchIndex(x);
        float dx = x - x_[i];
        float dy = b_[i] + 2 * c_[i] * dx + 3 * d_[i] * dx * dx;
        return dy;
    }

    float calcSecondDerivative(float x)
    {
        int i = this->searchIndex(x);
        float dx = x - x_[i];
        float ddy = 2 * c_[i] + 6*d_[i]*dx;
        return ddy;
    }

};



class PoseDrawer
{
public:
    PoseDrawer()
    {
        pc_sub_=nh_.subscribe<sensor_msgs::PointCloud>("/local/waypoints", 10, &PoseDrawer::transformPointCloudCB, this);
    }

    geometry_msgs::Pose2D calcPosition(float s)
    {
        geometry_msgs::Pose2D pos;
        pos.x = sx_.calcPosition(s);
        pos.y = sy_.calcPosition(s);

        float dx = sx_.clacFirstDerivative(s);
        float dy = sy_.clacFirstDerivative(s);

        pos.theta = atan2(dy, dx);

        return pos;
    }

    float calCurvature(float s)
    {
        float dx, ddx, dy, ddy, k;
        dx=sx_.clacFirstDerivative(s);
        ddx = sx_.calcSecondDerivative(s);
        dy=sy_.clacFirstDerivative(s);
        ddy=sy_.calcSecondDerivative(s);
        k=(ddy*dx-ddx*dy)/pow(pow(dx,2)+pow(dy,2),1.5);
        return k;
    }
    

private:
    ros::Subscriber pc_sub_;
    std::string REFERENCE_FRAME = "world";
    ros::NodeHandle nh_;

    CublicSpline1D sx_;
    CublicSpline1D sy_;
    
    vector<float> ds_;

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
        
        vector<float> pc_x, pc_y;
        vector<float> s_;

        for(auto &i : cloud_out.points)
        {
            pc_x.push_back(i.x);
            pc_y.push_back(i.y);
        }

        s_=this->calS(pc_x, pc_y);
        sx_.getSXY(s_, pc_x);
        sy_.getSXY(s_, pc_y);

    }

    vector<float> calS(vector<float> xp, vector<float>yp)
    {
        vector<float> dx, dy;
        ds_.clear();

        vector<float> s_;
        s_.push_back(0);
        
        float diff_x, diff_y;
        float abs_diff;
        
        for(int i=0; i<xp.size()-1; i++)
        {
            diff_x=xp[i+1]-xp[i];
            diff_y=yp[i+1]-yp[i];
            dx.push_back(diff_x);
            dy.push_back(diff_y);
            abs_diff = sqrt(pow(diff_x,2) + pow(diff_y,2));
            ds_.push_back(abs_diff);
            s_.push_back(s_[i]+abs_diff);
        }

        return s_;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_to_world_link");

    PoseDrawer pd;
    ros::spin();

    
    return 0;
}