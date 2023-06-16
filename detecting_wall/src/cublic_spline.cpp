#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include <cmath>
#include <vector>

#include <Eigen/Dense>

using namespace std;

class CublicSpline1D
{
private:

public:
    CublicSpline1D() = default;
    void getSXY(vector<float> s, vector<float> xy)
    {

    }

};

class CublicSpline2D
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber points_sub_;

    CublicSpline1D sx_;
    CublicSpline1D sy_;
    
    vector<float> ds_;

    void pointcloudCallback(const sensor_msgs::PointCloud::ConstPtr & pc)
    {
        vector<float> pc_x, pc_y;
        vector<float> s_;
        for(auto &i : pc->points)
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

public:
    CublicSpline2D(void)
    {
        points_sub_= nh_.subscribe("/local/waypoints", 10, &CublicSpline2D::pointcloudCallback, this);

    }



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cublic_spline");

    CublicSpline2D c2;
    
    return 0;
}