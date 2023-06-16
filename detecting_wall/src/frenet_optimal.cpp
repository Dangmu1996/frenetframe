#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "detecting_wall/frenet.h"

#include <utility>

using namespace std;

// cost weights
#define K_J 0.1;
#define K_T 0.1;
#define K_D 1.0;
#define K_LAT 1.0;
#define K_LON 1.0;
#define TARGET_SPEED 0.5;

struct FrenetPath
{
    vector<float> t;
    vector<float> d;
    vector<float> d_d;
    vector<float> d_dd;
    vector<float> d_ddd;
    vector<float> s;
    vector<float> s_d;
    vector<float> s_dd;
    vector<float> s_ddd;
    float cd;
    float cv;
    float cf;

    vector<float> x;
    vector<float> y;
    vector<float> yaw;
    vector<float> vyaw;
    vector<float> v;
    vector<float> ds;
    vector<float> c;
    bool defalt = false;
};

class QuinticPolynomial
{
public:
    QuinticPolynomial() = default;
    

    void getParam(float xs, float vxs, float axs, float xe, float vxe, float axe, float time)
    {
        a0=xs;
        a1=vxs;
        a2=axs/2;

        t << pow(time, 3), pow(time, 4), pow(time,5),
             pow(time,2)*3, pow(time, 3)*4, pow(time, 4)*5,
             6*time, 12*pow(time, 2), 20*pow(time, 3);
        left_hand << xe - a0 - a1*time - a2*pow(time, 2), vxe - a1 - 2*a2*time, axe-2*a2;

        a=t.inverse()*left_hand;
        a3=a[0];
        a4=a[1];
        a5=a[2];
    }

    float calPoint(float current_time)
    {
        float pos;
        pos = a0 + a1*current_time + a2*pow(current_time,2) + a3*pow(current_time,3) + a4*pow(current_time, 4) + a5*pow(current_time, 5);
        return pos;
    }

    float calFirstDerivative(float current_time)
    {
        float vel;
        vel = a1 + 2*a2*current_time + 3*a3*pow(current_time,2) + 4*a4*pow(current_time, 3) + 5*a5*pow(current_time, 4);
        return vel;
    }

    float calSecondDerivative(float current_time)
    {
        float accel;
        accel =  2*a2 + 6*a3*current_time + 12*a4*pow(current_time, 2) + 20*a5*pow(current_time, 3);
        return accel;
    }

    float calThirdDerivative(float current_time)
    {
        float jerk;
        jerk =  6*a3 + 24*a4*current_time + 60*a5*pow(current_time, 2);
        return jerk;
    }

private:
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;
    float a3 = 0;
    float a4 = 0;
    float a5 = 0;

    Eigen::Matrix3f t;
    Eigen::Vector3f left_hand;
    Eigen::Vector3f a;

};

class QuarticPolynomial
{
public:
    QuarticPolynomial() = default;
    void getParam(float xs, float vxs, float axs, float vxe, float axe, float time)
    {
        a0 = xs;
        a1 = vxs;
        a2 = axs / 2.0;

        t << pow(time,2)*3, pow(time, 3)*4,
             6*time, 12*pow(time, 2);
        left_hand << vxe - a1 - 2*a2*time, axe-2*a2;
        a=t.inverse()*left_hand;

        a3 = a[0];
        a4 = a[1];
    }

    float calPoint(float current_time)
    {
        float pos;
        pos = a0 + a1*current_time + a2*pow(current_time,2) + a3*pow(current_time,3) + a4*pow(current_time, 4);
        return pos;
    }

    float calFirstDerivative(float current_time)
    {
        float vel;
        vel = a1 + 2*a2*current_time + 3*a3*pow(current_time,2) + 4*a4*pow(current_time, 3);
        return vel;
    }

    float calSecondDerivative(float current_time)
    {
        float accel;
        accel =  2*a2 + 6*a3*current_time + 12*a4*pow(current_time, 2);
        return accel;
    }

    float calThirdDerivative(float current_time)
    {
        float jerk;
        jerk =  6*a3 + 24*a4*current_time;
        return jerk;
    }



private:
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;
    float a3 = 0;
    float a4 = 0;

    Eigen::Matrix2f t;
    Eigen::Vector2f left_hand;
    Eigen::Vector2f a;
};



class CublicSpline1D //a=dt^3+ct^2+b*t+a
{
private:
    size_t dimension;
    vector<float> a_, b_, c_, d_;
    vector<float> x_,y_;

    Eigen::MatrixXf calA(vector<float> h)
    {
        Eigen::MatrixXf A=Eigen::MatrixXf::Zero(dimension, dimension);
        A(0, 0) = 1.0;
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
        if(x==0)
        {
            return 0;
        }

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
        x_=x;
        y_=y;

        vector<float> h;
        for(int i=0; i<x.size()-1; i++)
        {
            h.push_back(x[i+1]-x[i]);
        }

        dimension = x.size();
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
    //float s_max;

    PoseDrawer()
    {
        pc_sub_=nh_.subscribe<sensor_msgs::PointCloud>("/local/waypoints", 1, &PoseDrawer::transformPointCloudCB, this);
        frenet_sub_=nh_.subscribe("/frenetDY", 1, &PoseDrawer::frenetCB,this);
        
        current_pos_sub_ = nh_.subscribe("/pose2D", 1 , &PoseDrawer::currentPoseCB, this);
        way_pub_=nh_.advertise<sensor_msgs::PointCloud>("/frenet/waypoints", 1);
        obstacle_sub_=nh_.subscribe<sensor_msgs::PointCloud>("/front_obstacle", 1, &PoseDrawer::transformObstacleCB, this);

        cmd_pub_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
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

    void publishWaypoints(vector<float> x, vector<float> y)
    {
        // cout<<"4_1"<<endl;
        // cout<<x[0]<<y[0]<<endl;
        
        sensor_msgs::PointCloud waypoints;
        geometry_msgs::Point32 points;
        waypoints.header.frame_id="world";
        waypoints.header.stamp=ros::Time::now();
        for(int i=0; i<x.size(); i++)
        {
            points.x=x[i];
            points.y=y[i];
            points.z=0;
            waypoints.points.push_back(points);
        }
        way_pub_.publish(waypoints);
        
    }

    void publishCmd(float v, float w)
    {
        geometry_msgs::Twist vel;
        vel.linear.x = v;
        vel.angular.z = w;

        cmd_pub_.publish(vel);

    }

    geometry_msgs::Pose2D getCurrentPose()
    {
        return current_pose_;
    }

    vector<geometry_msgs::Point32> getObstacle()
    {
        return obstacle_list_;
    }

    vector<float> getInitalFrenet()
    {
        vector<float> answer;
        answer.push_back(this->d0);
        answer.push_back(this->iyaw);
        return answer;
    }
    

private:
    ros::Subscriber pc_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber current_pos_sub_;
    ros::Subscriber frenet_sub_;

    ros::Publisher way_pub_;
    ros::Publisher cmd_pub_;
    CublicSpline1D sx_;
    CublicSpline1D sy_;
    ros::NodeHandle nh_;
    std::string REFERENCE_FRAME = "world";
    
    geometry_msgs::Pose2D current_pose_;
    vector<geometry_msgs::Point32> obstacle_list_;
    float iyaw, d0;
    
    vector<float> ds_;

    bool getTransform(const std::string &refFrame, const std::string &childFrame, tf::StampedTransform &transform)
    {
        std::string errMsg;
		tf::TransformListener tf_listener;
        if (!tf_listener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.3), ros::Duration(0.01), &errMsg))
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

    void transformObstacleCB(const sensor_msgs::PointCloudConstPtr & pc)
    {
        if(!pc->points.empty())
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

            obstacle_list_.clear();

            obstacle_list_=cloud_out.points;
        }
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
    
        // pc_x.push_back(this->current_pose_.x);
        // pc_y.push_back(this->current_pose_.y);

        for(auto &i : cloud_out.points)
        {
            pc_x.push_back(i.x);
            pc_y.push_back(i.y);
            // cout<<i.x<<","<<i.y<<endl;
        }

        s_=this->calS(pc_x, pc_y);
        // cout<<"callback(s):"<<s_.empty()<<endl;
        sx_.getSXY(s_, pc_x);
        sy_.getSXY(s_, pc_y);
    }

    void frenetCB(const detecting_wall::frenetConstPtr &f)
    {
        this->d0=f->d;
        this->iyaw=f->yaw;
    }

    void currentPoseCB(const geometry_msgs::Pose2DConstPtr &cp)
    {
        
        this->current_pose_.x=cp->x;
        this->current_pose_.y=cp->y;
        this->current_pose_.theta=cp->theta;
        // cout<<this->current_pose_.x<<endl;
    }

    vector<float> calS(vector<float> xp, vector<float>yp)
    {
        // vector<float> dx, dy;
        ds_.clear();

        vector<float> s_;
        s_.push_back(0);
        
        float diff_x, diff_y;
        float abs_diff;
        
        for(int i=0; i<xp.size()-1; i++)
        {
            diff_x=xp[i+1]-xp[i];
            diff_y=yp[i+1]-yp[i];
            // dx.push_back(diff_x);
            // dy.push_back(diff_y);
            abs_diff = sqrt(pow(diff_x,2) + pow(diff_y,2));
            ds_.push_back(abs_diff);
            s_.push_back(s_[i]+abs_diff);
        }

        return s_;
    }

};

vector<FrenetPath> calcFrenetPaths(float c_speed, float c_accel, float c_d, float c_d_d, float c_d_dd, float s0)
{
    vector<FrenetPath> frenet_paths;
    QuinticPolynomial lat_qp;
    QuarticPolynomial lon_qp;

    for(float di = - 1; di<= 1; di=di+0.1) // maximum road width and d road
    {
        for(float Ti = 0.1; Ti <= 30; Ti=Ti+0.1) // min time and max time, dt
        {
            // cout<<Ti<<endl;
            FrenetPath fp;
            lat_qp.getParam(c_d, c_d_d, c_d_dd, di, 0, 0, Ti);
            
            for(float i =0; i<=Ti; i=i+0.1)
            {
                fp.t.push_back(i);
                fp.d.push_back(lat_qp.calPoint(i));
                fp.d_d.push_back(lat_qp.calFirstDerivative(i));
                fp.d_dd.push_back(lat_qp.calSecondDerivative(i));
                fp.d_ddd.push_back(lat_qp.calThirdDerivative(i));
            }

            for(float tv = 0.1-0.1; tv<=0.1+0.1; tv=tv+0.05) //about sampling speed
            {
                FrenetPath tfp = fp;
                float Jp=0, Js=0;
                lon_qp.getParam(s0, c_speed, c_accel, tv, 0.0, Ti);
                for(float i =0; i<=Ti; i=i+0.1)
                {
                    tfp.s.push_back(lon_qp.calPoint(i));
                    tfp.s_d.push_back(lon_qp.calFirstDerivative(i));
                    tfp.s_dd.push_back(lon_qp.calSecondDerivative(i));
                    tfp.s_ddd.push_back(lon_qp.calThirdDerivative(i));
                    Jp=Jp+tfp.d_ddd.back()*tfp.d_ddd.back();
                    Js=Js+tfp.s_ddd.back()*tfp.s_ddd.back();
                }

                float ds = pow( 0.1 - tfp.s_d.back(), 2); //target speed
                tfp.cd = 0.1*Jp + 0.1*Ti + 1.0*tfp.d.back()*tfp.d.back(); //cost weight
                tfp.cv = 0.1*Js + 0.1*Ti + 1.0*ds;
                tfp.cf = 1.0*tfp.cd + 1.0*tfp.cv;

                frenet_paths.push_back(tfp);
            }
        }
    }
    return frenet_paths;
}

vector<FrenetPath> calcGlobalPaths(vector<FrenetPath> fplist, PoseDrawer &csp, float y0)
{
    vector<FrenetPath> answer;
    float old_yaw = y0;
    for(auto &fp : fplist)
    {
        for(int j =0; j<fp.s.size(); j++)
        {
            geometry_msgs::Pose2D ipos = csp.calcPosition(fp.s[j]);
            float di = fp.d[j];
            float fx = ipos.x + di*cos(ipos.theta + 1.570796);
            float fy = ipos.y + di*sin(ipos.theta + 1.570796);
            fp.x.push_back(fx);
            fp.y.push_back(fy);
        }

        for(int j=0; j<fp.x.size()-1; j++)
        {
            float dx = fp.x[j+1]-fp.x[j];
            float dy = fp.y[j+1]-fp.y[j];
            float yaw = atan2(dy, dx);
            float vyaw = (yaw - old_yaw)/0.1;
            float v = sqrt(pow(dx, 2)+pow(dy, 2))/0.1;
            old_yaw = yaw;
            fp.v.push_back(v);
            fp.yaw.push_back(yaw);
            fp.vyaw.push_back(vyaw);
            fp.ds.push_back(sqrt(pow(dy, 2)+pow(dx,2)));
        }

        fp.yaw.push_back(fp.yaw.back());
        fp.ds.push_back(fp.ds.back());
        fp.vyaw.push_back(0);
        fp.v.push_back(fp.v.back());

        for(int i =0; i<fp.yaw.size() -1; i++)
        {
            fp.c.push_back((fp.yaw[i+1]-fp.yaw[i])/fp.ds[i]);
        }
        answer.push_back(fp);
    }

    return answer;
};

bool checkCollision(FrenetPath fp, vector<geometry_msgs::Point32> obs)
{
    bool flag=false;
    float minimum_dis = 0.45;
    for(int i=0; i<fp.x.size(); i++)
    {
        if(obs.empty())
        {
            break;
        }
        else
        {
            for(auto &j : obs)
            {
                if(sqrt(pow(j.x-fp.x[i],2)+pow(j.y-fp.y[i],2))<minimum_dis)
                {
                    flag=true;
                    break;
                }
            }
            if(flag)
            {
                break;
            }
        }
        
    }


    return flag;
}

vector<FrenetPath> checkPaths(vector<FrenetPath> fplist, vector<geometry_msgs::Point32> ob)
{
    vector<FrenetPath> ok_path;
    
    for(auto &i: fplist)
    {
        bool flag = false;
        for(auto &j: i.s_d)
        {
            if(j>2.0) //max_speed(2.0m/s)
            {
                flag=true;
                break;
            }
        }
        if(flag)
        {
            continue;
        }
        for(auto &j: i.s_dd)
        {
            if(abs(j)>0.5) //max_accel(1.0m/ss)
            {
                flag=true;
                break;
            }
        }
        if(flag)
        {
            continue;
        }
        for(auto &j: i.c)
        {
            if(abs(j)>1.0) //max_curvature(1.0m/s)
            {
                flag=true;
                break;
            }
        }
        if(flag)
        {
            continue;
        }

        //check obstacle colloision
        if(checkCollision(i ,ob))
        {
            flag=true;
            continue;
        }

        for(auto &j: i.vyaw)
        {
            if(abs(j)>2.0) //max_twist(1.0rad/s)
            {
                flag=true;
                break;
            }
        }
        if(flag)
        {
            continue;
        }
        // cout<<"there is ok"<<endl;
        ok_path.push_back(i);
    }

    return ok_path;
}



FrenetPath frenetOptimalPlanning(PoseDrawer & csp, float s0, float c_speed, float c_accel, float c_d, float c_d_d, float c_d_dd, vector<geometry_msgs::Point32> ob, float y0)
{
    FrenetPath best_path;
    vector<FrenetPath> fplist = calcFrenetPaths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0);
    fplist = calcGlobalPaths(fplist, csp, y0);
    fplist = checkPaths(fplist, ob);

    if(fplist.empty())
    {
        best_path.defalt=true;
    }
    else
    {
        float min_cost = INFINITY;
        for(auto & fp : fplist)
        {
            if(min_cost >= fp.cf)
            {
                min_cost = fp.cf;
                best_path = fp;
            }
        }
    }
    
    return best_path;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_to_world_link");

    
    PoseDrawer pd;
    ros::Rate r(10);
    ros::Rate r2(1);

    cout<<"0"<<endl;
    for(int i=0; i<4; i++)
    {
        ros::spinOnce();
        r2.sleep();
    }
    
    cout<<"1"<<endl;
    // creating path
    // vector<geometry_msgs::Pose2D> target_pos;
    // vector<float> culvature;
    // for(float i=0.0; i<1.8; i+=0.1)
    // {
    //     target_pos.push_back(pd.calcPosition(i));
    //     culvature.push_back(pd.calCurvature(i));
    // }

    cout<<"2"<<endl;
    //obstacle
    vector<geometry_msgs::Point32> ob=pd.getObstacle();
    cout<<"3"<<endl;
    //initial pose
    geometry_msgs::Pose2D curpos;
    geometry_msgs::Pose2D frepos;
    curpos=pd.getCurrentPose();
    cout<<"4"<<endl;
    frepos=pd.calcPosition(0);
    cout<<"5"<<endl;
    vector<float> initialFrenet; //0: d, 1: yaw
    initialFrenet=pd.getInitalFrenet();
    cout<<"6"<<endl;
    float yawi = initialFrenet[1];

    float c_speed = 0.5*cos(yawi); //current speed
    float c_accel = 0.0; //current acceleration
    
    float c_d_d=0.5*sin(yawi); // current lateral speed
    float c_d_dd=0; // current lateral acceleration
    
    float s0=0; // current cousrse postiion
    float c_d =initialFrenet[0];
    // pd.publishCmd(0.5, 0);
    
    cout<<"0"<<endl;
    r.sleep();
    FrenetPath path_;
    while(ros::ok())
    {
        path_ = frenetOptimalPlanning(pd, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, curpos.theta);
        if(path_.defalt)
        {
            bool is_turning = false;
            bool turning_flag = true;
            float start_theta = 0;
            float object_theta = 0;
            
            while(turning_flag)
            {
                if(!is_turning)
                {
                    is_turning=true;
                    start_theta=curpos.theta;
                    if(start_theta<0)
                    {
                        object_theta=3.141592+start_theta;
                    }
                    else
                    {
                        object_theta=-3.141592+start_theta;
                    }
                }
                else
                {
                    double refined_gap=0;
                    double gap=object_theta - curpos.theta;
                    if(gap>3.141592)
                        refined_gap=-6.283184+gap;
                    else if(gap < -3.141592)
                        refined_gap=6.283184+gap;
                    else
                        refined_gap=gap;
                    
                    if(abs(refined_gap)<=0.1)
                    {
                        is_turning=false;
                        pd.publishCmd(0, 0);
                        turning_flag=false;
                    }
                    else
                    {
                        if(refined_gap>=0)
                        {
                            pd.publishCmd(0, 0.15);
                        }
                        else
                        {
                            pd.publishCmd(0, -0.15);
                        }
                    }
                    
                }

                ros::spinOnce();
                curpos=pd.getCurrentPose();
                r.sleep();

            }
            
            for(int i=0; i<2; i++)
            {
                r2.sleep();
                ros::spinOnce();
            }
        }
        else
        {
            pd.publishCmd(0.5,0);
            for(int i=0; i<path_.v.size()-1; i++)
            {
                r.sleep();
                pd.publishCmd(path_.v[i+1], path_.vyaw[i+1]);
                s0 = path_.s[i+1];
                c_d = path_.d[i+1];
                c_d_dd = path_.d_d[i+1];
                c_speed=path_.s_d[i+1]; 
                c_accel=path_.s_dd[i+1];
                curpos.theta = path_.yaw[i+1];
                pd.publishWaypoints(path_.x, path_.y);

            }

            r.sleep();
            pd.publishCmd(0,0);
            for(int i=0; i<2; i++)
            {
                r2.sleep();
                ros::spinOnce();
            }


            curpos=pd.getCurrentPose();
            frepos=pd.calcPosition(0);
            initialFrenet=pd.getInitalFrenet();
            ob=pd.getObstacle();

            yawi = initialFrenet[1];
            c_speed = 0.5*cos(yawi);
            c_accel = 0.0;

            c_d_d=0.5*sin(yawi);
            c_d_dd=0;

            s0=0; // current cousrse postiion
            c_d =initialFrenet[0];

            // target_pos.push_back(pd.calcPosition(i));
            // culvature.push_back(pd.calCurvature(i));
        }
    }
    
    

    
    return 0;
}