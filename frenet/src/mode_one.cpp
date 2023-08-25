#include "frenet/CublicSpline.hpp"
#include "frenet/FrenetPath.hpp"
#include "frenet/QuinticPolynomial.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "obstacle_detector/Obstacles.h"

#include <actionlib/server/simple_action_server.h>
#include "frenet/FrenetAction.h"

#include "Eigen/Dense"

#define RADIAN_INC 0.0087
#define START_SPEED 0.25
#define K_J 0.01
#define K_T 0.1
#define K_D 1.0
#define K_LAT 1.5
#define K_LON 1.0

using namespace std;

class FindPath
{
public:
    FindPath(string name) /*inital the class*/
    : rate(10), action_name_(name),
    as_(nh_, name, boost::bind(&FindPath::executeCB, this, _1), false)
    {   
        if(!nh_.getParam("/mode_one/step_distance", this->step_distance_))
            this->step_distance_ = 1.0;
        
        as_.start();
        this->scan_sub_=nh_.subscribe("/projected/scan", 10, &FindPath::laserScanCB, this);
        this->obs_sub_ =nh_.subscribe("/raw_obstacles", 10, &FindPath::obstacleCB, this);
        // this->way_pub_=nh_.advertise<sensor_msgs::PointCloud>("/frenet/waypoint", 10);
        this -> way_pub_ = nh_.advertise<nav_msgs::Path>("/frenet/waypoint", 10);
        this->cmd_pub_=nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        this -> global_pub_ = nh_.advertise<nav_msgs::Path>("/global/path", 10);
        // this->global_pub_=nh_.advertise<sensor_msgs::PointCloud>("/global/path", 10);
    }

    /*what to do when action come*/
    void executeCB(const frenet::FrenetGoalConstPtr & goal)
    {
        // ros::spinOnce();
        bool success = goal->start;
        this->initFrenets();
        
        float c_speed = START_SPEED*cos(this->yaw0);
        float c_d_d = START_SPEED*sin(this->yaw0);

        float s0 =0;
        float c_d = this->d0;

        float c_accel = 0.0;
        float c_d_dd=0;

        if(as_.isPreemptRequested())
        {
            as_.setPreempted();
            success=false;
            return;
        }

        FrenetPath path_;
        path_=frenetOptimalPlanning(s0, c_speed, c_accel, c_d, c_d_d, c_d_dd);

        if(path_.defalt)
        {
            result_.finish=false;
            as_.setSucceeded(result_);
            return;
        }
        this->publishWaypoints(path_);

        publishCmd(START_SPEED, 0);

        if(as_.isPreemptRequested())
        {
            as_.setPreempted();
            success=false;
            return;
        }

        for(int i=0; i<path_.v.size()-1; i++)
        {
            if(as_.isPreemptRequested())
            {
                as_.setPreempted();
                success=false;
                break;
            }
            rate.sleep();
            publishCmd(path_.v[i+1], path_.vyaw[i+1]);
        }

        rate.sleep();
        publishCmd(0,0);
        if(!success)
            return;
        else
        {
            result_.finish=true;
            as_.setSucceeded(result_);
        }
    }

private:
    /*ROS DEFAULT*/
    ros::NodeHandle nh_;
    ros::Rate rate;

    /*PUBLISH SUBSCRIBE*/
    ros::Subscriber scan_sub_;
    ros::Subscriber obs_sub_;
    ros::Publisher way_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher global_pub_;

    /*MAKING GLOABAL PATH*/
    Eigen::Vector4f middle_line_;
    Eigen::Vector4f left_line_;
    Eigen::Vector4f right_line_;


    /*RESULT FOR LASER SCAN*/
    obstacle_detector::Obstacles obs_;
    
    /*Param for robot in FrenetFrame*/
    float d0, yaw0;
    CublicSpline1D sx_, sy_;
    float step_distance_;

    /*for the frenet action server*/
    actionlib::SimpleActionServer<frenet::FrenetAction> as_;
    string action_name_;
    frenet::FrenetResult result_;

    /*FOR CALCULATING GLOBAL PATH AND OBSTACLE*/
    void laserScanCB(const sensor_msgs::LaserScanConstPtr &scan)
    {
        // cout<<"callback called"<<endl;
        // obs_vec.clear();
        float x_temp, y_temp;
        float left_y=-0.5, right_y=0.5;
        geometry_msgs::Point obs;
        vector<float> right_x_vec;
        vector<float> right_y_vec;
        vector<float> left_x_vec;
        vector<float> left_y_vec;

        for(int i=0; i<scan->ranges.size(); i++)
        {
            if(scan->ranges[i]!=INFINITY)
            {
                x_temp=scan->ranges[i]*cos(i*RADIAN_INC)*(-1);
                y_temp=scan->ranges[i]*sin(i*RADIAN_INC)*(-1);

                if( -1 <= x_temp && x_temp <= 3.0)
                {
                    // if(x_temp>0)
                    // {
                    //     obs.x=x_temp;
                    //     obs.y=y_temp;
                    //     obs.z=0;
                    //     obs_vec.push_back(obs);
                    // }

                    if(y_temp>0.6)
                    {
                        right_x_vec.push_back(x_temp);
                        right_y = right_y*0.8 + y_temp*0.2; //for the low pass filter
                        right_y_vec.push_back(right_y);
                    }
                    else if(y_temp<-0.6)
                    {
                        left_x_vec.push_back(x_temp);
                        left_y = left_y*0.8 + y_temp*0.2; //for the loe psdd filter
                        left_y_vec.push_back(left_y);
                    }
                    else
                    {}
                }
            }
        }

        right_line_=linearFitting(right_x_vec, right_y_vec);
        left_line_=linearFitting(left_x_vec, left_y_vec);
        this->calcMiddleLine();
    }

    void obstacleCB(const obstacle_detector::ObstaclesConstPtr &obs)
    {
        obs_.circles=obs->circles;
        obs_.segments=obs->segments;
    }

    Eigen::Vector4f linearFitting(vector<float> xvec, vector<float> yvec)
    {
        Eigen::VectorXf X_TRIPLE;
        Eigen::VectorXf X_SQUARE;
        Eigen::VectorXf X;
        Eigen::VectorXf Y;
        Eigen::VectorXf SIGMA;
        Eigen::MatrixXf A;
        Eigen::Matrix4f temp;

        X_TRIPLE=Eigen::VectorXf(xvec.size());
        X_SQUARE=Eigen::VectorXf(xvec.size());
        X=Eigen::VectorXf(xvec.size());
        Y=Eigen::VectorXf(yvec.size());
        SIGMA=Eigen::VectorXf(xvec.size());

        for(int ii=0; ii<xvec.size(); ii++)
        {
            X_TRIPLE[ii]=xvec[ii]*xvec[ii]*xvec[ii];
            X_SQUARE[ii]=xvec[ii]*xvec[ii];
            X[ii]=xvec[ii];
            Y[ii]=yvec[ii];
            SIGMA[ii]=1;
        }

        A=Eigen::MatrixXf(xvec.size(),4);
        A.col(0)<<X_TRIPLE;
        A.col(1)<<X_SQUARE;
        A.col(2)<<X;
        A.col(3)<<SIGMA;

        temp=A.transpose()*A;

        return temp.inverse()*A.transpose()*Y;
    }

    void calcMiddleLine()
    {
        //left line, middle line, right line
        vector<Eigen::Vector4f> line;
        double min_distance[3];
        int line_cost[3];

        for(int i=0; i<3; i++)
        {
            Eigen::Vector4f temp_line;
            temp_line[0] =(left_line_[0]+right_line_[0])/2;
            temp_line[1] =(left_line_[1]+right_line_[1])/2;
            temp_line[2] =(left_line_[2]+right_line_[2])/2;
            temp_line[3] =(left_line_[3]+right_line_[3])/2;
            line.push_back(temp_line);
        }
        line[0][3] += 0.5;
        line[2][3] -= 0.5;

        //cost of line [left lane, middle lane , right lane] 
        line_cost[0] = 5;
        line_cost[1] = 0;
        line_cost[2] = 10;

        //min distance of line[left min distace, middle min distance, right min distance]
        for(int i=0; i<3; i++)
        {
            min_distance[i]=std::numeric_limits<double>::max();
        }
        
        //calculating min distance with segments
        for(const auto& obs_line: obs_.segments)
        {
            Eigen::Vector2d pointA;
            Eigen::Vector2d pointB;

            pointA[0]=obs_line.first_point.x;
            pointA[1]=obs_line.first_point.y;

            pointB[0]=obs_line.last_point.x;
            pointB[1]=obs_line.last_point.y;

            Eigen::Vector2d lineOrigin=pointA;
            Eigen::Vector2d lineDirection= pointB - pointA;

            for(double t=0.0; t<=1.0; t+=0.05)
            {
                Eigen::Vector2d currentPoint = lineOrigin + t*lineDirection;
                double y_online[3];
                for(int i=0; i<3; i++)
                {
                    y_online[i]= line[i][0]*t*t*t + line[i][1]*t*t + line[i][2]*t+line[i][3];
                    if(std::abs(currentPoint.y()-y_online[i])<min_distance[i])
                    {
                        min_distance[i] = std::abs(currentPoint.y() - y_online[i]);
                    }
                }
            }
        }

        for(int i=0; i<3; i++)
        {
            if(min_distance[i] < 0.35)
                line_cost[i]+=100;
        }

        for(int i=0; i<3; i++)
        {
            min_distance[i]=std::numeric_limits<double>::max();
        }
        
        for(const auto& cir: obs_.circles)
        {
            double y_online[3];
            for(double x=0.0; x<=2.0; x+=0.1)
            {
                for(int i=0; i<3; i++)
                {
                    y_online[i]=line[i][0]*x*x*x + line[i][1]*x*x + line[i][2]*x+line[i][3];
                    double distance = sqrt((cir.center.x-x)*(cir.center.x-x)+(cir.center.y-y_online[i])*(cir.center.y-y_online[i]));
                    if(distance<min_distance[i])
                    {
                        min_distance[i]=distance;
                    }
                }

            }  
        }

        for(int i=0; i<3; i++)
        {
            if(min_distance[i] < 0.5)
                line_cost[i]+=100;
        }
        
        int best_iteration=0;
        int best_cost=std::numeric_limits<int>::max();

        for(int i=0; i<3; i++)
        {
            if(line_cost[i]<best_cost)
            {
                best_iteration=i;
                best_cost = line_cost[i];
            }
        }

        this->middle_line_=line[best_iteration];
    }

    /*FOR CALUCLATING FRENET FRAME*/
    void initFrenets()
    {
        // sensor_msgs::PointCloud global_path;
        nav_msgs::Path global_path;
        global_path.header.frame_id="velodyne";
        geometry_msgs::PoseStamped points;

        d0 = -(-0.027*middle_line_[0]+0.09*middle_line_[1] - 0.3*middle_line_[2] + middle_line_[3]); //offset velodyne and scout -0.3
        yaw0 = -atan2(3*0.09*middle_line_[0]-2*middle_line_[1]*0.3+middle_line_[2],1);
        
        vector<float> s_;
        vector<float> p_x, p_y;
        float x_temp, y_temp;

        for(float i=-0.3; i<1.5; i=i+0.1)
        {
            x_temp=i;
            y_temp=i*i*i*middle_line_[0] + i*i*middle_line_[1] + i*middle_line_[2] + middle_line_[3];
            p_x.push_back(x_temp);
            p_y.push_back(y_temp);
            
            // points.x=x_temp;
            // points.y=y_temp;
            points.pose.position.x = x_temp;
            points.pose.position.y = y_temp;
            points.pose.position.z = 0;

            global_path.poses.push_back(points);
            // global_path.points.push_back(points);
        }
        global_path.header.stamp=ros::Time::now();
        global_pub_.publish(global_path);

        s_=this->calS(p_x, p_y);
        sx_.getSXY(s_, p_x);
        sy_.getSXY(s_, p_y);
    }

    vector<float> calS(vector<float> xp, vector<float>yp)
    {
        vector<float> ds_;
        vector<float> s_;
        
        s_.push_back(0);
        
        float diff_x, diff_y;
        float abs_diff;
        
        for(int i=0; i<xp.size()-1; i++)
        {
            diff_x=xp[i+1]-xp[i];
            diff_y=yp[i+1]-yp[i];
            abs_diff = sqrt(pow(diff_x,2) + pow(diff_y,2));
            ds_.push_back(abs_diff);
            s_.push_back(s_[i]+abs_diff);
        }

        return s_;
    }

    FrenetPath frenetOptimalPlanning(float s0, float c_speed, float c_accel, float c_d, float c_d_d, float c_d_dd)
    {
        FrenetPath best_path;
        vector<FrenetPath> fplist = calcFrenetPaths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0);
        fplist = calcGlobalPaths(fplist);
        
        fplist = checkPaths(fplist);

        cout<<fplist.size()<<endl;
        // this->publishWaypoints(fplist);
        

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

    vector<FrenetPath> calcFrenetPaths(float c_speed, float c_accel, float c_d, float c_d_d, float c_d_dd, float s0)
    {
        vector<FrenetPath> frenet_paths;
        QuinticPolynomial heading_qp;
        QuarticPolynomial perpend_qp;

        float tc = this -> step_distance_;
        
        for(float tv =0.1; tv<=0.2; tv=tv+0.01)
        {
            for(float Ti = 0.3; Ti <= 3; Ti=Ti+0.1) // min time and max time, dt
            {
                FrenetPath fp;
                heading_qp.getParam(s0, c_speed, c_accel, tc, tv, 0.0, Ti);
                for(float i =0; i<=Ti; i=i+0.1)
                {
                    fp.t.push_back(i);
                    fp.s.push_back(heading_qp.calPoint(i));
                    fp.s_d.push_back(heading_qp.calFirstDerivative(i));
                    fp.s_dd.push_back(heading_qp.calSecondDerivative(i));
                    fp.s_ddd.push_back(heading_qp.calThirdDerivative(i));
                }
                for(float td_d = -0.08; td_d<=0.08; td_d=td_d+0.01) //about sampling speed
                {
                    FrenetPath tfp = fp;
                    float Jp=0, Js=0;
                    perpend_qp.getParam(c_d, c_d_d, c_d_dd, td_d, 0.0, Ti);
                    for(float i =0; i<=Ti; i=i+0.1)
                    {
                        tfp.d.push_back(perpend_qp.calPoint(i));
                        tfp.d_d.push_back(perpend_qp.calFirstDerivative(i));
                        tfp.d_dd.push_back(perpend_qp.calSecondDerivative(i));
                        tfp.d_ddd.push_back(perpend_qp.calThirdDerivative(i));
                        Jp=Jp+tfp.d_ddd.back()*tfp.d_ddd.back();
                        Js=Js+tfp.s_ddd.back()*tfp.s_ddd.back();
                    }
                    float ds = pow( 0.0 - tfp.s_d.back(), 2); //target speed
                    tfp.cd = K_J*Jp + K_T*Ti + K_D*tfp.d.back()*tfp.d.back(); //cost weight
                    tfp.cv = K_J*Js + K_T*Ti + K_D*ds;
                    tfp.cf = K_LAT*tfp.cd + K_LON*tfp.cv;
                    frenet_paths.push_back(tfp);
                }
            }    
        }
            
        
        return frenet_paths;
    }

    vector<FrenetPath> calcGlobalPaths(vector<FrenetPath> fplist)
    {
        vector<FrenetPath> answer;
        float old_yaw = yaw0;
        for(auto &fp : fplist)
        {
            for(int j =0; j<fp.s.size(); j++)
            {
                geometry_msgs::Pose2D ipos = this->calcPosition(fp.s[j]);
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

    geometry_msgs::Pose2D calcPosition(float s)
    {
        geometry_msgs::Pose2D pos;
        pos.x=sx_.calcPosition(s);
        pos.y=sy_.calcPosition(s);
        float dx = sx_.clacFirstDerivative(s);
        float dy = sy_.clacFirstDerivative(s);
        pos.theta=atan2(dy, dx);
        return pos;
    }
    vector<FrenetPath> checkPaths(vector<FrenetPath> fplist)
    {
        vector<FrenetPath> ok_path;
        
        for(auto &i: fplist)
        {
            bool flag = false;
            
            for(auto &j: i.s_d)
            {
                if(j>0.7) //max_speed(2.0m/s)
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
                if(abs(j)>0.4) //max_accel(1.0m/ss)
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
                if(abs(j)>10.0) //max_curvature(1.0m/s)
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
            // if(checkCollision(i))
            // {
            //     flag=true;
            //     continue;
            // }
            for(auto &j: i.vyaw)
            {
                if(j>4.0) //max_twist(1.0rad/s)
                    j=4.0;
                else if(j< -4.0)
                    j=-4.0;
            }
            
            // for(auto &j: i.vyaw)
            // {
            //     if(abs(j)>5.0) //max_twist(1.0rad/s)
            //     {
            //         flag=true;
            //         break;
            //     }
            // }
            // if(flag)
            // {
            //     continue;
            // }
            // cout<<"there is ok"<<endl;
            ok_path.push_back(i);
        }
    
        return ok_path;
    }

    

    /*PUBLISHING CMD*/
    // void publishWaypoints(vector<FrenetPath> fplist)
    // {
    //     sensor_msgs::PointCloud waypoints;
    //     geometry_msgs::Point32 points;
    //     waypoints.header.frame_id="velodyne";
    //     waypoints.header.stamp=ros::Time::now();

    //     for(int i=0; i<fplist.size(); i++)
    //     {
    //         for(int j=0; j<fplist[i].x.size(); j++)
    //         {
    //             points.x = fplist[i].x[j];
    //             points.y = fplist[i].y[j];
    //             points.z = 0;
    //             waypoints.points.push_back(points);
    //         }
            
    //     }

    //     way_pub_.publish(waypoints);
        
    // }

    void publishWaypoints(FrenetPath fplist)
    {
        nav_msgs::Path waypoints;
        geometry_msgs::PoseStamped points;
        waypoints.header.frame_id="velodyne";
        waypoints.header.stamp=ros::Time::now();

        
        for(int j=0; j<fplist.x.size(); j++)
        {
            points.pose.position.x = fplist.x[j];
            points.pose.position.y = fplist.y[j];
            points.pose.position.z = 0;
            waypoints.poses.push_back(points);
        }

        way_pub_.publish(waypoints);        
    }

    void publishCmd(float v, float w)
    {
        geometry_msgs::Twist vel;
        vel.linear.x = v;
        vel.angular.z = w*1.2;

        cmd_pub_.publish(vel);

    }

};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "mode_one");
    FindPath fp("Frenet");

    ros::spin();

    return 0;
}