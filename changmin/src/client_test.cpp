#include "ros/ros.h"

#include "frenet/FrenetAction.h"
#include <actionlib/client/simple_action_client.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");
    actionlib::SimpleActionClient<frenet::FrenetAction> ac("Frenet", true);
    ac.waitForServer();

    frenet::FrenetGoal goal;
    goal.start = true;
    while(ros::ok())
    {
        ac.sendGoal(goal);
        ac.waitForResult();
    }
    
    return 0;
}