
#include"nav/planner.h"

int main(int argc, char  *argv[])
{
    /* code */

    ros::init(argc,argv,"nav");
    ros::NodeHandle nh("~");

    nav::planner nav_planner(nh);

    while (ros::ok())
    {        
        ros::spinOnce();
        nav_planner.planning();
    }
       

    return 0;
}