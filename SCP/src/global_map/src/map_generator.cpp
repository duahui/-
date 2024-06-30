#include"map/random_map.h"
#include"map/current.h"

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc,argv,"map_generator");
    ros::NodeHandle nh("~");


    map_::random_map _map;
    _map.init(nh);

    map_::current_map _current;
    _current.init(nh);

    /* 
    ros::Publisher example = nh.advertise<geometry_msgs::PoseArray>("example",1);
    geometry_msgs::PoseArray array;
    geometry_msgs::Pose tmp_pose;
    tmp_pose.position.x = 0;
    tmp_pose.position.y = 0;
    tmp_pose.position.z = 0;
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, 0.785);
    tmp_pose.orientation.x = qtn.getX();
    tmp_pose.orientation.y = qtn.getY();
    tmp_pose.orientation.z = qtn.getZ();
    tmp_pose.orientation.w = qtn.getW();
    array.poses.push_back(tmp_pose);

    tmp_pose.position.x = 0;
    tmp_pose.position.y = 1;
    tmp_pose.position.z = 0;
    qtn.setRPY(0, 0, 2.355);
    tmp_pose.orientation.x = qtn.getX();
    tmp_pose.orientation.y = qtn.getY();
    tmp_pose.orientation.z = qtn.getZ();
    tmp_pose.orientation.w = qtn.getW();
    array.poses.push_back(tmp_pose);

    array.header.frame_id = "world";
    array.header.stamp = ros::Time::now();
    */
    ros::Rate loop_rate(_map.get_rate()); //每秒执行几次循环，一般实时性要求10hz，但此处地图只有1hz
    while (ros::ok())
    {
        // example.publish(array);
        _map.PubMap();
        // _current.pub_current();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
