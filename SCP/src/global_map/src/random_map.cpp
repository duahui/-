#include"map/random_map.h"

#include<random>
#include<cmath>
#include<iostream>


namespace map_
{
    

    void random_map::init(ros::NodeHandle &nh)
    {
        nh.param("/has_map",has_map,false);
        nh.param("/map/x_size",x_size,20.0);  //地图面积是50×50,但搜索算法处理时，不一定要50×50，应该是50*5的大小
        nh.param("/map/y_size",y_size,20.0);
        nh.param("/map/z_size",z_size,10.0);
        nh.param("/map/dim_flag",dim_flag,false);

        nh.param("map/obs_num",ob_num,30);
        nh.param("map/resolution",resolution,0.1);

        nh.param("ObstacleShape/lower_wid",w_l,0.3);
        nh.param("ObstacleShape/upper_wid",w_h,0.8);
        nh.param("ObstacleShape/lower_hei",h_l,3.0);
        nh.param("ObstacleShape/upper_hei",h_h,7.0);

        nh.param("init_state_x",init_x,0.0);
        nh.param("init_state_y",init_y,0.0);
        
        nh.param("sensing/rate",rate,1.0);
        

        x_l=-x_size/2.0;
        x_h=x_size/2.0;
        y_l=-y_size/2.0;
        y_h=y_size/2.0;

        map_pub=nh.advertise<sensor_msgs::PointCloud2>("global_map",1);
        map_ob_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map_ob",1);
        static_ob_pub = nh.advertise<global_map::Obstacles>("static_obs",10);
        dynamic_ob_pub = nh.advertise<global_map::DynamicObstacles>("dynamic_obs",1);
    }
    using namespace std;   
     
    void random_map::PubMap()
    {
        if (has_map){
            // map_pub.publish(global_map); 
            // map_ob_pub.publish(global_map_ob);
            static_ob_pub.publish(static_obs);
            dynamic_ob_pub.publish(dynamic_obs);
            return;
        }
        // RandomMapGenerate();
        ob_polygon();
        // global_map.header.frame_id="world";
        // global_map_ob.header.frame_id="world";
        has_map=true;  
      
    }

    void random_map::ob_polygon()
    {
        static_obs.obstacles.clear();

        /* 下 */
        geometry_msgs::Polygon polygon;

        //左下
        geometry_msgs::Point32 point1;
        point1.x =  x_l;
        point1.y =  y_l;
        point1.z =  0;
        polygon.points.push_back(point1);
        //右下
        geometry_msgs::Point32 point2;
        point2.x = x_h + 0.1;
        point2.y = y_l;
        point2.z = 0;
        polygon.points.push_back(point2);
        //右上
        geometry_msgs::Point32 point3;
        point3.x  = x_h + 0.1;
        point3.y  = y_l + 0.1 - 0.5 ;
        point3.z  = 0;
        polygon.points.push_back(point3);
        //左上
        geometry_msgs::Point32 point4;
        point4.x  = x_l;
        point4.y  = y_l + 0.1 - 0.5;
        point4.z  = 0;
        polygon.points.push_back(point4);

        static_obs.obstacles.push_back(polygon);

        /* 右 */
        polygon.points.clear();
        //左下
        point1.x =  x_h;
        point1.y =  y_l;
        point1.z =  0;
        polygon.points.push_back(point1);
        //右下
        point2.x = x_h + 0.4;
        point2.y = y_l;
        point2.z = 0;
        polygon.points.push_back(point2);
        //右上
        point3.x  = x_h + 0.4;
        point3.y  = y_h;
        point3.z  = 0;
        polygon.points.push_back(point3);
        //坐上
        point4.x  = x_h;
        point4.y  = y_h;
        point4.z  = 0;
        polygon.points.push_back(point4);

        static_obs.obstacles.push_back(polygon);

        /* 上 */
        polygon.points.clear();
        //左下
        point1.x =  x_l;
        point1.y =  y_h;
        point1.z =  0;
        polygon.points.push_back(point1);
        //右下
        point2.x = x_h;
        point2.y = y_h;
        point2.z = 0;
        polygon.points.push_back(point2);
        //右上
        point3.x  = x_h;
        point3.y  = y_h + 0.4;
        point3.z  = 0;
        polygon.points.push_back(point3);
        //右下
        point4.x  = x_l;
        point4.y  = y_h + 0.4;
        point4.z  = 0;
        polygon.points.push_back(point4);

        static_obs.obstacles.push_back(polygon);

        /* 左 */
        polygon.points.clear();
        //左下
        point1.x =  x_l - 0.4;
        point1.y =  y_l;
        point1.z =  0;
        polygon.points.push_back(point1);
        //右下
        point2.x = x_l;
        point2.y = y_l;
        point2.z = 0;
        polygon.points.push_back(point2);
        //右上
        point3.x  = x_l;
        point3.y  = y_h;
        point3.z  = 0;
        polygon.points.push_back(point3);
        //右下
        point4.x  = x_l - 0.4;
        point4.y  = y_h;
        point4.z  = 0;
        polygon.points.push_back(point4);

        static_obs.obstacles.push_back(polygon);

        //中间
        polygon.points.clear();
        generate_polygon(0.0, y_l, 1.0, 0.0, polygon);
        static_obs.obstacles.push_back(polygon);

        //dynamic_obs
        // generate_dynamic_polygon(x_h/4, 3, 2, 10, 1.0, 0.4, 13.0, M_PI);
        // generate_dynamic_polygon(x_h/4, x_l * 3/4, 2, 2, 3.0, 0.4, 13.0, M_PI/2);
        generate_dynamic_polygon(x_h*3/4, y_l*3/4, 2.0, 2.0, 0.0, 0.4, 10.0, M_PI/2);
        generate_dynamic_polygon(x_l/8, 0.0, 2.0, 5.0, 15.0, 0.4, 18.0, M_PI/2);
        generate_dynamic_polygon(x_h/2, y_l /2, 2.0, 2.0, 3.0, 0.4, 9.0, M_PI*3/4);
    }
    void random_map::generate_polygon(double x_l, double y_l, double x_h, double y_h, geometry_msgs::Polygon &polygon)
    { 
        //左下
        geometry_msgs::Point32 point1;
        point1.x =  x_l;
        point1.y =  y_l;
        point1.z =  0;
        polygon.points.push_back(point1);
        //右下
        geometry_msgs::Point32 point2;
        point2.x = x_h;
        point2.y = y_l;
        point2.z = 0;
        polygon.points.push_back(point2);
        //右上
        geometry_msgs::Point32 point3;
        point3.x  = x_h;
        point3.y  = y_h;
        point3.z  = 0;
        polygon.points.push_back(point3);
        //左上
        geometry_msgs::Point32 point4;
        point4.x  = x_l;
        point4.y  = y_h;
        point4.z  = 0;
        polygon.points.push_back(point4);

    }

    // polygen是位于原点的矩形，后续会对这个变换
    void random_map::generate_dynamic_polygon(double start_x, double start_y, double length, double width, double start_t, double ts, double tot_t, double phi)
    {
        global_map::DynamicObstacle temp;
        geometry_msgs::Polygon polygon_temp;
        generate_polygon(0, 0, length, width, polygon_temp);
        temp.polygon = polygon_temp;
        double step = (tot_t-start_t)/ts;
        for (int i = 0; i < (int)step; ++i)
        {
            global_map::DynamicTrajectoryPoint point;
            point.x = start_x + i * ts * 1 * cos(phi);
            point.y = start_y + i * ts * 1 * sin(phi);
            point.time = start_t + i * ts;
            point.theta = phi;
            temp.trajectory.push_back(point);
        }
        dynamic_obs.obstacles.push_back(temp);        
    }

    void random_map::RandomMapGenerate()
    {
        dynamic_obs.obstacles.clear();
        static_obs.obstacles.clear();
        random_device rd;
        default_random_engine end(rd());

        uniform_real_distribution<double> rand_x=uniform_real_distribution<double>(x_l,x_h);
        uniform_real_distribution<double> rand_y=uniform_real_distribution<double>(y_l,y_h);
        uniform_real_distribution<double> rand_w=uniform_real_distribution<double>(w_l,w_h);
        uniform_real_distribution<double> rand_h=uniform_real_distribution<double>(h_l,h_h);

        for (int i = 0; i < ob_num; ++i)
        {

            pcl::PointXYZ pt_random;

            double x,y,w,h;
            x=rand_x(end);
            y=rand_y(end);
            w=rand_w(end);
            h=rand_h(end);
            
            if(sqrt( pow(x - init_x, 2) + pow(y - init_y, 2) ) < 0.8 ) 
                continue;
            x = floor(x/resolution) * resolution + resolution / 2.0;
            y = floor(y/resolution) * resolution + resolution / 2.0;

            int widNum = ceil(w/resolution);//这样做是因为看生成几个ob点，例如，3.5～4.5,可以生成1个、10个、100个ob点。精度一般是0.1,0.001,0.0001等。

            for(int r = -widNum/2.0; r < widNum/2.0;  ++r )
            {
                for(int s = -widNum/2.0; s < widNum/2.0;  ++s )
                {  
                    int heiNum = 2.0 * ceil(h/resolution);
                    for(int t = 0; t < heiNum;  ++ t)
                    {
                        pt_random.x = x + (r+0.0) * resolution + 0.001;
                        pt_random.y = y + (s+0.0) * resolution + 0.001;
                        pt_random.z =     (t+0.0) * resolution * 0.5 + 0.001;
                        random_cloud_3.points.push_back( pt_random );
                    }
                }
            }
            geometry_msgs::Polygon polygon;
            
            //左下
            pcl::PointXYZ ob_vex1;
            geometry_msgs::Point32 point1;
            point1.x = ob_vex1.x = max(x - w/2.0, x_l);
            point1.y = ob_vex1.y = max(y - w/2.0, y_l);
            point1.z = ob_vex1.z = 0;
            random_ob.points.push_back( ob_vex1 );
            polygon.points.push_back(point1);
            //左上
            pcl::PointXYZ ob_vex2;
            geometry_msgs::Point32 point2;
            point2.x = ob_vex2.x = max(x - w/2.0, x_l);
            point2.y = ob_vex2.y = min(y + w/2.0, y_h);
            point2.z = ob_vex2.z = 0;
            random_ob.points.push_back( ob_vex2 );
            polygon.points.push_back(point2);
            //右上
            pcl::PointXYZ ob_vex4;
            geometry_msgs::Point32 point3;
            point3.x = ob_vex4.x = min(x + w/2.0, x_h);
            point3.y = ob_vex4.y = min(y + w/2.0, y_h);
            point3.z = ob_vex4.z = 0;
            random_ob.points.push_back( ob_vex4 );
            polygon.points.push_back(point3);
            //右下
            pcl::PointXYZ ob_vex3;
            geometry_msgs::Point32 point4;
            point4.x = ob_vex3.x = min(x + w/2.0, x_h);
            point4.y = ob_vex3.y = max(y - w/2.0, y_l);
            point4.z = ob_vex3.z = 0;
            random_ob.points.push_back( ob_vex3 );
            polygon.points.push_back(point4);
            
            static_obs.obstacles.push_back(polygon);
        }

        if(dim_flag)
        {
            for (auto &point:random_cloud_3)
            {                
                point.z=0.0;  
            }  
            // ROS_INFO("true ,%f",random_cloud_3.points[0].z);         
        }
        random_cloud_3.is_dense=true;
        random_cloud_3.width=random_cloud_3.points.size();
        random_cloud_3.height=1;
        pcl::toROSMsg(random_cloud_3,global_map);

        random_ob.is_dense=true;
        random_ob.width=random_ob.points.size();
        random_ob.height=1;
        pcl::toROSMsg(random_ob,global_map_ob);
        
        return;

    }



}