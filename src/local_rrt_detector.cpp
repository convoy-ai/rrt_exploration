#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clicked_points[5]; 
int clicked_point_index = 0;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points, line;
float xdim, ydim, resolution, Xstartx, Xstarty, init_map_x, init_map_y;

rdm r; // for generating random numbers



//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    mapData = *msg;
}


 
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (clicked_point_index < 5) {
        clicked_points[clicked_point_index] = *msg;
        clicked_point_index++;
    }
}




int main(int argc, char **argv) {

    unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
    MTRand_int32 irand(init, length); // 32-bit int generator
    // this is an example of initializing by an array
    // you may use MTRand(seed) with any 32bit integer
    // as a seed for a simpler initialization
    MTRand drand; // double in [0, 1) generator, already init

    // generate the same numbers as in the original C test program

    ros::init(argc, argv, "local_rrt_frontier_detector");
    ros::NodeHandle nh;
  
    // fetching all parameters
    float eta;
    std::string map_topic, base_frame;
    int obstacle_threshold, update_rate;

    std::string ns = ros::this_node::getName();


    ros::param::param<float>(ns + "/eta", eta, 0.5);
    ros::param::param<std::string>(ns + "/map_topic", map_topic, "map");
    ros::param::param<std::string>(ns + "/base_frame", base_frame, "base_link");
    ros::param::param<int>(ns + "/obstacle_threshold", obstacle_threshold, 70);
    ros::param::param<int>(ns + "/rate", update_rate, 20);


    //---------------------------------------------------------------
    ros::Subscriber map_sub= nh.subscribe(map_topic, 100, mapCallBack);
    ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100, rvizCallBack);

    ros::Publisher targets_pub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
    ros::Publisher rviz_pub = nh.advertise<visualization_msgs::Marker>(ns+"/shapes", 10);

    ros::Rate rate(update_rate);


    tf2_ros::Buffer tf_buffer(ros::Duration(15.0));
    tf2_ros::TransformListener tf_listener(tf_buffer);
 
    // wait until map is received
    while (mapData.data.size() < 1)  {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // visualizations: points and lines
    points.header.frame_id=mapData.header.frame_id;
    line.header.frame_id=mapData.header.frame_id;
    points.header.stamp=ros::Time(0);
    line.header.stamp=ros::Time(0);
	
    points.ns = line.ns = "markers";
    points.id = 0;
    line.id = 1;


    points.type = points.POINTS;
    line.type = line.LINE_LIST;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = points.ADD;
    line.action = line.ADD;
    points.pose.orientation.w = 1.0;
    line.pose.orientation.w = 1.0;
    line.scale.x =  0.01;
    line.scale.y = 0.01;
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // red line, red points
    line.color.r =255.0/255.0;
    line.color.g= 0.0/255.0;
    line.color.b =0.0/255.0;
    points.color.r = 255.0/255.0;
    points.color.g = 0.0/255.0;
    points.color.b = 0.0/255.0;
    points.color.a = 1.0;
    line.color.a = 1.0;
    points.lifetime = ros::Duration();
    line.lifetime = ros::Duration();


    while (clicked_point_index < 5) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    geometry_msgs::PointStamped point1 = clicked_points[0];
    geometry_msgs::PointStamped point2 = clicked_points[2];
    geometry_msgs::PointStamped point1_out, point2_out;

    std::string target_frame = mapData.header.frame_id;

    tf_buffer.transform<geometry_msgs::PointStamped>(point1, point1_out, target_frame, ros::Duration(5.0));
    tf_buffer.transform<geometry_msgs::PointStamped>(point2, point2_out, target_frame, ros::Duration(5.0));


    std::vector<float> temp1;
    temp1.push_back(point1_out.point.x);
    temp1.push_back(point1_out.point.y);
	
    std::vector<float> temp2;
    temp2.push_back(point2_out.point.x);
    temp2.push_back(point1_out.point.y);

    float region_width = Norm(temp1, temp2); // in direction of x
    temp2.clear();

    temp2.push_back(point1_out.point.x);
    temp2.push_back(point2_out.point.y);

    float region_height = Norm(temp1, temp2); // in direction of y
    
    temp1.clear();
    temp2.clear();

    float center_x = (point1_out.point.x + point2_out.point.x) * .5;
    float center_y = (point1_out.point.y + point2_out.point.y) * .5;


    std::vector< std::vector<float>> V;

    // find initial robot position
    int found_transform = 0;
    geometry_msgs::TransformStamped transform_stamped;

    while (found_transform==0) {
        try {
            found_transform = 1;
            transform_stamped = tf_buffer.lookupTransform(mapData.header.frame_id, base_frame, ros::Time(0));
        }
        catch (tf2::ExtrapolationException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.5).sleep();
        }
    }

    std::vector<float> init_point;
    init_point.push_back(transform_stamped.transform.translation.x);
    init_point.push_back(transform_stamped.transform.translation.y);

    V.push_back(init_point);


    points.points.clear();
    rviz_pub.publish(points);

    std::vector<float> frontiers;
    float xr,yr;
    std::vector<float> x_rand, x_nearest, x_new;
    geometry_msgs::Point p;

    // ---------------------------------------------------
    
    ROS_INFO("Local detector: begin building RRT");

    // Main loop
    while (ros::ok()) {

        // Sample free
        x_rand.clear();
        xr=(drand() * region_width) - (region_width * 0.5) + center_x;
        yr=(drand() * region_height) - (region_height * 0.5) + center_y;


        x_rand.push_back( xr );
        x_rand.push_back( yr );


        // Nearest
        x_nearest=Nearest(V, x_rand);

        // Steer
        x_new=Steer(x_nearest, x_rand, eta);


        // ObstacleFree    1:free     -1:unknown (frontier region)      0:obstacle
        int checking = ObstacleFree(x_nearest, x_new, mapData, obstacle_threshold);

        if (checking == -1) {
            exploration_goal.header.stamp=ros::Time(0);
            exploration_goal.header.frame_id=mapData.header.frame_id;
            exploration_goal.point.x=x_new[0];
            exploration_goal.point.y=x_new[1];
            exploration_goal.point.z=0.0;
            p.x=x_new[0];
            p.y=x_new[1];
            p.z=0.0;

            points.points.push_back(p);
            rviz_pub.publish(points);
            targets_pub.publish(exploration_goal);

            points.points.clear();
            V.clear();


            geometry_msgs::TransformStamped transform_stamped;
            int found_transform = 0;
            while (found_transform==0) {
                try {
                    found_transform = 1;
                    transform_stamped = tf_buffer.lookupTransform(mapData.header.frame_id, base_frame, ros::Time(0));
                }
                catch (tf2::ExtrapolationException &ex) {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(0.5).sleep();
                }
            }

            x_new[0] = transform_stamped.transform.translation.x;
            x_new[1] = transform_stamped.transform.translation.y;
            V.push_back(x_new);
            line.points.clear();
            ROS_DEBUG("Reset local RRT");
        } else if (checking == 1) {
            V.push_back(x_new);

            p.x=x_new[0];
            p.y=x_new[1];
            p.z=0.0;
            line.points.push_back(p);
            p.x=x_nearest[0];
            p.y=x_nearest[1];
            p.z=0.0;
            line.points.push_back(p);
        } else {
            // x_new is in obstacle region
        }

        rviz_pub.publish(line);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
