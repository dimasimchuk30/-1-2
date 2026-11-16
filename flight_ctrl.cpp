#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

struct Point3D {
    double x, y, z;
};

static Point3D current_pos = {0.0, 0.0, 0.0};
static bool odom_received = false;
static double total_distance = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double z = msg->pose.pose.position.z;

    if (odom_received) {
        const double dx = x - current_pos.x;
        const double dy = y - current_pos.y;
        const double dz = z - current_pos.z;
        total_distance += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    current_pos.x = x;
    current_pos.y = y;
    current_pos.z = z;
    odom_received = true;
}

double getDistance(const Point3D& p1, const Point3D& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

bool readPoints(ros::NodeHandle& nh_private,
                std::vector<Point3D>& targets,
                std::vector<std::string>& missing) {
    static const char* names[3] = {"p2", "p3", "p4"};
    static const char* axes[3] = {"x", "y", "z"};

    missing.clear();
    for (int i = 0; i < 3; ++i) {
        double v[3] = {0.0, 0.0, 0.0};
        for (int a = 0; a < 3; ++a) {
            const std::string key = std::string(names[i]) + "_" + axes[a];
            if (!nh_private.getParam(key, v[a])) {
                missing.push_back(key);
            }
        }
        targets[i + 1].x = v[0];
        targets[i + 1].y = v[1];
        targets[i + 1].z = v[2];
    }
    return missing.empty();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    double arrival_radius, hold_time, param_wait_timeout;
    std::string log_file;
    nh_private.param("arrival_radius", arrival_radius, 1.5);
    nh_private.param("hold_time", hold_time, 1.0);
    nh_private.param("param_wait_timeout", param_wait_timeout, 120.0);
    nh_private.param<std::string>("log_file", log_file, "flight_log.csv");

    ros::Subscriber odom_sub = nh.subscribe("/drone_0_visual_slam/odom", 10, odomCallback);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::Rate loop_rate(10); 

    const std::string node_name = ros::this_node::getName();  

    ROS_INFO("Mission node started. Coordinates of points 2, 3, 4 are read from rosparam, e.g.:");
    ROS_INFO("  rosparam set %s/p2_x -10.0   (and p2_y, p2_z, p3_x..p3_z, p4_x..p4_z)", node_name.c_str());
    ROS_INFO("Waiting for odometry...");
    while (ros::ok() && !odom_received) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    if (!ros::ok()) return 0;

    std::vector<Point3D> targets(5);
    std::vector<std::string> missing;
    int wait_ticks = 0;
    while (ros::ok() && !readPoints(nh_private, targets, missing)) {
        if (wait_ticks % 50 == 0) {  
            std::string list;
            for (size_t i = 0; i < missing.size(); ++i) {
                list += (i ? ", " : "") + missing[i];
            }
            ROS_WARN("Waiting for parameters: %s", list.c_str());
            ROS_WARN("Set them with: rosparam set %s/%s <value>", node_name.c_str(), missing[0].c_str());
        }
        if (wait_ticks * 0.1 > param_wait_timeout) {
            ROS_ERROR("Timeout: coordinates were not set within %.0f s. Shutting down.", param_wait_timeout);
            return 1;
        }
        ++wait_ticks;
        ros::spinOnce();
        loop_rate.sleep();
    }
    if (!ros::ok()) return 0;

    targets[0] = current_pos; 
    targets[4] = targets[0];   
    const std::vector<std::string> target_names = {"1", "2", "3", "4", "1 (Return)"};

    for (size_t i = 0; i < targets.size(); ++i) {
        ROS_INFO("Point %-10s: %.2f, %.2f, %.2f", target_names[i].c_str(),
                 targets[i].x, targets[i].y, targets[i].z);
    }

    int sub_wait = 0;
    while (ros::ok() && goal_pub.getNumSubscribers() == 0 && sub_wait < 100) {
        if (sub_wait == 0) ROS_WARN("No subscribers on the goal topic yet (is the planner running?)...");
        ++sub_wait;
        ros::spinOnce();
        loop_rate.sleep();
    }
    if (goal_pub.getNumSubscribers() == 0) {
        ROS_WARN("Still no subscribers on the goal topic; publishing anyway.");
    }

    std::ofstream csv_file(log_file.c_str());
    if (!csv_file.is_open()) {
        ROS_ERROR("Cannot open log file '%s' for writing.", log_file.c_str());
        return 1;
    }
    csv_file << std::fixed << std::setprecision(3);
    csv_file << "Time,X,Y,Z,Target_Index,Distance_to_Target\n";

    int current_target_idx = 1;
    bool target_published = false;
    bool in_arrival_zone = false;
    ros::Time arrival_start_time;
    const ros::Time mission_start_time = ros::Time::now();
    ros::Time segment_start_time = mission_start_time;
    std::vector<double> segment_times;

    while (ros::ok() && current_target_idx < 5) {
        ros::spinOnce();

        const Point3D current_target = targets[current_target_idx];
        const double dist_to_target = getDistance(current_pos, current_target);

        if (!target_published) {
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "world";
            goal_msg.pose.position.x = current_target.x;
            goal_msg.pose.position.y = current_target.y;
            goal_msg.pose.position.z = current_target.z;
            goal_msg.pose.orientation.w = 1.0;

            goal_pub.publish(goal_msg);
            target_published = true;
            in_arrival_zone = false;
            ROS_INFO("Flying to target %s (%.2f, %.2f, %.2f)",
                     target_names[current_target_idx].c_str(),
                     current_target.x, current_target.y, current_target.z);
        }

        ROS_INFO_THROTTLE(1.0, "Distance to %s: %.2f m",
                          target_names[current_target_idx].c_str(), dist_to_target);

        if (dist_to_target < arrival_radius) {
            if (!in_arrival_zone) {
                in_arrival_zone = true;
                arrival_start_time = ros::Time::now();
            } else if ((ros::Time::now() - arrival_start_time).toSec() >= hold_time) {
                const double seg_time = (ros::Time::now() - segment_start_time).toSec();
                segment_times.push_back(seg_time);
                ROS_INFO("Target %s reached in %.2f s",
                         target_names[current_target_idx].c_str(), seg_time);

                ++current_target_idx;
                target_published = false;
                segment_start_time = ros::Time::now();
            }
        } else {
            in_arrival_zone = false;
        }

        const double time_from_start = (ros::Time::now() - mission_start_time).toSec();
        csv_file << time_from_start << ","
                 << current_pos.x << "," << current_pos.y << "," << current_pos.z << ","
                 << current_target_idx << "," << dist_to_target << "\n";

        loop_rate.sleep();
    }

    csv_file.close();
    const double total_time = (ros::Time::now() - mission_start_time).toSec();
    if (current_target_idx < 5) {
        ROS_WARN("=== an ERROR occured (reached %zu of 4 targets) ===", segment_times.size());
    } else {
        ROS_INFO("=== Mission accomplished ===");
    }
    ROS_INFO("Mission timer: %.2f s", total_time);
    ROS_INFO("Path length:  %.2f m", total_distance);
    for (size_t i = 0; i < segment_times.size(); ++i) {
        ROS_INFO("Segment %s -> %s: %.2f s",
                 target_names[i].c_str(), target_names[i + 1].c_str(), segment_times[i]);
    }

    return 0;
}
