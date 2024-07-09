#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <livox_ros_driver/CustomMsg.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sensor_msgs/PointField.h>

class PointCloudAccumulator {
public:
    PointCloudAccumulator() : nh_("~") {
        global_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    void run(int argc, char** argv) {
        if (argc < 3) {
            printHelp();
            return;
        }

        std::string bag_file = argv[1];
        std::string output_file = argv[2];
        std::string topic_name = (argc > 3) ? argv[3] : "";

        if (topic_name.empty()) {
            topic_name = promptForTopic(bag_file);
        }

        processBag(bag_file, topic_name);
        savePointCloud(output_file);
    }

private:
    ros::NodeHandle nh_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud_;

    void printHelp() {
        std::cout << "Usage: rosrun pointcloud_accumulator pointcloud_accumulator_node <bag_file> <output_file> [topic_name]" << std::endl;
        std::cout << "  bag_file: Path to the input .bag file" << std::endl;
        std::cout << "  output_file: Path for the output .pcd file" << std::endl;
        std::cout << "  topic_name: (Optional) Name of the topic to process" << std::endl;
    }

    std::string promptForTopic(const std::string& bag_file) {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        rosbag::View view(bag);
        std::vector<std::string> topics;

        for (const rosbag::ConnectionInfo* info : view.getConnections()) {
            if (info->datatype == "sensor_msgs/PointCloud2" || info->datatype == "livox_ros_driver/CustomMsg") {
                topics.push_back(info->topic);
            }
        }

        bag.close();

        if (topics.empty()) {
            throw std::runtime_error("No suitable topics found in the bag file.");
        }

        if (topics.size() == 1) {
            return topics[0];
        }

        std::cout << "Multiple suitable topics found. Please choose one:" << std::endl;
        for (size_t i = 0; i < topics.size(); ++i) {
            std::cout << i + 1 << ": " << topics[i] << std::endl;
        }

        size_t choice;
        while (true) {
            std::cout << "Enter your choice (1-" << topics.size() << "): ";
            std::cin >> choice;
            if (choice > 0 && choice <= topics.size()) {
                return topics[choice - 1];
            }
            std::cout << "Invalid choice. Please try again." << std::endl;
        }
    }

    void processBag(const std::string& bag_file, const std::string& topic_name) {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(topic_name);

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for (const rosbag::MessageInstance& msg : view) {
            if (msg.getDataType() == "sensor_msgs/PointCloud2") {
                sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                if (cloud_msg != nullptr) {
                    processPointCloud2(cloud_msg);
                }
            } else if (msg.getDataType() == "livox_ros_driver/CustomMsg") {
                livox_ros_driver::CustomMsg::ConstPtr livox_msg = msg.instantiate<livox_ros_driver::CustomMsg>();
                if (livox_msg != nullptr) {
                    processLivoxMsg(livox_msg);
                }
            }
        }

        bag.close();
    }


void processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // Create a new point cloud
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.points.resize(cloud_msg->width * cloud_msg->height);

    // Get field offsets and data types
    int x_idx = -1, y_idx = -1, z_idx = -1, reflectivity_idx = -1;
    for (size_t i = 0; i < cloud_msg->fields.size(); ++i) {
        if (cloud_msg->fields[i].name == "x") x_idx = i;
        if (cloud_msg->fields[i].name == "y") y_idx = i;
        if (cloud_msg->fields[i].name == "z") z_idx = i;
        if (cloud_msg->fields[i].name == "reflectivity") reflectivity_idx = i;
    }

    // Ensure we have x, y, z fields
    if (x_idx == -1 || y_idx == -1 || z_idx == -1 || reflectivity_idx == -1) {
        ROS_ERROR("Point cloud is missing required fields!");
        return;
    }

    // Copy data
    const uint8_t* ptr = cloud_msg->data.data();
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        // Copy x, y, z
        memcpy(&cloud.points[i].x, ptr + cloud_msg->fields[x_idx].offset, sizeof(float));
        memcpy(&cloud.points[i].y, ptr + cloud_msg->fields[y_idx].offset, sizeof(float));
        memcpy(&cloud.points[i].z, ptr + cloud_msg->fields[z_idx].offset, sizeof(float));

        // Handle reflectivity
        uint8_t reflectivity;
        memcpy(&reflectivity, ptr + cloud_msg->fields[reflectivity_idx].offset, sizeof(uint8_t));
        cloud.points[i].intensity = static_cast<float>(reflectivity);

        ptr += cloud_msg->point_step;
    }

    *global_cloud_ += cloud;

}


    void processLivoxMsg(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.resize(livox_msg->points.size());

        for (size_t i = 0; i < livox_msg->points.size(); ++i) {
            cloud[i].x = livox_msg->points[i].x;
            cloud[i].y = livox_msg->points[i].y;
            cloud[i].z = livox_msg->points[i].z;
            cloud[i].intensity = livox_msg->points[i].reflectivity;
        }

        *global_cloud_ += cloud;
    }

    void savePointCloud(const std::string& output_file) {
        if (pcl::io::savePCDFileBinary(output_file, *global_cloud_) == -1) {
            ROS_ERROR("Failed to save point cloud to file: %s", output_file.c_str());
        } else {
            ROS_INFO("Point cloud saved to file: %s", output_file.c_str());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_accumulator");
    PointCloudAccumulator accumulator;
    accumulator.run(argc, argv);
    return 0;
}
