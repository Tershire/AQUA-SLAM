#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <image_transport/image_transport.h>  // original
#include <image_transport/image_transport.hpp>
// #include <cv_bridge/cv_bridge.h>  // original
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

// ──────────────────────────────────────────────────────────────────────────────
// Grabber helper classes (ported from ros_stereo_DVL_tighly.cc)
// ──────────────────────────────────────────────────────────────────────────────

class ImuGrabber
{
public:
    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {
        unique_lock<mutex> lock(mBufMutex);
        imuBuf.push(imu_msg);
    }

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
    mutex mBufMutex;
};

class DVLGrabber
{
public:
    void GrabDVL(const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
        unique_lock<mutex> lock(mBufMutex);
        dvlBuf.push(msg);
    }

    queue<nav_msgs::msg::Odometry::SharedPtr> dvlBuf;
    mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, DVLGrabber *pDvlGb)
        : mpSLAM(pSLAM), mpImuGb(pImuGb), mpDvlGb(pDvlGb) {}

    void GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr &msg)
    {
        unique_lock<mutex> lock(mBufMutexLeft);
        imgLeftBuf.push(msg);
    }

    void GrabImageRight(const sensor_msgs::msg::Image::SharedPtr &msg)
    {
        unique_lock<mutex> lock(mBufMutexRight);
        imgRightBuf.push(msg);
    }

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr &img_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("aqua_slam"), "cv_bridge exception: %s", e.what());
        }
        return cv_ptr->image.clone();
    }

    // Main sync loop — runs in a dedicated thread
    void SyncWithImu()
    {
        const double maxTimeDiff = 0.1;
        while (true) {
            cv::Mat imLeft, imRight;
            double tImLeft = 0, tImRight = 0;

            if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty()) {
                tImLeft  = rclcpp::Time(imgLeftBuf.front()->header.stamp).seconds();
                tImRight = rclcpp::Time(imgRightBuf.front()->header.stamp).seconds();

                {
                    unique_lock<mutex> lock(mBufMutexRight);
                    while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
                        imgRightBuf.pop();
                        tImRight = rclcpp::Time(imgRightBuf.front()->header.stamp).seconds();
                    }
                }
                {
                    unique_lock<mutex> lock(mBufMutexLeft);
                    while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
                        imgLeftBuf.pop();
                        tImLeft = rclcpp::Time(imgLeftBuf.front()->header.stamp).seconds();
                    }
                }

                if (abs(tImLeft - tImRight) > maxTimeDiff)
                    continue;

                {
                    unique_lock<mutex> lock(mpImuGb->mBufMutex);
                    if (tImLeft > rclcpp::Time(mpImuGb->imuBuf.back()->header.stamp).seconds())
                        continue;
                }

                {
                    unique_lock<mutex> lock(mBufMutexLeft);
                    imLeft = GetImage(imgLeftBuf.front());
                    imgLeftBuf.pop();
                }
                {
                    unique_lock<mutex> lock(mBufMutexRight);
                    imRight = GetImage(imgRightBuf.front());
                    imgRightBuf.pop();
                }

                // Collect IMU + DVL measurements and merge into GyroDvlPoint vector
                vector<ORB_SLAM3::IMU::GyroDvlPoint> vGyroDVLMeas;
                vector<ORB_SLAM3::IMU::DvlPoint>     vDVLMeas;

                {
                    unique_lock<mutex> lock(mpImuGb->mBufMutex);
                    while (!mpImuGb->imuBuf.empty() &&
                           rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp).seconds() <= tImLeft) {
                        double t = rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp).seconds();
                        const auto &imu = mpImuGb->imuBuf.front();
                        float ax = imu->linear_acceleration.x;
                        float ay = imu->linear_acceleration.y;
                        float az = imu->linear_acceleration.z;
                        float wx = imu->angular_velocity.x;
                        float wy = imu->angular_velocity.y;
                        float wz = imu->angular_velocity.z;
                        vGyroDVLMeas.push_back(ORB_SLAM3::IMU::GyroDvlPoint(
                            ax, ay, az, wx, wy, wz, 0, 0, 0, 0, 0, 0, 0, t));
                        mpImuGb->imuBuf.pop();
                    }
                }

                {
                    unique_lock<mutex> lock(mpDvlGb->mBufMutex);
                    while (!mpDvlGb->dvlBuf.empty() &&
                           rclcpp::Time(mpDvlGb->dvlBuf.front()->header.stamp).seconds() <= tImLeft) {
                        double t = rclcpp::Time(mpDvlGb->dvlBuf.front()->header.stamp).seconds();
                        const auto &dvl = mpDvlGb->dvlBuf.front();
                        float vx = dvl->twist.twist.linear.x;
                        float vy = dvl->twist.twist.linear.y;
                        float vz = dvl->twist.twist.linear.z;
                        vDVLMeas.push_back(ORB_SLAM3::IMU::DvlPoint(vx, vy, vz, 0, 0, 0, 0, t));
                        vGyroDVLMeas.push_back(ORB_SLAM3::IMU::GyroDvlPoint(
                            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0, 0, 0, t));
                        mpDvlGb->dvlBuf.pop();
                    }
                }

                if (vDVLMeas.size() >= 2)
                    vDVLMeas.resize(1);

                if (vGyroDVLMeas.empty())
                    continue;

                sort(vGyroDVLMeas.begin(), vGyroDVLMeas.end(),
                     [](const ORB_SLAM3::IMU::GyroDvlPoint &a,
                        const ORB_SLAM3::IMU::GyroDvlPoint &b) { return a.t < b.t; });

                mpSLAM->TrackStereoGroDVL(imLeft, imRight, tImLeft, vGyroDVLMeas, !vDVLMeas.empty());
            }

            this_thread::sleep_for(chrono::milliseconds(1));
        }
    }

    queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf;
    mutex mBufMutexLeft, mBufMutexRight;

    ORB_SLAM3::System *mpSLAM;
    ImuGrabber        *mpImuGb;
    DVLGrabber        *mpDvlGb;
};

// ──────────────────────────────────────────────────────────────────────────────
// ROS 2 node
// ──────────────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aqua_slam_node");

    if (argc < 3) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: aqua_slam_node <path_to_vocabulary> <path_to_settings>");
        rclcpp::shutdown();
        return 1;
    }

    // Read topic names from settings YAML
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    string imu_topic   = fsSettings["ImuTopic"];
    string dvl_topic   = fsSettings["DvlTopic"];
    string img_l_topic = fsSettings["LeftImgTopic"];
    string img_r_topic = fsSettings["RightImgTopic"];

    RCLCPP_INFO(node->get_logger(), "IMU topic:   %s", imu_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "DVL topic:   %s", dvl_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "Left image:  %s", img_l_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "Right image: %s", img_r_topic.c_str());

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::DVL_STEREO, node, false);

    ImuGrabber   imugb;
    DVLGrabber   dvlgb;
    ImageGrabber igb(&SLAM, &imugb, &dvlgb);

    // IMU subscriber
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 100,
        [&imugb](const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
            imugb.GrabImu(msg);
        });

    // DVL subscriber
    auto dvl_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        dvl_topic, 100,
        [&dvlgb](const nav_msgs::msg::Odometry::SharedPtr msg) {
            dvlgb.GrabDVL(msg);
        });

    // Stereo image subscribers via image_transport (compressed transport)
    auto it = image_transport::create_subscription(
        node.get(), img_l_topic,
        [&igb](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            igb.GrabImageLeft(std::make_shared<sensor_msgs::msg::Image>(*msg));
        },
        "compressed");

    auto it_r = image_transport::create_subscription(
        node.get(), img_r_topic,
        [&igb](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            igb.GrabImageRight(std::make_shared<sensor_msgs::msg::Image>(*msg));
        },
        "compressed");

    // Launch sync thread
    thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    rclcpp::spin(node);

    rclcpp::shutdown();
    sync_thread.join();
    return 0;
}
