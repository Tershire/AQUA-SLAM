/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ds_sensor_msgs/Dvl.h>


#include<opencv2/core/core.hpp>

#include <Eigen/Core>

#include <System.h>
#include <ImuTypes.h>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
using namespace std;
namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

void init_logging()
{
//	logging::add_file_log("debug_log.log");
	logging::add_file_log
		(
			keywords::file_name = "log/log_%N.log",
			keywords::rotation_size = 10 * 1024 * 1024,
			keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
			keywords::format = "%Message%",
			keywords::auto_flush = true
		);
//	logging::add_console_log();
	logging::core::get()->set_filter(
		logging::trivial::severity >= logging::trivial::info
	);
}

class ImuGrabber
{
public:
	ImuGrabber()
	{
		mT_e0_ei = Eigen::Isometry3d::Identity();
		t_last = 0;
	};
	void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
	void GrabImu2(const nav_msgs::OdometryConstPtr &odo_msg);

	queue<sensor_msgs::ImuConstPtr> imuBuf;
	Eigen::Isometry3d mT_e0_ei;
	double t_last;
	std::mutex mBufMutex;
};

class DVLGrabber
{
public:
	DVLGrabber()
	{};
	void GrabDVL(const ds_sensor_msgs::DvlConstPtr &odo);

//	queue<nav_msgs::OdometryConstPtr> dvlBuf;
	queue<ds_sensor_msgs::DvlConstPtr> dvlBuf;
	std::mutex mBufMutex;

};

class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, DVLGrabber *pDvlGb, const bool bRect, const bool bClahe)
		: mpSLAM(pSLAM), mpImuGb(pImuGb), mpDvlGb(pDvlGb), do_rectify(bRect), mbClahe(bClahe)
	{}

	void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);
	void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);
	cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
	void SyncWithImu();

	queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
	std::mutex mBufMutexLeft, mBufMutexRight;

	ORB_SLAM3::System *mpSLAM;
	ImuGrabber *mpImuGb;
	DVLGrabber *mpDvlGb;

	const bool do_rectify;
	cv::Mat M1l, M2l, M1r, M2r;

	const bool mbClahe;
	cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv)
{
	init_logging();
	ros::init(argc, argv, "Stereo_Inertial");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	bool bEqual = false;
	if (argc < 4 || argc > 5) {
		cerr << endl
			 << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]"
			 << endl;
		ros::shutdown();
		return 1;
	}

	std::string sbRect(argv[3]);
	if (argc == 5) {
		std::string sbEqual(argv[4]);
		if (sbEqual == "true") {
			bEqual = true;
		}
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::DVL_STEREO, true);

	ImuGrabber imugb;
	DVLGrabber dvlgb;
	ImageGrabber igb(&SLAM, &imugb, &dvlgb, sbRect == "false", bEqual);



	// Maximum delay, 5 seconds
	// flowave
//	ros::Subscriber sub_imu = n.subscribe("/BlueRov2/imu/data/ENU", 100, &ImuGrabber::GrabImu, &imugb);
////	ros::Subscriber sub_imu2 = n.subscribe("/odometry/filtered", 100, &ImuGrabber::GrabImu2, &imugb);
//	ros::Subscriber sub_dvl = n.subscribe("/BlueRov2/DVL", 100, &DVLGrabber::GrabDVL, &dvlgb);
//	ros::Subscriber sub_img_left = n.subscribe("/suv3d/left/rgb_rect", 100, &ImageGrabber::GrabImageLeft, &igb);
//	ros::Subscriber sub_img_right = n.subscribe("/suv3d/right/rgb_rect", 100, &ImageGrabber::GrabImageRight, &igb);

	//rovco
	ros::Subscriber sub_imu = n.subscribe("/kvh_1750_imu/imu", 100, &ImuGrabber::GrabImu, &imugb);
//	ros::Subscriber sub_imu2 = n.subscribe("/odometry/filtered", 100, &ImuGrabber::GrabImu2, &imugb);
	ros::Subscriber sub_dvl = n.subscribe("/devices/dvl/dvl", 100, &DVLGrabber::GrabDVL, &dvlgb);
	ros::Subscriber sub_img_left = n.subscribe("/suv3d/left/rgb_rect", 100, &ImageGrabber::GrabImageLeft, &igb);
	ros::Subscriber sub_img_right = n.subscribe("/suv3d/right/rgb_rect", 100, &ImageGrabber::GrabImageRight, &igb);

	std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

	ros::spin();

	return 0;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
	BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9) << "left recieved!, time: " << img_msg->header.stamp.toSec();
//	cout<<"left recieved!, time: "<<img_msg->header.stamp.toNSec()<<endl;
	mBufMutexLeft.lock();
//	if (!imgLeftBuf.empty())
//	{
//		imgLeftBuf.pop();
//	}
	imgLeftBuf.push(img_msg);
	mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
	BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9) << "right recieved!, time: " << img_msg->header.stamp.toSec();
//	cout<<"right recieved!, time: "<<img_msg->header.stamp.toNSec()<<endl;
	mBufMutexRight.lock();
//	if (!imgRightBuf.empty())
//	{
//		imgRightBuf.pop();
//	}
	imgRightBuf.push(img_msg);
	mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	if (cv_ptr->image.type() == 0) {
		return cv_ptr->image.clone();
	}
	else {
		std::cout << "Error type" << std::endl;
		return cv_ptr->image.clone();
	}
}

void ImageGrabber::SyncWithImu()
{
	const double maxTimeDiff = 0.1;
	while (1) {
		cv::Mat imLeft, imRight;
		double tImLeft = 0, tImRight = 0;
		if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty()) {
//			mBufMutexLeft.lock();
			tImLeft = imgLeftBuf.front()->header.stamp.toSec();
//			mBufMutexLeft.unlock();
//			mBufMutexRight.lock();
			tImRight = imgRightBuf.front()->header.stamp.toSec();
//			mBufMutexRight.unlock();

			this->mBufMutexRight.lock();
			// right timestamp is too smaller than left
			while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
				cout << "pop right" << endl;
				imgRightBuf.pop();
				tImRight = imgRightBuf.front()->header.stamp.toSec();
			}
			this->mBufMutexRight.unlock();

			this->mBufMutexLeft.lock();
			// left timestamp is too smaller than right
			while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
				cout << "pop left" << endl;
				imgLeftBuf.pop();
				tImLeft = imgLeftBuf.front()->header.stamp.toSec();
			}
			this->mBufMutexLeft.unlock();

			// cannot find image with good timestamp diff, waiting for the following image
			if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
//				std::cout << "big time difference" << std::endl;
				continue;
			}

			// keep saving IMU data until the timestamp of lasted IMU data > the timestamp of lasted Image
			if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec()) {
//				cout<<"continue waiting for IMU Meas"<<endl;
				continue;
			}


			this->mBufMutexLeft.lock();
			imLeft = GetImage(imgLeftBuf.front());
			imgLeftBuf.pop();
			this->mBufMutexLeft.unlock();

			this->mBufMutexRight.lock();
			imRight = GetImage(imgRightBuf.front());
			imgRightBuf.pop();
			this->mBufMutexRight.unlock();

			vector<ORB_SLAM3::IMU::ImuPoint> vImuMeas;
			vector<ORB_SLAM3::IMU::DvlPoint> vDVLMeas;
			mpImuGb->mBufMutex.lock();
			if (!mpImuGb->imuBuf.empty()) {
				// Load imu measurements from buffer
				vImuMeas.clear();
				// save all IMU data between last Frame and current Frame to current Frame
				while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft) {
					double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
					cv::Point3f acc(0, 0, 0);
					cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
									mpImuGb->imuBuf.front()->angular_velocity.y,
									mpImuGb->imuBuf.front()->angular_velocity.z);
					vImuMeas.push_back(ORB_SLAM3::IMU::ImuPoint(acc, gyr, t));
					mpImuGb->imuBuf.pop();
				}
			}
			mpImuGb->mBufMutex.unlock();
			mpDvlGb->mBufMutex.lock();
			if (!mpDvlGb->dvlBuf.empty()) {
				vDVLMeas.clear();
				while (!mpDvlGb->dvlBuf.empty() && mpDvlGb->dvlBuf.front()->header.stamp.toSec() <= tImLeft) {
					double t = mpDvlGb->dvlBuf.front()->header.stamp.toSec();
					cv::Point3f vel(mpDvlGb->dvlBuf.front()->velocity.x,
									mpDvlGb->dvlBuf.front()->velocity.y,
									mpDvlGb->dvlBuf.front()->velocity.z);
					vDVLMeas.push_back(ORB_SLAM3::IMU::DvlPoint(vel, t));
					mpDvlGb->dvlBuf.pop();
				}
			}
			mpDvlGb->mBufMutex.unlock();

			if (vDVLMeas.size() >= 2) {
				BOOST_LOG_TRIVIAL(info) << "there are two DVL measurement between Frames!";
				vDVLMeas.clear();
				continue;
			}

			if (vImuMeas.empty()) {
				BOOST_LOG_TRIVIAL(info) << "no IMU measurement between Frames!";
				vImuMeas.clear();
				continue;
			}

			if (mbClahe) {
				mClahe->apply(imLeft, imLeft);
				mClahe->apply(imRight, imRight);
			}


			//virtual IMU Meas for DVL
			// in that virtual Meas, we save velocity to acc
			if (!vDVLMeas.empty() && !vImuMeas.empty()) {
				if (vImuMeas[0].t > vDVLMeas[0].t) {
					vImuMeas.insert(vImuMeas.begin(),
									ORB_SLAM3::IMU::ImuPoint(vDVLMeas[0].v, vImuMeas[0].w, vDVLMeas[0].t));
				}
				else if (vImuMeas[vImuMeas.size() - 1].t < vDVLMeas[0].t) {
					vImuMeas.push_back(ORB_SLAM3::IMU::ImuPoint(vDVLMeas[0].v,
																vImuMeas[vImuMeas.size() - 1].w,
																vDVLMeas[0].t));
				}
				else {
					for (int i = 0; i < vImuMeas.size(); i++) {
						if (vImuMeas[i].t > vDVLMeas[0].t) {
							vImuMeas.insert(vImuMeas.begin() + i,
											ORB_SLAM3::IMU::ImuPoint(vDVLMeas[0].v, vImuMeas[i].w, vDVLMeas[0].t));
							break;
						}
					}
				}
			}
			else {
				continue;
			}

			BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9)
									<< "Track Stereo!\n"
									<< "left image time: " << tImLeft;
			for (int i = 0; i < vImuMeas.size(); ++i) {
				BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9)
										<< "IMU[" << i << "] time: "
										<< vImuMeas[i].t << "\n"
										<< "a: " << vImuMeas[i].a << "\n"
										<< "w: " << vImuMeas[i].w << "\n";
			}
			for (int i = 0; i < vDVLMeas.size(); ++i) {
				BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9)
										<< "DVL[" << i << "] time: "
										<< vDVLMeas[i].t << "\n"
										<< vDVLMeas[i].v << "\n";
			}
			mpSLAM->TrackStereoGroDVL(imLeft, imRight, tImLeft, vImuMeas, !vDVLMeas.empty());

//			std::chrono::milliseconds tSleep(1);
//			std::this_thread::sleep_for(tSleep);
		}

	}
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
	BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9) << "IMU recieved! time:" << imu_msg->header.stamp.toSec();
//	cout<<"IMU recieved! time:"<<imu_msg->header.stamp.toNSec()<<endl;
	mBufMutex.lock();
//	Eigen::AngleAxisd r_x(1,
//					   Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z));
//	Eigen::AngleAxisd r_x(1,
//						  Eigen::Vector3d(imu_msg->angular_velocity.z,
//										  -imu_msg->angular_velocity.x,
//										  -imu_msg->angular_velocity.y));

	sensor_msgs::ImuPtr p_new_msg(new sensor_msgs::Imu());
	p_new_msg->header = imu_msg->header;
	p_new_msg->angular_velocity = imu_msg->angular_velocity;
//	p_new_msg->angular_velocity.x = p_new_msg->angular_velocity.z;
//	p_new_msg->angular_velocity.y = -p_new_msg->angular_velocity.x;
//	p_new_msg->angular_velocity.z = -p_new_msg->angular_velocity.y;

//	Eigen::Quaterniond q_r(1, 0, 0, 0);
//	q_r = q_r * r_x;
//	imuBuf.push(imu_msg);
	imuBuf.push(p_new_msg);
	mBufMutex.unlock();
	return;
}
void ImuGrabber::GrabImu2(const nav_msgs::OdometryConstPtr &odo_msg)
{
//	BOOST_LOG_TRIVIAL(info) << "EKF IMU recieved! time:" << odo_msg->header.stamp.toNSec();
	sensor_msgs::ImuPtr imu(new sensor_msgs::Imu());
	imu->header = odo_msg->header;

	Eigen::Quaterniond q_r(odo_msg->pose.pose.orientation.w,
						   odo_msg->pose.pose.orientation.x,
						   odo_msg->pose.pose.orientation.y,
						   odo_msg->pose.pose.orientation.z);
	Eigen::Vector3d t(odo_msg->pose.pose.position.x, odo_msg->pose.pose.position.y, odo_msg->pose.pose.position.z);
	Eigen::Isometry3d T_e0_ej = Eigen::Isometry3d::Identity();
	T_e0_ej.rotate(q_r);
	T_e0_ej.pretranslate(t);
	Eigen::Isometry3d T_ei_ej = Eigen::Isometry3d::Identity();

	mBufMutex.lock();
	T_ei_ej = mT_e0_ei.inverse() * T_e0_ej;
	mT_e0_ei = T_e0_ej;
	double delta_t = odo_msg->header.stamp.toSec() - t_last;
	t_last = odo_msg->header.stamp.toSec();
	BOOST_LOG_TRIVIAL(info) << "delta_t:" << delta_t;
//	Eigen::AngleAxisd r_x(odo_msg->twist.twist.angular.x, Eigen::Vector3d::UnitX());
//	Eigen::AngleAxisd r_y(odo_msg->twist.twist.angular.y, Eigen::Vector3d::UnitY());
//	Eigen::AngleAxisd r_z(odo_msg->twist.twist.angular.z, Eigen::Vector3d::UnitZ());
//
//	Eigen::Quaterniond q_r(1,0,0,0);
//	q_r = q_r * r_x * r_y * r_z;
//	Eigen::AngleAxisd v_r(q_r);
//	Eigen::Vector3d eular_r= q_r.toRotationMatrix().eulerAngles(0,1,2);
//	BOOST_LOG_TRIVIAL(info)<<"reading data: "<<odo_msg->twist.twist.angular<<endl;
//	BOOST_LOG_TRIVIAL(info)<<"eular data: "<<eular_r<<endl;
//	BOOST_LOG_TRIVIAL(info)<<"axis: "<<v_r.axis()<<" angles: "<<v_r.angle()<<"vector: "<<v_r.angle()*v_r.axis()<<endl;
	Eigen::AngleAxisd v_r(T_ei_ej.rotation());
	Eigen::Vector3d vector_r = v_r.angle() / 0.0337441 * v_r.axis();

	imu->angular_velocity.x = vector_r.x();
	imu->angular_velocity.y = vector_r.y();
	imu->angular_velocity.z = vector_r.z();



//	cout<<"IMU recieved! time:"<<imu_msg->header.stamp.toNSec()<<endl;

	imuBuf.push(imu);
	mBufMutex.unlock();
	return;
}
void DVLGrabber::GrabDVL(const ds_sensor_msgs::DvlConstPtr &odo)
{
	BOOST_LOG_TRIVIAL(info) << fixed << setprecision(9) << "DVL recieved! time:" << odo->header.stamp.toSec();
//	cout<<"DVL recieved! time:"<<odo->header.stamp.toNSec()<<endl;
	mBufMutex.lock();
	dvlBuf.push(odo);
	mBufMutex.unlock();
	return;
}


