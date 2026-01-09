#include "LKTracker.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace ORB_SLAM3;
LKTracker* tracker= nullptr;

void img_callback(const sensor_msgs::ImageConstPtr &left, const sensor_msgs::ImageConstPtr &right)
{
	cv_bridge::CvImagePtr cv_ptr_l;
	cv_bridge::CvImagePtr cv_ptr_r;
	try {
		cv_ptr_l = cv_bridge::toCvCopy(left, "bgr8");
		cv_ptr_r = cv_bridge::toCvCopy(right, "bgr8");
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	double cur_img_time = left->header.stamp.toSec();

	cv::Mat gray_img_l, gray_img_r;
	cv::cvtColor(cv_ptr_l->image, gray_img_l, cv::COLOR_BGR2GRAY);
	cv::cvtColor(cv_ptr_r->image, gray_img_r, cv::COLOR_BGR2GRAY);
	if(tracker){
		tracker->trackImage(cur_img_time,gray_img_l,gray_img_r);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "LKTrcker");
	ros::NodeHandle n("~");
	image_transport::ImageTransport it(n);
	tracker = new LKTracker();



	image_transport::TransportHints hints("compressed");
	image_transport::SubscriberFilter img_l_sub(it, "/camera/left/image_dehazed", 50, hints);
	image_transport::SubscriberFilter img_r_sub(it, "/camera/right/image_dehazed", 50, hints);
	typedef sync_policies::ApproximateTime<Image, Image> Img_sync;
	Synchronizer<Img_sync> img_sync(Img_sync(50), img_l_sub, img_r_sub);
	img_sync.registerCallback(boost::bind(&img_callback, _1, _2));
	ros::spin();
	return 0;
}