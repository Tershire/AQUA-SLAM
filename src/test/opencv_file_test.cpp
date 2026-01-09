#include <opencv2/core.hpp>
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

int main(int ac, char **av)
{
	FileStorage fs;
	fs.open("/home/da/project/ORB_SLAM3_DVL_tightly/data/underwater_orbslam3.yaml",FileStorage::READ);
	Mat T;
	FileNode n = fs["T_gyro_c"];
	T = n.mat();
	cout<<T<<endl;
	return 0;
}