#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char **argv)
{
	Eigen::Matrix3d R;
	R << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	Eigen::Vector3d t(1,2,3);
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.rotate(R);
	T.pretranslate(t);
	cout <<"R:\n"<< R << endl;
	cout <<"t:\n"<< t << endl;
	cout <<"T:\n"<< T.matrix() << endl;
}

