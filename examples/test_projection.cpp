#include <gtsam/geometry/Point2.h>
//#include <gtsam/geometry/Line3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include "LinesProjectionFactor.h"
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/CalibratedCamera.h>

#include <vector>
//#include <thread>

//#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>

using namespace std;
using namespace gtsam;

//std::vector<gtsam::Point3> createPoints() {
//  std::vector<gtsam::Point3> points;
//  points.push_back(gtsam::Point3(10.0,10.0,10.0));
//  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
//  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
//  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
//  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
//  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
//  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
//  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));
//
//  return points;
//}

//LineSegment getLineFrom2Point(const Point3 & p1, const Point3 & p2)
//{
//    LineSegment ls; ls.resize(6);
//    ls << p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z();
//    return ls;
//}
//
//Polylines createLines(const std::vector<gtsam::Point3> & p){
//    Polylines poly;
//    poly.push_back(getLineFrom2Point(p[0], p[1]));
//    poly.push_back(getLineFrom2Point(p[2], p[3]));
//    poly.push_back(getLineFrom2Point(p[4], p[5]));
//    poly.push_back(getLineFrom2Point(p[6], p[7]));
//    //poly.push_back(getLineFrom2Point(p[8], p[9]));
//    return poly;
//};
//

std::vector<gtsam::Pose3> createPoses(
	const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0)),
	const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0, -M_PI / 4, 0), gtsam::Point3(sin(M_PI / 4) * 30, 0, 30 * (1 - sin(M_PI / 4)))),
	int steps = 8) {
	std::vector<gtsam::Pose3> poses;
	int i = 1;
	poses.push_back(init);
	for (; i < steps; ++i)
		poses.push_back(poses[i - 1].compose(delta));
	return poses;
}

Matrix26 _Dpose(const Point2& pn, double d) {
	const double u = pn.x(), v = pn.y();
	double uv = u * v, uu = u * u, vv = v * v;
	Matrix26 Dpn_pose;
	Dpn_pose <<
		uv, -1 - uu, v, -d, 0, d * u,
		1 + vv, -uv, -u, 0, -d, d * v;
	return Dpn_pose;
}

/* ************************************************************************* */
Matrix23 _Dpoint(const Point2& pn, double d, const Matrix3& Rt) {
	// optimized version of derivatives, see CalibratedCamera.nb
	const double u = pn.x(), v = pn.y();
	Matrix23 Dpn_point;
	Dpn_point << //
		Rt(0, 0) - u * Rt(2, 0), Rt(0, 1) - u * Rt(2, 1), Rt(0, 2) - u * Rt(2, 2), //
		Rt(1, 0) - v * Rt(2, 0), Rt(1, 1) - v * Rt(2, 1), Rt(1, 2) - v * Rt(2, 2);
	Dpn_point *= d;
	return Dpn_point;
}

int main(int argc, char* argv[]) {
	Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
	gtsam::Pose3 pose_init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(20, 0, 0));
	// rotation [0 0 -1] [1 0 0] [0 -1 0] 
	std::cout << pose_init.matrix() << std::endl;
	PinholeCamera<Cal3_S2> camera(pose_init, *K);

	Point3 pw = gtsam::Point3(10.0, 3.0, 0.0);
	//Point3 p3e = gtsam::Point3(10.0, 1.0, 0.0);
	//Point2 measurement = camera.project(p3);
	//std::cout << p3 << std::endl << measurement << std::endl;
	//std::cout << camera.project(p3) << std::endl << camera.project(p3e) << std::endl;

	gtsam::Matrix H1 = Matrix::Zero(2, 6);
	gtsam::Matrix H2 = Matrix::Zero(2, 3);

	Point2 m = camera.project(pw, H1, H2);
	//std::cout << "Default : \n" << m << std::endl 
	//	<< "H1:\n" << H1 << std::endl 
	//	<< "H2:\n" << H2 << std::endl;

	gtsam::Matrix Dpose = Matrix::Zero(2, 6);
	gtsam::Matrix Dpoint = Matrix::Zero(2, 3);

	gtsam::Matrix3 Rt = camera.pose().rotation().matrix().transpose();
	const Point3 q = camera.pose().transformTo(pw);
	const Point2 pn = camera.Project(q);


	const double d = 1.0 / q.z();
	Dpose = _Dpose(pn, d);
	Dpoint = _Dpoint(pn, d, Rt);

	Matrix2 Dpi_pn;
	Matrix25 Dcal;
	Dpi_pn << 320, 0, 0, 320;
	const Point2 pi = camera.calibration().uncalibrate(pn, Dcal, &Dpi_pn);
	Dpose = Dpi_pn * Dpose;
	Dpoint = Dpi_pn * Dpoint;

	std::cout << "Default: \n" << m << std::endl
		<< "MOD: \n" << pi << std::endl;
	std::cout << "H1: \n" << H1 << std::endl
		<< "DPose: \n" << Dpose << std::endl;
	std::cout << "H1: \n" << H2 << std::endl
		<< "DPose: \n" << Dpoint << std::endl;


	//const Point2 pi = calibration().uncalibrate(pn, Dcal, Dpose || Dpoint ? &Dpi_pn : 0);


	//getch();
}
