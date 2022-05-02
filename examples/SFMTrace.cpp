#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>
#include <conio.h>

using namespace std;
using namespace gtsam;

//Matrix kTestMatrix = (Matrix23() << 11,12,13,21,22,23).finished(); //Apple M1 build Error

int main(int argc, char* argv[]) {
	
	gtsam::Pose3 initPose = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0));

	//cout << "Pose: \n" << initPose.matrix() << endl;
	Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
	PinholeCamera<Cal3_S2> camera(initPose, *K);
	
	gtsam::Point3 point(10.0, 10.0, 10.0);

	Matrix D1 = Matrix::Zero(3, 6);
	Matrix D2 = Matrix::Zero(2, 4);
	Matrix d1 = Matrix::Zero(2, 6);
	Matrix d2 = Matrix::Zero(2, 3);

	Point2 p = camera.project(point, d1, d2, boost::none);
	D1.block(0, 0, 2, 6) = d1;
	D2.block(0, 0, 2, 3) = d2;
	cout << "Has Dpose: \n" << D1 << endl;

	//cout << "p: \n" << p << endl;
	//char k;cin >> k;

	//_getch();
    return 0;
}
