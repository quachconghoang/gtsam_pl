#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

using namespace std;
using namespace gtsam;

//Matrix kTestMatrix = (Matrix23() << 11,12,13,21,22,23).finished(); //Apple M1 build Error

int main(int argc, char* argv[]) {
	
	gtsam::Pose3 initPose = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0));

	//cout << "Pose: \n" << initPose.matrix() << endl;
	Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
	PinholeCamera<Cal3_S2> camera(initPose, *K);

	gtsam::Point3 point(10.0, 10.0, 10.0);
	Matrix d1 = Matrix(2, 6).setZero();
	Matrix d2 = Matrix(2, 3).setZero();
	Matrix D1 = Matrix(3, 6).setZero();
	Matrix D2 = Matrix(3, 3).setZero();

	OptionalJacobian<2, 6> Dpose(d1);
	OptionalJacobian<2, 3> Dpoint(d2);
	OptionalJacobian<3, 6> Dpose2(D1);
	OptionalJacobian<3, 3> Dpoint2(D2);

	Point2 p = camera.project(point, Dpose, Dpoint, boost::none);

    cout << "Has Dpose: \n" << Dpose->matrix() << endl;
    cout << "Has Dpoint: \n" << Dpoint->matrix() << endl;

	Dpose2->matrix().block(0, 0, 2, 6) = Dpose->matrix();
	Dpoint2->matrix().block(0, 0, 2, 3) = Dpoint->matrix();
	cout << "Has Dpose2: \n" << Dpose2->matrix() << endl;
	cout << "Has Dpoint2: \n" << Dpoint2->matrix() << endl;

	//if (Dpose_2)   cout << "Has Dpose: \n" << Dpose_2->matrix() << endl;
	//if (Dpoint_2)  cout << "Has Dpoint: \n" << Dpoint_2->matrix() << endl;

	//cout << "p: \n" << p << endl;
	char k;cin >> k;
    return 0;
}
