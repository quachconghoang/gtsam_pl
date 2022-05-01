#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

using namespace std;
using namespace gtsam;

Matrix kTestMatrix = (Matrix23() << 11,12,13,21,22,23).finished();

int main(int argc, char* argv[]) {
	
	gtsam::Pose3 initPose = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0));

	cout << "Pose: \n" << initPose.matrix() << endl;
	Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
	PinholeCamera<Cal3_S2> camera(initPose, *K);

	gtsam::Point3 point(10.0, 10.0, 10.0);
	

	Matrix36 Mpose_2(Matrix(3, 6).setZero());
	Matrix33 Mpoint_2(Matrix(3, 3).setZero());
	//Matrix26 Mpose = Mpose_2.block(0, 0, 2, 6);
	//Matrix23 Mpoint = Mpoint_2.block(0, 0, 2, 3);

	
	OptionalJacobian<3, 6> D1(Mpose_2);
	OptionalJacobian<3, 3> D2(Mpoint_2);
	
	
	OptionalJacobian<2, 6> Dpose(Matrix(2,6).setZero());
	OptionalJacobian<2, 3> Dpoint(Matrix(2,3).setZero());


	Point2 p = camera.project(point, Dpose, Dpoint, boost::none);

    if(Dpose)   cout << "Has Dpose: \n" << Dpose->matrix() << endl;
    if(Dpoint)  cout << "Has Dpoint: \n" << Dpoint->matrix() << endl;

	//D1->matrix().resize(3, 3);

	cout << D1->matrix() << endl;
	cout << D2->matrix() << endl;

	//if (Dpose_2)   cout << "Has Dpose: \n" << Dpose_2->matrix() << endl;
	//if (Dpoint_2)  cout << "Has Dpoint: \n" << Dpoint_2->matrix() << endl;

	//cout << "p: \n" << p << endl;
	char k;cin >> k;
    return 0;
}
