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
	OptionalJacobian<2, 6> Dpose(Matrix(2,6).setZero());
    OptionalJacobian<2, FixedDimension<Point3>::value> Dpoint(Matrix(2,3).setZero());

	Point2 p = camera.project(point, Dpose, Dpoint, boost::none);

    if(Dpose)   cout << "Has Dpose: \n" << Dpose->matrix() << endl;
    if(Dpoint)  cout << "Has Dpoint: \n" << Dpoint->matrix() << endl;

	cout << "p: \n" << p << endl;
    return 0;
}
