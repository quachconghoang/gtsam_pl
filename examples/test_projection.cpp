#include <gtsam/geometry/Point2.h>
//#include <gtsam/geometry/Line3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include "LinesProjectionFactor.h"

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
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,-M_PI/2), gtsam::Point3(30, 0, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,-M_PI/4,0), gtsam::Point3(sin(M_PI/4)*30, 0, 30*(1-sin(M_PI/4)))),
            int steps = 8) {
	std::vector<gtsam::Pose3> poses;
	int i = 1;
	poses.push_back(init);
	for (; i < steps; ++i)
		poses.push_back(poses[i - 1].compose(delta));
	return poses;
}

int main(int argc, char* argv[]) {
	Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
	gtsam::Pose3 pose_init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(20, 0, 0));
	// rotation [0 0 -1] [1 0 0] [0 -1 0] 
	std::cout << pose_init.matrix() << std::endl;
	PinholeCamera<Cal3_S2> camera(pose_init, *K);

	Point3 p3 = gtsam::Point3(10.0, 0.0, 0.0);
	//Point3 p3e = gtsam::Point3(10.0, 1.0, 0.0);
	//Point2 measurement = camera.project(p3);
	//std::cout << p3 << std::endl << measurement << std::endl;
	//std::cout << camera.project(p3) << std::endl << camera.project(p3e) << std::endl;

	gtsam::Matrix H0;
	gtsam::Matrix H1 = Matrix::Zero(2, 6);
	gtsam::Matrix H2 = Matrix::Zero(2, 3);

	Point2 m = camera.project(p3, H1, H2);
	std::cout << "Default : \n" << m << std::endl 
		<< "H1:\n" << H1 << std::endl 
		<< "H2:\n" << H2;


}

/*  
int main(int argc, char* argv[]) {
    // Define the camera calibration parameters
    Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
    auto measurementNoise= noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v
    vector<Point3> points = createPoints();
    Polylines poly = createLines(points);
    vector<Pose3> poses = createPoses();
    NonlinearFactorGraph graph;
    // Add a prior on pose x1. This indirectly specifies where the origin is.
    auto poseNoise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.addPrior(Symbol('x', 0), poses[0], poseNoise);  // add directly to graph

    // Simulated measurements from each camera pose, adding them to the factor graph
    for (size_t i = 0; i < poses.size(); ++i) {
        PinholeCamera<Cal3_S2> camera(poses[i], *K);
        for (size_t j = 0; j < points.size(); ++j) {
            Point2 measurement = camera.project(points[j]);
			//graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
			//	measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
            graph.emplace_shared<LinesProjectionFactor<Pose3, Point3, Cal3_S2> >(
                    measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
        }
    }

    // Because the structure-from-motion problem has a scale ambiguity, the
    // problem is still under-constrained Here we add a prior on the position of
    // the first landmark. This fixes the scale by indicating the distance between
    // the first camera and the first landmark. All other landmark positions are
    // interpreted using this scale.
    auto pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.addPrior(Symbol('l', 0), points[0],pointNoise);  // add directly to graph
//  graph.print("Factor Graph:\n");

    // Create the data structure to hold the initial estimate to the solution
    // Intentionally initialize the variables off from the ground truth
    Values initialEstimate;
    for (size_t i = 0; i < poses.size(); ++i) {
        auto corrupted_pose = poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20)));
        initialEstimate.insert(Symbol('x', i), corrupted_pose);
    }
    for (size_t j = 0; j < points.size(); ++j) {
        Point3 corrupted_point = points[j] + 0.25*Eigen::VectorXd::Random(3);
        initialEstimate.insert<Point3>(Symbol('l', j), corrupted_point);
    }

//    for (size_t k = 0; k < poly.size(); ++k) {
//        Eigen::VectorXd err = 0.25 * Eigen::VectorXd::Random(6);
//        LineSegment corrupted_line = poly[k] + err;
//        cout << corrupted_line.transpose() << std::endl;
//        initialEstimate.insert<LineSegment>(Symbol('l', k), corrupted_line);
//    }

//  initialEstimate.print("Initial Estimates:\n");

    //Optimize the graph and print results
    Values result = DoglegOptimizer(graph, initialEstimate).optimize();
//  result.print("Final results:\n");
    cout << "initial error = " << graph.error(initialEstimate) << endl;
    cout << "final error = " << graph.error(result) << endl;

    return 0;
}
*/


