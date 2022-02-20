#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace gtsam;

std::vector<gtsam::Point3> createPoints() {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses(
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,-M_PI/2), gtsam::Point3(30, 0, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,-M_PI/4,0), gtsam::Point3(sin(M_PI/4)*30, 0, 30*(1-sin(M_PI/4)))),
            int steps = 8) {

  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for(; i < steps; ++i) {
    poses.push_back(poses[i-1].compose(delta));
  }

  return poses;
}


/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
//  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
  // Define the camera observation noise model
  auto measurementNoise= noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v
  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();
  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();
  // Create a factor graph
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
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
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
    Point3 corrupted_point = points[j] + Point3(-0.25, 0.20, 0.15);
    initialEstimate.insert<Point3>(Symbol('l', j), corrupted_point);
  }
//  initialEstimate.print("Initial Estimates:\n");

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
//  result.print("Final results:\n");
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

  return 0;
}
/* ************************************************************************* */


