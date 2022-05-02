/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample.cpp
 * @brief   A structure-from-motion problem on a simulated dataset
 * @author  Duy-Nguyen Ta
 */
#include "SFMdata.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/LinesProjectionFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>
#include <random>
#include <conio.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(320.0, 320.0, 0.0, 320.0, 240.0));
//  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
  // Define the camera observation noise model
  auto measurementNoise= noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v
  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();
  vector<Vector4> w_points = createWeightedPoints();

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

	  int line_num = j / 2;
	  Point2 p0 = camera.project(points[line_num * 2]);
	  Point2 p1 = camera.project(points[line_num * 2 + 1]);

	  float semantic_measurement = 1.0f;
	  Vector4 line_measurement(p0[0], p0[1], p1[0], p1[1]);
      //graph.emplace_shared<LinesProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement, semantic_measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
	  //graph.emplace_shared<LinesProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
	  graph.emplace_shared<LinesProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement, line_measurement , measurementNoise, Symbol('x', i), Symbol('l', j), K);

    }
  }


  auto pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.addPrior(Symbol('l', 0), points[0],pointNoise);  // add directly to graph
//  graph.print("Factor Graph:\n");

  std::random_device rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  std::uniform_real_distribution<> rot_dis(-0.1, 0.1);

  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i) {
    //auto corrupted_pose = poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20)));
	  auto corrupted_pose = poses[i].compose(Pose3(Rot3::Rodrigues(rot_dis(gen), rot_dis(gen), rot_dis(gen)), Point3(dis(gen), dis(gen), dis(gen))));
    initialEstimate.insert(Symbol('x', i), corrupted_pose);
  }
  for (size_t j = 0; j < points.size(); ++j) {
    //Point3 corrupted_point = points[j] + Point3(-0.25, 0.20, 0.15);
	  Point3 corrupted_point = points[j] + Point3(dis(gen), dis(gen), dis(gen));
    initialEstimate.insert<Point3>(Symbol('l', j), corrupted_point);
  }
//  initialEstimate.print("Initial Estimates:\n");

  /* Optimize the graph and print results */
  DoglegParams params;
  params.setVerbosity("ERROR");
  //params.print();
  Values result = DoglegOptimizer(graph, initialEstimate, params).optimize();
//  result.print("Final results:\n");
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;

#if defined(WIN32)
  _getch();
#endif
#ifdef _MAC
#endif //_MAC
  
  return 0;
}
/* ************************************************************************* */

