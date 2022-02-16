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
    OptionalJacobian<2, 6> Dpose(Matrix(2,6).setZero());
    OptionalJacobian<2, FixedDimension<Point3>::value> Dpoint(Matrix(2,3).setZero());

//    OptionalJacobian<2, 5> Dcal;
    Matrix testMat(2,5);
    testMat.setZero();
    cout << testMat << endl;
    cout << kTestMatrix << endl;
//    Eigen::Matrix<double, 2, 3> xxx;
//    std::cout << xxx;
//    Dpose.
//    Dpose->setZero();
    std::cout << Dpose.operator bool();

    if(Dpose)   cout << "Has Dpose: \n" << Dpose->matrix() << endl;
    if(Dpoint)  cout << "Has Dpoint: \n" << Dpoint->matrix() << endl;
    //{{2, 3, 4},{5, 6, 7}};

    return 0;
}
