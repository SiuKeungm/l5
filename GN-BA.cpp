//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    fstream fin1(p3d_file);
    while(!fin1.eof()){
        double p1 , p2 , p3;
        fin1 >> p1 >> p2 >> p3;
        Vector3d p3d_unit(p1 ,p2 , p3);
        p3d.push_back(p3d_unit);
    }
    fin1.close();
    fstream fin2(p2d_file);
    while(!fin2.eof()){
        double u , v;
        fin2 >> u >> v;
        Vector2d p2d_unit(u,v);
        p2d.push_back(p2d_unit);
    }
    fin2.close();

    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose
    //cout << T_esti.matrix() << endl;

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Vector3d rotate_p = T_esti*p3d[i];
            Vector3d projection = K*rotate_p;//SE3重载的乘法只有右乘ve3d，所以必须加括号
            double xi = rotate_p(0),yi = rotate_p(1),zi = rotate_p(2);
            Vector2d e = p2d[i] - Vector2d(projection(0)/projection(2),projection(1)/projection(2));
            cost += e.norm()*e.norm();

	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            double  z_2 = zi*zi;
            J(0,0) = -fx/zi;
            J(0,1) = 0;
            J(0,2) = fx*xi/z_2;
            J(0,3) = fx*xi*yi/z_2;
            J(0,4) =-fx*(1+xi*xi/z_2);
            J(0,5) = fx*yi/zi;
            J(1,0) = 0;
            J(1,1) = -fy/zi;
            J(1,2) = fy*yi/z_2;
            J(1,3) = fy*(1+yi*yi/z_2);
            J(1,4) = -fy*xi*yi/z_2;
            J(1,5) = -fy*xi/zi;

                    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE
        dx = H.ldlt().solve(b);

        // END YOUR CODE HERE

        if (std::isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE
        T_esti = Sophus::SE3::exp(dx)*T_esti;

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
