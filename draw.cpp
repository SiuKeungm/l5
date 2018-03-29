#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <thread>
using namespace std;

// path to trajectory file
string trajectory_file = "./compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> );

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e,poses_before;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts1,pts2;

    /// implement pose reading code
    // start your code here (5~10 lines)
    fstream fin(trajectory_file);
    if(!fin)
    {
        cerr << "请在有trajectory.txt的目录下运行此程序" << endl;
        return 1;
    }
    double time_e , tx_e, ty_e, tz_e, qx_e, qy_e, qz_e, qw_e;
    double time_g , tx_g, ty_g, tz_g, qx_g, qy_g, qz_g, qw_g;
    while(!fin.eof())
    {
        fin >> time_e >> tx_e >> ty_e >> tz_e >> qx_e >> qy_e >> qz_e >> qw_e;
        fin >> time_g >> tx_g >> ty_g >> tz_g >> qx_g >> qy_g >> qz_g >> qw_g;
        Eigen::Quaterniond qe(qw_e , qx_e , qy_e , qz_e);
        //q.normalize();
        Eigen::Vector3d te(tx_e , ty_e, tz_e);
        Sophus::SE3 pose_e(qe,te);
        pts1.push_back(te);
        poses_e.push_back(pose_e);

        Eigen::Quaterniond qg(qw_g , qx_g , qy_g , qz_g);
        Eigen::Vector3d tg(tx_g , ty_g, tz_g);
        pts2.push_back(tg);
        Sophus::SE3 pose_g(qg,tg);
        poses_g.push_back(pose_g);
    }
    fin.close();

    //thread thread1(DrawTrajectory,poses_e,poses_g);
    poses_before = poses_e;


    Eigen::Vector3d p1(0,0,0),p2(0,0,0);
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];

    }
    p1 /=N;  p2 /=N;
    //remove the center
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> q1(N),q2(N); //vector遍历前要预先分配内存
    for(int i = 0;i < N ; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i =0;i < N;i++)
    {
        W += q2[i] * q1[i].transpose();
    }
    //SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W , Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R_ = U*V.transpose();
    Eigen::Vector3d t_ = p2 - R_*p1;

    Sophus::SE3 T_ge(R_ ,t_);

    for(auto& pose:poses_e){
        pose = T_ge*pose;
    }


    // end your code here

    // draw trajectory in pangolin
    //DrawTrajectory(poses_e);
   // thread1.detach();
   DrawTrajectory(poses_g,poses_e,poses_before);

    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses3) {
    if (poses1.empty() || poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::CreatePanel("ui").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> before("ui.Before_align", false,true);
    pangolin::Var<bool> after("ui.After_align", false, true);


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        pangolin::glDrawAxis(3);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            //glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glColor3f(0.2 , 0.0f, 0);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        if(before) {
            for (size_t i = 0; i < poses3.size() - 1; i++) {
                //glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
                glColor3f(0, 0, 1);
                glBegin(GL_LINES);
                auto p1 = poses3[i], p2 = poses3[i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }
        }
        if(after) {
            for (size_t i = 0; i < poses2.size() - 1; i++) {
                //glColor3f(1 - (float) i / poses2.size(), 0.0f, (float) i / poses2.size());
                glColor3f(1.0, 0.0, 0);
                glBegin(GL_LINES);
                auto p1 = poses2[i], p2 = poses2[i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                glEnd();
            }
        }
       // pangolin::SaveFramebuffer();

        pangolin::FinishFrame();
        usleep(3500);   // sleep 5 ms
    }

}