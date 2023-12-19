// ============================================================================
//  Header
// ============================================================================

#include <iostream>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <fstream>
#include <time.h>
#include <string>
#include <lt_api.h>
#include <vector>


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "kalmanfilter.h"
#include "movementcontroller.h"
#include "featuredetector.h"
#include "houghtransform.h"

using namespace std;
using namespace cv;

// ============================================================================
//  main
// ============================================================================

/*
int main(int argc, char **argv) {
    Robot robot;
    robot.enable_lidar();
    robot.enable_compass();
    robot.enable_encoder();

    KalmanFilter* ekf = new KalmanFilter();

    std::ofstream knownfeaturesFile;
    std::string knownfeaturesfileName = "./knownfeaturesRun.txt";
    std::string knownfeaturesPath = knownfeaturesfileName;
    std::remove(knownfeaturesPath.c_str());
    // covariance
    std::ofstream covFile;
    std::string covfileName = "./covRun.txt";
    std::string covPath = covfileName;
    std::remove(covPath.c_str());

    double dt = 0.001;

    FeatureDetector* f = new FeatureDetector();

    int time = 0;
    while(1) {
        double compass;
        std::vector<Feature> fvec;

        vector<float>& lidar = robot.readLidar();

        /*for (int i=0; i<lidar.size(); i++){
            cout << lidar[i] << " ";
        }
        cout << "\n";* /


        ekf->doPropagation(dt, covFile, knownfeaturesFile);
        f->getFeatures(&fvec, &compass, ekf->Phi, lidar);
        ekf->doUpdateCompass(compass, 0.0005);

        for (int i=0; i<fvec.size(); i++) {
            //Initialize matrices
            Eigen::MatrixXd z_chunk(2,1);
            Eigen::MatrixXd R(2,2);
            Eigen::MatrixXd R_chunk(2,2);
            Eigen::MatrixXd G(2,2);

            z_chunk << (fvec[i].x/1000.0), (fvec[i].y/1000.0);
            double fx = fvec[i].x/1000.0;
            double fy = fvec[i].y/1000.0;
            double dist = sqrt(fx*fx + fy*fy);
            double bearing = atan2(fy, fx);

            //Compute R
            R << 0.0025, 0, 0, 0.0001;
            G << cos(bearing), -dist * sin(bearing), sin(bearing), dist * cos(bearing);
            R_chunk = G * R * G.transpose();

            std::cout << "Update: ";
            ekf->doUpdate(z_chunk, R_chunk);
            std::cout << ekf->Num_Landmarks << std::endl;
            
            double newX = fx*cos(ekf->Phi) - fy*sin(ekf->Phi);
            double newY = fx*sin(ekf->Phi) + fy*cos(ekf->Phi);

            // cout << newX << " " << newY << std::endl;
        }

        cout << ekf->X << " " << ekf->Y << std::endl;

        /*if ( time > 4 ) {
            break;
        }* /
        // break;
        time += 1;
    }
    return 0;
}
*/



int main(int argc, char **argv)
{
    Robot robot;

    robot.enable_lidar();
    robot.enable_compass();
    robot.enable_encoder();

    Mat map(600, 600, CV_8UC3);

    
    float r_x=0, r_y=0, r_th=0;

    vector<float>& lidar = robot.readLidar();
    vector<float>& encoder = robot.readEncoder();
    float north = robot.readCompass();
    float last_left = encoder[0], last_right = encoder[1];

    while(1) {
        map.setTo(0);

        encoder = robot.readEncoder();
        float north = robot.readCompass();
        lidar = robot.readLidar();

        float world_r_x = 300+r_x;
        float world_r_y = 300-r_y;
        float r_th_rad = (r_th * M_PI) / 180.0;

        circle(map, Point(world_r_x, world_r_y), 7, Scalar(255,0,0), 1);
        line(map, Point(world_r_x, world_r_y), Point(world_r_x+10*cos(r_th_rad), world_r_y-10*sin(r_th_rad)), Scalar(255, 0, 0), 1, LINE_8);

        // Desenha os pontos do lidar
        float angulo = -M_PI/2.0;
        float diff_angulo = 3.141592 / lidar.size();

        for (int i=0; i<lidar.size(); i++) {
            float dist = lidar[i];

            uint16_t y = world_r_y - ( sin(r_th_rad-angulo) * dist * 5.0 );
            uint16_t x = world_r_x + ( cos(r_th_rad-angulo) * dist * 5.0 );

            if ( y < 600 && x < 600 ) {
                map.at<Vec3b>(y, x) = Vec3b(0,255,0);
            }
            
            angulo += diff_angulo;
        }


        float diff_left = encoder[0] - last_left;
        float diff_right = encoder[1] - last_right;
        last_left = encoder[0];
        last_right = encoder[1];

        float r_desl = (diff_right + diff_left) / 4.0;
        r_th += (diff_right - diff_left) / (2.0 * 0.047);
        r_x += r_desl * cos(r_th_rad);
        r_y += r_desl * sin(r_th_rad);
        if ( r_th > 360.0 ) {
            r_th = 0.0;
        } else if ( r_th < 0.0 ) {
            r_th = 360.0 + r_th;
        }

        printf("%.2f %.2f -> %.2f %.2f -> %.2f %f -> %f %f \n", encoder[0], encoder[1], diff_left, diff_right, r_th, r_desl, r_y, r_x  );

        /*
        vector<Vec4i> lines; // will hold the results of the detection
        HoughLinesP(map, lines, 0.5, CV_PI/180, 2, 1.5, 35.0 ); // runs the actual detection
        // Draw the lines
        printf("%d\n", lines.size());
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            line( map, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, LINE_AA);
        }*/

        imshow("map", map);
        waitKey(10);
    }

//    MovementController movement_ctrl;
//    movement_ctrl.start();
//    movement_ctrl.join();

    return 0;
}
