// ============================================================================
//  Header
// ============================================================================

#include <iostream>
#include <math.h>

#include "movementcontroller.h"

#define PI 3.14159265

void* main_move_control(void* args);
bool shouldTurnLeft(std::vector<float>& readings);
bool shouldStop(std::vector<float>& readings);

using namespace std;

// ============================================================================
//  MovementController
// ============================================================================

MovementController::MovementController() {
}

MovementController::~MovementController() {
}

void MovementController::start() { 
    int tc = pthread_create(&m_thread, nullptr, main_move_control, (void*)this);
    if (tc) std::cout << "Thread creation Error\n";
}

void MovementController::join() {
    pthread_join(m_thread, nullptr);
}

void* main_move_control(void* args) {
    MovementController* self = (MovementController*) args;

    self->m_robot.enable_lidar();
    self->m_robot.enable_motor();

    self->m_robot.setVel(1);

    self->m_robot.readLidar();
    self->m_robot.readLidar();

    while(1) {
        vector<float>& lidar = self->m_robot.readLidar();
        
        if ( shouldStop(lidar) ) {
            self->m_robot.setVel(0);
            break;
        }
    }

    pthread_exit(nullptr);
}

// ============================================================================
//  Private Functions
// ============================================================================

//Get average distance between 0 degrees and 40, if larger than thresh -> return true
bool shouldTurnLeft(std::vector<float>& readings) {
       
    if(readings.size() == 0) { 
        return false; 
    }

    float distToLeftWall = 0;
    float threshDistToLeftWall = 1300;
    bool shouldTurn = false;

    int count = 0;
    for (double theta=0.0;theta<40.0; theta+=5.0) {
        if(readings.size() != 0) {
            for (int i=0;i<5; i++) {
                count = count + 1;
                distToLeftWall += fabs(((readings)[theta+i])*sin((theta+i)*PI/180));
            }
        }
    }

    distToLeftWall /= count; // Average the distnace
    if (distToLeftWall > threshDistToLeftWall) {
        shouldTurn = true;
    }
    return shouldTurn; 
}


bool shouldStop(std::vector<float>& readings) {
    if(readings.size() == 0) { return false; }

    // Average distance in front left and front right window of scanner
    // if distance < thresh -> return true
    float distToFrontWallLeft = 0;
    float distToFrontWallRight = 0;
    bool shouldStop = false;

    //Average the distance
    //Sample every 2nd reading from 80 + 
    for(int i=0; i<=10; i+=2) {
        distToFrontWallLeft += fabs(((readings)[80+i])*sin((80+i)*PI/180));
    }
    distToFrontWallLeft /= 5.0;

    //Average the distance
    for(int i=0; i<=10; i+=2) {
        distToFrontWallRight += fabs(((readings)[90+i])*sin((90+i)*PI/180));
    }
    distToFrontWallRight /= 5.0; 

    // check
    printf("%f %f\n", distToFrontWallLeft, distToFrontWallRight);
    if (distToFrontWallLeft < 5 || distToFrontWallRight < 5) { 
        // 1 Meters, need to figure out what distance is good
        shouldStop = true;
    }

    return shouldStop; 
}