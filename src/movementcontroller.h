// ============================================================================
//  Header
// ============================================================================

#pragma once

#include <iostream>
#include <unistd.h>
#include <mutex>
#include <vector>

#include "robot.h"

// ============================================================================
//  MovementController
// ============================================================================

class MovementController {
public:
    MovementController();
    ~MovementController();
    void start();
    void join();

// private:
    Robot m_robot;
    pthread_t m_thread;
};


