#pragma once

struct ArSensorReading {
  double getRange(){return 0;}
  double getLocalX(){return 0;}
  double getLocalY(){return 0;}
};

struct ArTime {
    unsigned long long mySec;
    unsigned long long myMSec;

    bool isAt(ArTime time){return false;}
};