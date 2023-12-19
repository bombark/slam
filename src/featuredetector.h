// ============================================================================
//  Header
// ============================================================================

#pragma once 

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <unistd.h>
#include <ctime>
#include <sys/time.h>

#include "houghtransform.h"

typedef struct Feature {
  double x, y;
  double dist, bear;
} Feature;

// ============================================================================
//  FeatureDetector
// ============================================================================

class FeatureDetector
{
public:
  double NO_COMPASS = 100.0;
  
  static const int MAX_DIST = HoughTransform::MAX_DIST;
  static const int MIN_DIST = 1000 * 1000;
  static const int MIN_POINTS = 3;  //minimum number of points needed for a line segment
  static const int POINT_DIST = 600; //distance between points on same line segment (mm)
  
  double CORNER_THETA = 22.0 * 3.141592654/180.0;   //min angle between segments making a corner
  static const int CORNER_DIST = 90000; //squared distance between segment and feature (mm)
  
  double COMPASS_THRESH = 10 * 3.141592654/180.0;   //maximum angle between parallel lines
  
  // FeatureDetector(ArSick* sick);
  FeatureDetector();
  ~FeatureDetector();
  
  //get the features for the most recent scan
  int getFeatures(std::vector<Feature>* featVec, double* structCompass, double curPhi, std::vector<float>& readings);
  
private:
  struct lineSegment{
    double radius;
    double theta;
    double startX, startY;
    double endX, endY;
    int numPoints;
    struct lineSegment* next;
  };
  
  struct compassgroup{
    double theta;
    double weight;
  };

  double COMPASS_OFFSET = 100.0;
  long Last_Time;

  HoughTransform* hough;
  
  int fitLineSegments(std::vector<float>& readings, 
                      std::vector<struct houghLine> *lines, 
                      std::vector<struct lineSegment> *segments);
    
  double getStructCompass(std::vector<struct houghLine> *lines, double curPhi);
  int extractCorners(std::vector<Feature> *featVec, std::vector<struct lineSegment> *segments);
};

