#pragma once

#include <iostream>
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;

void BodyDetecting(Mat &src_image, vector<Point> &body_contour);