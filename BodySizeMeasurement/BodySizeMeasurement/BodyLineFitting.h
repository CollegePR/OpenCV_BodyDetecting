#pragma once

#include <iostream>
#include <opencv/cv.hpp>
#include "LineData.h"

using namespace std;
using namespace cv;

void drawInterPoint(Mat image, new_line line1, new_line line2, Scalar sc = Scalar(255, 255, 255));
void BodyLineFitting(Mat &src_image, vector<Point> contour);
