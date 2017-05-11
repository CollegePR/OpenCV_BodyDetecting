#pragma once

#include <iostream>
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;

void getBodyContour(Mat &src_image, Mat &dst_image);
void BodyDetecting(Mat &src_image, Mat &dst_image);