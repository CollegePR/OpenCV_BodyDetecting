#pragma once

#include <iostream>
#include <opencv/cv.hpp>
#include "LineData.h"

using namespace std;
using namespace cv;

void Preprocess(Mat &src_image, Mat &dst_image);
void GrabCut(Mat &src_image, Mat &foreground_dst_image, Mat &background_dst_image);