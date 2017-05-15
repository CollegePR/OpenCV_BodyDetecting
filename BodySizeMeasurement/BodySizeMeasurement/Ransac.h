#pragma once

#include <iostream>
#include <opencv/cv.hpp>
#include "LineData.h"

using namespace std;
using namespace cv;

bool find_in_samples(new_point *samples, int no_samples, new_point *data);
void get_samples(new_point *samples, int no_samples, new_point *data, int no_data);
int compute_model_parameter(new_point samples[], int no_samples, new_line &model);
double compute_distance(new_line &nline, new_point &x);
double model_verification(new_point *inliers, int *no_inliers, new_line &estimated_model, new_point *data, int no_data, double distance_threshold);
double ransac_line_fitting(new_point *data, int no_data, new_line &model, double distance_threshold);
new_line Ransac(Mat &src_image, vector<Point> contours, int s, int f, Scalar sc);