#pragma once

#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

bool isEqualPoint(Point p1, Point p2, int C);
vector<Point> ContourOptimization(vector<Point> &points);
void get4SidePoints(vector<Point> &points, vector<Point> &left, vector<Point> &top, vector<Point> &right, vector<Point> &bottom);
void GridRecovery(Mat &src_image, Mat &dst_image);