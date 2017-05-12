#pragma once

#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int ccw(Point p1, Point p2, Point p3, int C);
bool compx(Point &i, Point &j);
bool compy(Point &i, Point &j);
bool isEqualPoint(Point p1, Point p2, int C);
vector<Point> ContourOptimization(vector<Point> &points);
void get4SidePoints(vector<Point> &points, vector<Point> &left, vector<Point> &top, vector<Point> &right, vector<Point> &bottom);
void GridRecovery(Mat &src_image, Mat &dst_image, vector<vector<Point>> &result_points);