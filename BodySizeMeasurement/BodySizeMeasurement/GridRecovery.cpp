#include "GridRecovery.h"

int ccw(Point p1, Point p2, Point p3, int C)
{
	int minx = (p1.x<p2.x) ? (p1.x<p3.x) ? p1.x : p3.x : (p2.x<p3.x) ? p2.x : p3.x;
	int miny = (p1.y<p2.y) ? (p1.y<p3.y) ? p1.y : p3.y : (p2.y<p3.y) ? p2.y : p3.y;
	p1.x -= minx - 1;
	p2.x -= minx - 1;
	p3.x -= minx - 1;
	p1.y -= miny - 1;
	p2.y -= miny - 1;
	p3.y -= miny - 1;

	int tmp = (p1.x*p2.y + p2.x*p3.y + p3.x*p1.y) - (p1.x*p3.y + p2.x*p1.y + p3.x*p2.y);
	int result = (tmp >= -1 * C && tmp <= C) ? 0 : tmp;
	return result;
}

bool compx(Point &i, Point &j)
{
	return i.x<j.x;
}

bool compy(Point &i, Point &j)
{
	return i.y<j.y;
}

bool isEqualPoint(Point p1, Point p2, int C)
{
	float xx = pow(p1.x - p2.x, 2);
	float yy = pow(p1.y - p2.y, 2);
	float dist = sqrt(xx + yy);
	return (dist <= C);
}

vector<Point> ContourOptimization(vector<Point>& points)
{
	vector<Point> result;
	vector<bool> check(points.size());
	int C = 20;

	sort(points.begin(), points.end(), compx);

	for (int i = points.size() / 2; i >= 0; i--) {
		for (int j = 0; j<points.size(); j++) {
			if (i != j && !check[j] && isEqualPoint(points[i], points[j], C)) {
				check[i] = true;
				break;
			}
		}
	}

	for (int i = points.size() / 2 + 1; i<points.size(); i++) {
		for (int j = 0; j<points.size(); j++) {
			if (i != j && !check[j] && isEqualPoint(points[i], points[j], C)) {
				check[i] = true;
				break;
			}
		}
	}

	for (int i = 0; i<check.size(); i++) if (!check[i]) result.push_back(points[i]);


	sort(result.begin(), result.end(), compx);

	return result;
}

void get4SidePoints(vector<Point>& points, vector<Point>& left, vector<Point>& top, vector<Point>& right, vector<Point>& bottom)
{
	vector<bool> check(points.size());
	Point p1, p2, p3;
	int C1 = 150;
	int C2 = 300;
	int C3 = 300;
	int C4 = 300;

	///left
	p1 = points[0];
	p2 = points[1];
	check[0] = true;
	check[1] = true;
	left.push_back(p1);
	left.push_back(p2);

	for (int i = 2; i<points.size(); i++) {
		p3 = points[i];
		int ccwvalue = ccw(p1, p2, p3, C1);
		if (!check[i] && ccwvalue == 0) {
			check[i] = true;
			left.push_back(p3);
			p1 = p2;
			p2 = p3;
		}
	}
	sort(left.begin(), left.end(), compy);

	///top
	p1 = left[0];
	for (int i = 0; i<points.size(); i++) {
		int ccwvalue = ccw(left[1], left[0], points[i], 0);
		if (ccwvalue >= 1050 && ccwvalue <= 1100) {
			check[i] = true;
			p2 = points[i];
		}
	}
	top.push_back(p1);
	top.push_back(p2);
	for (int i = 0; i<points.size(); i++) {
		p3 = points[i];
		int ccwvalue = ccw(p1, p2, p3, C2);
		if (!check[i] && ccwvalue == 0) {
			check[i] = true;
			top.push_back(p3);
			p1 = p2;
			p2 = p3;
		}
	}
	sort(top.begin(), top.end(), compx);

	///right
	p1 = top[top.size() - 1];
	for (int i = 0; i<points.size(); i++) {
		int ccwvalue = ccw(top[top.size() - 2], top[top.size() - 1], points[i], 0);
		if (ccwvalue >= 900 && ccwvalue <= 950) {
			check[i] = true;
			p2 = points[i];
		}
	}
	right.push_back(p1);
	right.push_back(p2);
	for (int i = 0; i<points.size(); i++) {
		p3 = points[i];
		int ccwvalue = ccw(p1, p2, p3, C3);
		if (!check[i] && ccwvalue == 0) {
			check[i] = true;
			right.push_back(p3);
			p1 = p2;
			p2 = p3;
		}
	}
	sort(right.begin(), right.end(), compy);

	///bottom
	p1 = right[right.size() - 1];
	for (int i = 0; i<points.size(); i++) {
		int ccwvalue = ccw(right[right.size() - 2], right[right.size() - 1], points[i], 0);
		if (ccwvalue >= 1850 && ccwvalue <= 1900) {
			check[i] = true;
			p2 = points[i];
		}
	}
	bottom.push_back(p1);
	bottom.push_back(p2);
	for (int i = points.size() - 1; i >= 0; i--) {
		p3 = points[i];
		int ccwvalue = ccw(p1, p2, p3, C4);
		if (!check[i] && ccwvalue == 0) {
			check[i] = true;
			bottom.push_back(p3);
			p1 = p2;
			p2 = p3;
		}
	}
	bottom.push_back(left[left.size() - 1]);
	sort(bottom.begin(), bottom.end(), compx);

	Point tmp;
	tmp.x = (top[2].x + top[3].x) / 2;
	tmp.y = (top[2].y + top[3].y) / 2;
	top.insert(top.begin() + 3, tmp);

	tmp.x = (left[8].x + left[9].x) / 2;
	tmp.y = (left[8].y + left[9].y) / 2;
	left.insert(left.begin() + 9, tmp);

	tmp.x = (right[8].x + right[9].x) / 2;
	tmp.y = (right[8].y + right[9].y) / 2;
	right.insert(right.begin() + 9, tmp);

	for (int i = 1; i <= 4; i++) {
		tmp.x = ((5 - i)*bottom[1].x + i*bottom[2 + i - 1].x) / 5;
		tmp.y = ((5 - i)*bottom[1].y + i*bottom[2 + i - 1].y) / 5;
		bottom.insert(bottom.begin() + 1 + i, tmp);
	}
}

void GridRecovery(Mat &src_image, Mat &dst_image, vector<vector<Point>> &points)
{
	Mat image = src_image.clone();
	cvtColor(image, image, CV_RGB2GRAY);

	Mat eig_image = Mat(image.cols, image.rows, 0);
	Mat temp_image = Mat(image.cols, image.rows, 0);
	Mat gbi1 = image.clone();
	Mat gbi2 = image.clone();
	Mat gbi3 = image.clone();
	vector<Point> corners;
	vector<Point> left_points;
	vector<Point> top_points;
	vector<Point> right_points;
	vector<Point> bottom_points;
	int tmp = 13;

	int corner_count = 100;
	//goodFeaturesToTrack(image, corners, corner_count, 0.05, 5.0);

	double qualityLevel = 0.01;
	double minDistance = 20.;
	cv::Mat mask;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;

	cv::goodFeaturesToTrack(image, corners, corner_count, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k);

	cout<<corners.size()<<endl;

	return;

	corners = ContourOptimization(corners);

	for (int i = 0; i<corners.size(); i++) circle(image, corners[i], 5, CV_RGB(255, 0, 0));

	imshow("corners", image);

	get4SidePoints(corners, left_points, top_points, right_points, bottom_points);

	for (int i = 0; i<left_points.size(); i++) circle(gbi1, cvPoint(left_points[i].x, left_points[i].y), 8, CV_RGB(255, 255, 255), -1, 8);
	for (int i = 0; i<top_points.size(); i++) circle(gbi1, cvPoint(top_points[i].x, top_points[i].y), 7, CV_RGB(255, 0, 0), -1, 8);
	for (int i = 0; i<right_points.size(); i++) circle(gbi1, cvPoint(right_points[i].x, right_points[i].y), 6, CV_RGB(255, 255, 255), -1, 8);
	for (int i = 0; i<bottom_points.size(); i++) circle(gbi1, cvPoint(bottom_points[i].x, bottom_points[i].y), 5, CV_RGB(255, 0, 0), -1, 8);

	vector<Point> temp_points = top_points;
	vector<Point> prev_points = temp_points;
	int cnt_line = 1;
	int block_size = (int)top_points.size();
	int max_size = (int)left_points.size();
	int gap = max_size - block_size;

	points.push_back(temp_points);
	for (; block_size + cnt_line <= max_size; cnt_line++) {
		temp_points.clear();
		temp_points.push_back(left_points[cnt_line]);
		for (int i = 1; i<block_size - 1; i++) {
			Point2f o1 = prev_points[i + 1];
			Point2f p1 = left_points[i + cnt_line];
			Point2f o2 = prev_points[i - 1];
			Point2f p2 = right_points[block_size - i + cnt_line - 1];

			Point2f x = o2 - o1;
			Point2f d1 = p1 - o1;
			Point2f d2 = p2 - o2;
			Point interpoint;

			float cross = d1.x*d2.y - d1.y*d2.x;
			if (abs(cross) < 1e-8) interpoint = Point(-1, -1);

			double t1 = (x.x * d2.y - x.y * d2.x) / cross;
			interpoint = o1 + d1 * t1;

			temp_points.push_back(interpoint);

			line(gbi1, o1, p1, CV_RGB(255, 0, 0));
			line(gbi1, o2, p2, CV_RGB(255, 0, 0));
			circle(gbi1, interpoint, 5, CV_RGB(255, 0, 0));
		}
		temp_points.push_back(right_points[cnt_line]);
		points.push_back(temp_points);
		prev_points = temp_points;
	}

	for (; cnt_line<max_size; cnt_line++) {
		temp_points.clear();
		temp_points.push_back(left_points[cnt_line]);
		for (int i = 1; i<block_size - 1; i++) {
			Point2f o1;
			Point2f p1;
			Point2f o2;
			Point2f p2;

			int line1_index = cnt_line - gap + i;
			int line2_index1 = i - cnt_line + gap;
			int line2_index2 = block_size - 1 - line2_index1;

			if (line1_index >= block_size) {
				line1_index = line1_index - block_size + 1;
				o1 = right_points[line1_index + gap];
				p1 = bottom_points[line1_index];
			}
			else {
				o1 = prev_points[line1_index];
				p1 = left_points[line1_index + gap];
			}

			if (line2_index1<0) {
				line2_index1 = abs(line2_index1);
				line2_index2 = block_size - 1 - line2_index1;
				o2 = left_points[line2_index1 + gap];
				p2 = bottom_points[line2_index2];
			}
			else {
				o2 = prev_points[line2_index1];
				p2 = right_points[line2_index2 + gap];
			}

			Point2f x = o2 - o1;
			Point2f d1 = p1 - o1;
			Point2f d2 = p2 - o2;
			Point interpoint;

			float cross = d1.x*d2.y - d1.y*d2.x;
			if (abs(cross) < 1e-8) interpoint = Point(-1, -1);

			double t1 = (x.x * d2.y - x.y * d2.x) / cross;
			interpoint = o1 + d1 * t1;

			temp_points.push_back(interpoint);

			line(gbi1, o1, p1, CV_RGB(255, 0, 0));
			line(gbi1, o2, p2, CV_RGB(255, 0, 0));
			circle(gbi1, interpoint, 5, CV_RGB(255, 0, 0));
		}
		temp_points.push_back(right_points[cnt_line]);
		points.push_back(temp_points);
	}
	points.push_back(bottom_points);

	for (int i = 0; i<points.size(); i++) {
		for (int j = 0; j<points[i].size(); j++) {
			circle(gbi2, points[i][j], 5, CV_RGB(255, 0, 0));
		}
	}

	imshow("corner2", image);
	imshow("line", gbi1);
	imshow("result", gbi2);
}