#include "BodyDetecting.h"
#include "GrabCut.h"

void BodyDetecting(Mat & src_image, vector<Point> & body_contour)
{
	//�׷��� �˰����� ������� ���� ����̹����� ��ü�� �ܰ��� ����
	vector<vector<Point>> grabcut_contours;

	findContours(src_image, grabcut_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	vector<vector<Point>>::const_iterator itc = grabcut_contours.begin();

	Mat result(src_image.size(), CV_8U, Scalar(255));

	vector<Point> grabcut_contour = grabcut_contours[0];

	//����� �ܰ����� �� ���� ���̰� �� �ܰ��� ����
	while (itc != grabcut_contours.end()) {
		if (grabcut_contour.size()<(*itc).size()) grabcut_contour = *itc;
		itc++;
	}

	body_contour = grabcut_contour;
}
