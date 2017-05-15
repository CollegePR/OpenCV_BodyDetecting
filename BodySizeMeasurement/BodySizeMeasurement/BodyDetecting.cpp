#include "BodyDetecting.h"
#include "GrabCut.h"

void BodyDetecting(Mat & src_image, vector<Point> & body_contour)
{
	//그랩컷 알고리즘을 적용시켜 얻은 결과이미지로 신체의 외곽선 추출
	vector<vector<Point>> grabcut_contours;

	findContours(src_image, grabcut_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	vector<vector<Point>>::const_iterator itc = grabcut_contours.begin();

	Mat result(src_image.size(), CV_8U, Scalar(255));

	vector<Point> grabcut_contour = grabcut_contours[0];

	//추출된 외곽선들 중 가장 길이가 긴 외곽선 선택
	while (itc != grabcut_contours.end()) {
		if (grabcut_contour.size()<(*itc).size()) grabcut_contour = *itc;
		itc++;
	}

	body_contour = grabcut_contour;
}
