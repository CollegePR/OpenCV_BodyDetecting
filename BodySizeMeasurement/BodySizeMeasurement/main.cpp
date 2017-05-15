#include "main.h"
#include "GrabCut.h"
#include "BodyDetecting.h"
#include "GridRecovery.h"
#include "BodyLineFitting.h"

int main()
{
	Mat org_image = imread("image.jpeg");	//원본 이미지
	Mat org_image2 = org_image.clone();
	Mat foreground_image;	//이미지 전경
	Mat background_image;	//이미지 배경
	Mat grabcut_result_image;		//그랩컷 결과이미지
	Mat grid_recovery_image;	//격자점 복원 이미지
	vector<vector<Point>> grid_points;
	vector<Point> body_contour;

	imshow("orginal", org_image);

	//그랩컷(전처리(침식,팽창), 전경/배경 추출)
	GrabCut(org_image, foreground_image, background_image, grabcut_result_image);

	//배경처리
	GridRecovery(background_image, grid_recovery_image, grid_points);

	//전경처리
	BodyDetecting(grabcut_result_image, body_contour);
	for (int i = 0; i<body_contour.size(); i++) circle(org_image2, body_contour[i], 5, CV_RGB(255, 0, 0));
	imshow("body contour", org_image2);

	BodyLineFitting(org_image, body_contour);
	waitKey(0);
	return 0;
}