#include "main.h"
#include "GrabCut.h"
#include "BodyDetecting.h"
#include "GridRecovery.h"

int main()
{
	Mat org_image = imread("image.jpeg");	//원본 이미지
	Mat foreground_image;	//이미지 전경
	Mat background_image;	//이미지 배경
	Mat grid_recovery_image;	//격자점 복원 이미지
	vector<vector<Point>> grid_points;

	imshow("orginal", org_image);

	//그랩컷(전처리(침식,팽창), 전경/배경 추출)
	GrabCut(org_image, foreground_image, background_image);

	//배경처리
	GridRecovery(background_image, grid_recovery_image, grid_points);


	//전경처리

	waitKey(0);
	return 0;
}