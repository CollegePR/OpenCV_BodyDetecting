#include "main.h"
#include "GrabCut.h"
#include "BodyDetecting.h"
#include "GridRecovery.h"

int main()
{
	Mat org_image = imread("image.jpeg");	//원본 이미지
	Mat pre_image;	//그랩컷 전처리 이미지
	Mat foreground_image;	//이미지 전경
	Mat background_image;	//이미지 배경
	Mat grid_recovery_image;	//격자점 복원 이미지

	//그랩컷 전처리함수(침식,팽창)
	Preprocess(org_image, pre_image);
	//그랩컷(전경,배경 추출)
	GrabCut(pre_image, foreground_image, background_image);

	imshow("org_image", org_image);
	imshow("pre_image", pre_image);
	imshow("foreground", foreground_image);
	imshow("background", background_image);
	waitKey(0);
	return 0;
}