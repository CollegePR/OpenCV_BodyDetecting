#include "main.h"
#include "GrabCut.h"
#include "BodyDetecting.h"
#include "GridRecovery.h"
#include "BodyLineFitting.h"

int main()
{
	Mat org_image = imread("image.jpeg");	//���� �̹���
	Mat org_image2 = org_image.clone();
	Mat foreground_image;	//�̹��� ����
	Mat background_image;	//�̹��� ���
	Mat grabcut_result_image;		//�׷��� ����̹���
	Mat grid_recovery_image;	//������ ���� �̹���
	vector<vector<Point>> grid_points;
	vector<Point> body_contour;

	imshow("orginal", org_image);

	//�׷���(��ó��(ħ��,��â), ����/��� ����)
	GrabCut(org_image, foreground_image, background_image, grabcut_result_image);

	//���ó��
	GridRecovery(background_image, grid_recovery_image, grid_points);

	//����ó��
	BodyDetecting(grabcut_result_image, body_contour);
	for (int i = 0; i<body_contour.size(); i++) circle(org_image2, body_contour[i], 5, CV_RGB(255, 0, 0));
	imshow("body contour", org_image2);

	BodyLineFitting(org_image, body_contour);
	waitKey(0);
	return 0;
}