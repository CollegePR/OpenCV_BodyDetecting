#include "main.h"
#include "GrabCut.h"
#include "BodyDetecting.h"
#include "GridRecovery.h"

int main()
{
	Mat org_image = imread("image.jpeg");	//���� �̹���
	Mat foreground_image;	//�̹��� ����
	Mat background_image;	//�̹��� ���
	Mat grid_recovery_image;	//������ ���� �̹���
	vector<vector<Point>> grid_points;

	imshow("orginal", org_image);

	//�׷���(��ó��(ħ��,��â), ����/��� ����)
	GrabCut(org_image, foreground_image, background_image);

	//���ó��
	GridRecovery(background_image, grid_recovery_image, grid_points);


	//����ó��

	waitKey(0);
	return 0;
}