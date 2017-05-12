#include "main.h"
#include "GrabCut.h"
#include "BodyDetecting.h"
#include "GridRecovery.h"

int main()
{
	Mat org_image = imread("image.jpeg");	//���� �̹���
	Mat pre_image;	//�׷��� ��ó�� �̹���
	Mat foreground_image;	//�̹��� ����
	Mat background_image;	//�̹��� ���
	Mat grid_recovery_image;	//������ ���� �̹���

	//�׷��� ��ó���Լ�(ħ��,��â)
	Preprocess(org_image, pre_image);
	//�׷���(����,��� ����)
	GrabCut(pre_image, foreground_image, background_image);

	imshow("org_image", org_image);
	imshow("pre_image", pre_image);
	imshow("foreground", foreground_image);
	imshow("background", background_image);
	waitKey(0);
	return 0;
}