#include "GrabCut.h"

void Preprocess(Mat & src_image, Mat & dst_image)
{
	//ħ�� & ��â ������ ���� ������ڼ��� ������ ������濡 ��ü���¸� ������ ó��
	dst_image = src_image.clone();
	erode(dst_image, dst_image, Mat(), Point(-1, -1), 6);
	dilate(dst_image, dst_image, Mat(), Point(-1, -1), 6);
}

void GrabCut(Mat & src_image, Mat & foreground_dst_image, Mat & background_dst_image)
{
	//�׷��� �˰����� �̿��� ����(��ü)�� ���(�������) ����
	Rect grabcut_image_range(0, 0, src_image.cols - 31, src_image.rows - 31);

	Mat grabcut_result_image;
	Mat grabcut_background;
	Mat grabcut_background_model, grabcut_foreground_model;

	grabCut(src_image, grabcut_result_image, grabcut_image_range, grabcut_background_model, grabcut_foreground_model, 5, GC_INIT_WITH_RECT);

	compare(grabcut_result_image, GC_PR_BGD, grabcut_background, CMP_EQ);
	compare(grabcut_result_image, GC_PR_FGD, grabcut_result_image, CMP_EQ);

	Mat grabcut_foreground_image(src_image.size(), CV_8UC3, Scalar(255, 255, 255));
	Mat grabcut_background_image = src_image.clone();
	src_image.copyTo(grabcut_foreground_image, grabcut_result_image);

	for (int i = 0; i<grabcut_background_image.rows; i++) {
		for (int j = 0; j<grabcut_background_image.cols; j++) {
			if (grabcut_result_image.at<uchar>(i, j) == 255) {
				grabcut_background_image.at<Vec3b>(i, j)[0] = 255;
				grabcut_background_image.at<Vec3b>(i, j)[1] = 255;
				grabcut_background_image.at<Vec3b>(i, j)[2] = 255;
			}
		}
	}

	foreground_dst_image = grabcut_foreground_image;
	background_dst_image = grabcut_background_image;
}
