#include "GrabCut.h"

////////////////////////////////////////
// 침식 & 팽창 과정을 통해 흰색격자선을
// 뭉개서 검정배경에 신체형태만 남도록 처리
////////////////////////////////////////
void Preprocess(Mat & src_image, Mat & dst_image)
{
	
	dst_image = src_image.clone();
	erode(dst_image, dst_image, Mat(), Point(-1, -1), 6);
	dilate(dst_image, dst_image, Mat(), Point(-1, -1), 6);
}

////////////////////////////////////////
//그랩컷 알고리즘을 이용해 전경(신체)과 배경(격자) 분할
////////////////////////////////////////
void GrabCut(Mat & src_image, Mat & foreground_dst_image, Mat & background_dst_image, Mat & result_image)
{
	Mat pre_image;

	Preprocess(src_image, pre_image);

	Rect grabcut_image_range(0, 0, pre_image.cols - 31, pre_image.rows - 31);

	Mat grabcut_result_image;
	Mat grabcut_background;
	Mat grabcut_background_model, grabcut_foreground_model;

	grabCut(pre_image, grabcut_result_image, grabcut_image_range, grabcut_background_model, grabcut_foreground_model, 5, GC_INIT_WITH_RECT);

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

	result_image = grabcut_result_image;
	foreground_dst_image = grabcut_foreground_image;
	background_dst_image = grabcut_background_image;

	imshow("foreground", foreground_dst_image);
	imshow("background", background_dst_image);
	imshow("grabcut", result_image);
}
