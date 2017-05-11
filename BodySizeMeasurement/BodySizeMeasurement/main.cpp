#include "main.h"

int main()
{
	Mat org_image = imread("image.jpeg");

	imshow("rest", org_image);
	waitKey(0);
	return 0;
}