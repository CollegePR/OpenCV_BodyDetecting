#include "main.h"

int main()
{
	Mat org_image = imread("image.jpeg");

	imshow("rest", org_image);
	return 0;
}