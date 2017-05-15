#include "BodyLineFitting.h"
#include "Ransac.h"

void drawInterPoint(Mat image, new_line line1, new_line line2, Scalar sc) {
	Point2f o1 = Point2f(line1.sp.x, line1.sp.y);
	Point2f p1 = Point2f(line1.mp.x, line1.mp.y);
	Point2f o2 = Point2f(line2.sp.x, line2.sp.y);
	Point2f p2 = Point2f(line2.mp.x, line2.mp.y);

	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;
	Point result;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/ 1e-8) result = Point(-1, -1);

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	result = o1 + d1 * t1;

	//cout<<o1<<" , "<<p1<<" , "<<o2<<" , "<<p2<<endl;
	//cout<<result<<endl;

	circle(image, result, 7, sc);
}

void BodyLineFitting(Mat & src_image, vector<Point> contour)
{
	Mat image = src_image.clone();

	new_line nline[10];
	nline[0] = Ransac(image, contour, 100, 150, Scalar(255, 0, 0));
	nline[1] = Ransac(image, contour, 150, 400, Scalar(0, 255, 0));
	nline[2] = Ransac(image, contour, 400, 420, Scalar(0, 0, 255));
	nline[3] = Ransac(image, contour, 420, 570, Scalar(255, 255, 0));
	nline[4] = Ransac(image, contour, 570, 680, Scalar(0, 255, 255));

	nline[5] = Ransac(image, contour, 820, 930, Scalar(0, 255, 255));
	nline[6] = Ransac(image, contour, 930, 1080, Scalar(255, 255, 0));
	nline[7] = Ransac(image, contour, 1080, 1100, Scalar(0, 0, 255));
	nline[8] = Ransac(image, contour, 1100, 1340, Scalar(0, 255, 0));
	nline[9] = Ransac(image, contour, 1340, 1400, Scalar(255, 0, 0));

	drawInterPoint(image, nline[0], nline[1], Scalar(255, 0, 0));
	drawInterPoint(image, nline[3], nline[4], Scalar(255, 0, 0));

	drawInterPoint(image, nline[5], nline[6], Scalar(255, 0, 0));
	drawInterPoint(image, nline[8], nline[9], Scalar(255, 0, 0));

	imshow("BodyLineFitting", image);
}
