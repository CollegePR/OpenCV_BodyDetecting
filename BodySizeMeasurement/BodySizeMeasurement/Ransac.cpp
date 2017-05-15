#include "Ransac.h"


bool find_in_samples(new_point *samples, int no_samples, new_point *data)
{
	for (int i = 0; i<no_samples; ++i) {
		if (samples[i].x == data->x && samples[i].y == data->y) {
			return true;
		}
	}
	return false;
}

void get_samples(new_point *samples, int no_samples, new_point *data, int no_data)
{
	// �����Ϳ��� �ߺ����� �ʰ� N���� ������ ������ ä���Ѵ�.
	for (int i = 0; i<no_samples; ) {
		int j = rand() % no_data;

		if (!find_in_samples(samples, i, &data[j])) {
			samples[i] = data[j];
			++i;
		}
	};
}

int compute_model_parameter(new_point samples[], int no_samples, new_line &model)
{
	// PCA ������� ���� ���� �Ķ���͸� �����Ѵ�.

	double sx = 0, sy = 0;
	double sxx = 0, syy = 0;
	double sxy = 0, sw = 0;

	for (int i = 0; i<no_samples; ++i)
	{
		double &x = samples[i].x;
		double &y = samples[i].y;

		sx += x;
		sy += y;
		sxx += x*x;
		sxy += x*y;
		syy += y*y;
		sw += 1;
	}

	//variance;
	double vxx = (sxx - sx*sx / sw) / sw;
	double vxy = (sxy - sx*sy / sw) / sw;
	double vyy = (syy - sy*sy / sw) / sw;

	//principal axis
	double theta = atan2(2 * vxy, vxx - vyy) / 2;

	model.mp.x = cos(theta);
	model.mp.y = sin(theta);

	//center of mass(xc, yc)
	model.sp.x = sx / sw;
	model.sp.y = sy / sw;


	//������ ������: sin(theta)*(x - sx) = cos(theta)*(y - sy);
	return 1;
}

double compute_distance(new_line &nline, new_point &x)
{
	// �� ��(x)�κ��� ����(line)�� ���� ������ ����(distance)�� ����Ѵ�.
	double res = fabs((x.x - nline.sp.x)*nline.mp.y - (x.y - nline.sp.y)*nline.mp.x) / sqrt(nline.mp.x*nline.mp.x + nline.mp.y*nline.mp.y);
	return res;
}

double model_verification(new_point *inliers, int *no_inliers, new_line &estimated_model, new_point *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;

	double cost = 0.;

	for (int i = 0; i<no_data; i++) {
		// ������ ���� ������ ���̸� ����Ѵ�.
		double distance = compute_distance(estimated_model, data[i]);

		// ������ �𵨿��� ��ȿ�� �������� ���, ��ȿ�� ������ ���տ� ���Ѵ�.
		if (distance < distance_threshold) {
			cost += 1.;

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}

	return cost;
}

double ransac_line_fitting(new_point *data, int no_data, new_line &model, double distance_threshold)
{
	const int no_samples = 2;

	if (no_data < no_samples) {
		return 0.;
	}

	new_point *samples = new new_point[no_samples];

	int no_inliers = 0;
	new_point *inliers = new new_point[no_data];

	new_line estimated_model;
	double max_cost = 0.;

	int max_iteration = (int)(1 + log(1. - 0.99) / log(1. - pow(0.5, no_samples)));

	for (int i = 0; i<max_iteration; i++) {
		// 1. hypothesis

		// ���� �����Ϳ��� ���Ƿ� N���� ���� �����͸� ����.
		get_samples(samples, no_samples, data, no_data);

		// �� �����͸� �������� �����ͷ� ���� �� �Ķ���͸� �����Ѵ�.
		compute_model_parameter(samples, no_samples, estimated_model);

		// 2. Verification

		// ���� �����Ͱ� ������ �𵨿� �� �´��� �˻��Ѵ�.
		double cost = model_verification(inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);

		// ���� ������ ���� �� �´´ٸ�, �� �𵨿� ���� ��ȿ�� �����ͷ� ���ο� ���� ���Ѵ�.
		if (max_cost < cost) {
			max_cost = cost;

			compute_model_parameter(inliers, no_inliers, model);
		}
	}

	delete[] samples;
	delete[] inliers;

	return max_cost;
}

new_line Ransac(Mat &src_image, vector<Point> contours, int s, int f, Scalar sc) {
	int n = f - s + 1;
	new_point *data = new new_point[n];

	if (contours.size() <= s || contours.size() <= f) {
		new_line res;
		new_point p;
		p.x = 0;
		p.y = 0;
		res.mp = p;
		res.sp = p;
		return res;
	}

	// 759/329
	for (int i = s; i <= f; i++) {
		data[i - s].x = (double)contours[i].x;
		data[i - s].y = (double)contours[i].y;
	}

	/*for(int i = 0; i<n; i++) {
	circle(image, Point(data[i].x,data[i].y), 3, Scalar(255,0,0));
	//pDC->SetPixel((int)data[i].x, (int)data[i].y, RGB(255,0,0));
	}*/

	new_line res_line;
	double cost = ransac_line_fitting(data, n, res_line, 30);

	new_point p1;
	p1.x = (int)(res_line.sp.x - 500 * res_line.mp.x);
	p1.y = (int)(res_line.sp.y - 500 * res_line.mp.y);

	new_point p2;
	p2.x = (int)(res_line.sp.x + 500 * res_line.mp.x);
	p2.y = (int)(res_line.sp.y + 500 * res_line.mp.y);

	if (50. < cost) {
		line(src_image, Point(p1.x, p1.y), Point(p2.x, p2.y), sc, 2);
		//pDC->MoveTo((int)(line.sx-500*line.mx), (int)(line.sy-500*line.my));
		//pDC->LineTo((int)(line.sx+500*line.mx), (int)(line.sy+500*line.my));
	}

	new_line result;
	result.sp = p1;
	result.mp = p2;

	return result;
}

