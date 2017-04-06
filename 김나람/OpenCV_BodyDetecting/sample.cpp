#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

struct new_point{
    double x;
    double y;
};

struct new_line{
    new_point sp;
    new_point mp;
};

class WatershedSegmenter {
private:
    Mat markers;
    
public:
    void setMarkers(const Mat& markerImage) {
        markerImage.convertTo(markers,CV_32S);
        // 정수형 영상 변환
    }
    
    Mat process(const Mat &image) {
        watershed(image,markers);
        // 워터쉐드 적용
        return markers;
    }
    
    Mat getSegmentation() { // 영상 형태인 결과를 반환
        Mat tmp;
        markers.convertTo(tmp,CV_8U);
        // 255 이상인 레이블을 갖는 모든 분할은 255인 값으로 할당
        return tmp;
    }
    
    Mat getWatersheds() { // 영상 형태인 워터쉐드를 반환
        Mat tmp;
        markers.convertTo(tmp,CV_8U,255,255);
        return tmp;
    }
};

/*
 
 (X1,y1)
 (X2,y2)
 
 y=ax + b
 a=(y2-y1)/(x2-x1)
 b=y1-a*x1
 
 
 
 y=ax+b
 y=cx+d
 
 ax+b=cx+d
 
 (a-c)x=d-b
 
 x=(d-b)/(a-c)
 y=ax+b
 */

Point intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;
    
    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/ 1e-8)
        return Point(-1,-1) ;
    
    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    Point r = o1 + d1 * t1;
    
    cout<<o1<<" , "<<p1<<" , "<<o2<<" , "<<p2<<endl;
    return r ;
}

void drawInterPoint(Mat image, new_line line1, new_line line2, Scalar sc=Scalar(255,255,255)){
    Point2f o1=Point2f(line1.sp.x,line1.sp.y);
    Point2f p1=Point2f(line1.mp.x,line1.mp.y);
    Point2f o2=Point2f(line2.sp.x,line2.sp.y);
    Point2f p2=Point2f(line2.mp.x,line2.mp.y);
    
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;
    Point result;
    
    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/ 1e-8) result=Point(-1,-1);
    
    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    result = o1 + d1 * t1;
    
    cout<<o1<<" , "<<p1<<" , "<<o2<<" , "<<p2<<endl;
    cout<<result<<endl;
    
    circle(image, result, 7, sc);
}

bool find_in_samples (new_point *samples, int no_samples, new_point *data)
{
    for (int i=0; i<no_samples; ++i) {
        if (samples[i].x == data->x && samples[i].y == data->y) {
            return true;
        }
    }
    return false;
}

void get_samples (new_point *samples, int no_samples, new_point *data, int no_data)
{
    // 데이터에서 중복되지 않게 N개의 무작위 셈플을 채취한다.
    for (int i=0; i<no_samples; ) {
        int j = rand()%no_data;
        
        if (!find_in_samples(samples, i, &data[j])) {
            samples[i] = data[j];
            ++i;
        }
    };
}

int compute_model_parameter(new_point samples[], int no_samples, new_line &model)
{
    // PCA 방식으로 직선 모델의 파라메터를 예측한다.
    
    double sx  = 0, sy  = 0;
    double sxx = 0, syy = 0;
    double sxy = 0, sw  = 0;
    
    for(int i = 0; i<no_samples;++i)
    {
        double &x = samples[i].x;
        double &y = samples[i].y;
        
        sx  += x;
        sy  += y;
        sxx += x*x;
        sxy += x*y;
        syy += y*y;
        sw  += 1;
    }
    
    //variance;
    double vxx = (sxx - sx*sx/sw)/sw;
    double vxy = (sxy - sx*sy/sw)/sw;
    double vyy = (syy - sy*sy/sw)/sw;
    
    //principal axis
    double theta = atan2(2*vxy, vxx - vyy)/2;
    
    model.mp.x = cos(theta);
    model.mp.y = sin(theta);
    
    //center of mass(xc, yc)
    model.sp.x = sx/sw;
    model.sp.y = sy/sw;
    
    
    //직선의 방정식: sin(theta)*(x - sx) = cos(theta)*(y - sy);
    return 1;
}

double compute_distance(new_line &nline, new_point &x)
{
    // 한 점(x)로부터 직선(line)에 내린 수선의 길이(distance)를 계산한다.
    double res=fabs((x.x - nline.sp.x)*nline.mp.y - (x.y -nline.sp.y)*nline.mp.x)/sqrt(nline.mp.x*nline.mp.x + nline.mp.y*nline.mp.y);
    return res;
}

double model_verification (new_point *inliers, int *no_inliers, new_line &estimated_model, new_point *data, int no_data, double distance_threshold)
{
    *no_inliers = 0;
    
    double cost = 0.;
    
    for(int i=0; i<no_data; i++){
        // 직선에 내린 수선의 길이를 계산한다.
        double distance = compute_distance(estimated_model, data[i]);
        
        // 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
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
    
    int max_iteration = (int)(1 + log(1. - 0.99)/log(1. - pow(0.5, no_samples)));
    
    for (int i = 0; i<max_iteration; i++) {
        // 1. hypothesis
        
        // 원본 데이터에서 임의로 N개의 셈플 데이터를 고른다.
        get_samples (samples, no_samples, data, no_data);
        
        // 이 데이터를 정상적인 데이터로 보고 모델 파라메터를 예측한다.
        compute_model_parameter (samples, no_samples, estimated_model);
        
        // 2. Verification
        
        // 원본 데이터가 예측된 모델에 잘 맞는지 검사한다.
        double cost = model_verification (inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);
        
        // 만일 예측된 모델이 잘 맞는다면, 이 모델에 대한 유효한 데이터로 새로운 모델을 구한다.
        if (max_cost < cost) {
            max_cost = cost;
            
            compute_model_parameter (inliers, no_inliers, model);
        }
    }
    
    delete [] samples;
    delete [] inliers;
    
    return max_cost;
}

new_line drawRansac(Mat image, vector<Point> contours, int s, int f, Scalar sc){
    int n=f-s+1;
    new_point *data = new new_point[n];
    
    // 759/329
    for (int i=s; i<=f; i++) {
        data[i-s].x=(double)contours[i].x;
        data[i-s].y=(double)contours[i].y;
    }
    
    /*for(int i = 0; i<n; i++) {
        circle(image, Point(data[i].x,data[i].y), 3, Scalar(255,0,0));
        //pDC->SetPixel((int)data[i].x, (int)data[i].y, RGB(255,0,0));
    }*/
    
    new_line res_line;
    double cost = ransac_line_fitting (data, n, res_line, 30);
    
    new_point p1;
    p1.x=(int)(res_line.sp.x-500*res_line.mp.x);
    p1.y=(int)(res_line.sp.y-500*res_line.mp.y);
    
    new_point p2;
    p2.x=(int)(res_line.sp.x+500*res_line.mp.x);
    p2.y=(int)(res_line.sp.y+500*res_line.mp.y);
    
    if (50. < cost) {
        line(image, Point(p1.x,p1.y),Point(p2.x,p2.y), sc,2);
        //pDC->MoveTo((int)(line.sx-500*line.mx), (int)(line.sy-500*line.my));
        //pDC->LineTo((int)(line.sx+500*line.mx), (int)(line.sy+500*line.my));
    }
    
    new_line result;
    result.sp=p1;
    result.mp=p2;
    
    cout<<"["<<p1.x<<", "<<p1.y<<"], ["<<p2.x<<", "<<p2.y<<"]"<<endl;
    
    return result;
}

void drawExLine(Mat image, Point p1, Point p2, Scalar s, int size){
    line(image, p1, p2, s, size);
}

bool ccw(Point p1, Point p2, Point p3, int C){
    int minx=(p1.x<p2.x)?(p1.x<p3.x)?p1.x:p3.x:(p2.x<p3.x)?p2.x:p3.x;
    int miny=(p1.y<p2.y)?(p1.y<p3.y)?p1.y:p3.y:(p2.y<p3.y)?p2.y:p3.y;
    p1.x-=minx-1;
    p2.x-=minx-1;
    p3.x-=minx-1;
    p1.y-=miny-1;
    p2.y-=miny-1;
    p3.y-=miny-1;
    
    int tmp=(p1.x*p2.y+p2.x*p3.y+p3.x*p1.y)-(p1.x*p3.y+p2.x*p1.y+p3.x*p2.y);
    return (tmp>=-1*C && tmp<=C);
}

int _ccw(Point p1, Point p2, Point p3){
    return (p1.x*p2.y+p2.x*p3.y+p3.x*p1.y)-(p1.x*p3.y+p2.x*p1.y+p3.x*p2.y);
}

vector<Point> point_proc(vector<Point> target, int C, int rep){
    vector<Point> tar=target;
    vector<Point> ret;
    
    for(int tmp=1; tmp<=rep; tmp++){
        Point p1,p2,p3;
        ret.clear();
        if(tar.size()<=3) return ret;
        else{
            p1=tar[0];
            p2=tar[1];
            ret.push_back(p1);
        }
        for(int i=2; i<tar.size(); i++){
            p3=tar[i];
            if(ccw(p1,p2,p3,C)) p2=p3;
            else{
                ret.push_back(p2);
                p1=p2;
                p2=p3;
            }
        }
        tar=ret;
    }
    
    return ret;
}

void contour_proc_test(Mat image){
    vector<vector<Point>> contours;
    
    findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    //int cmin=50;
    //int cmax=1000000;
    
    cout<<contours.size()<<endl;
    
    
    vector<vector<Point>>::const_iterator itc=contours.begin();
    
    Mat result(image.size(),CV_8U,Scalar(255));
    
    vector<Point> target=contours[0];
    
    while(itc!=contours.end()){
        //if(itc->size()<cmin || itc->size()>cmax) itc=contours.erase(itc);
        //else itc++;
        if(target.size()<(*itc).size()) target=*itc;
        itc++;
    }
    
    vector<Point> result_point;
    Point p1,p2,p3;
    
    if(target.size()<=3) return;
    else{
        p1=target[0];
        p2=target[1];
        result_point.push_back(p1);
    }
    
    for(int i=2; i<target.size(); i++){
        p3=target[i];
        
        if(ccw(p1,p2,p3,20)){
            p2=p3;
        }
        else{
            result_point.push_back(p2);
            p1=p2;
            p2=p3;
        }
    }
    
    vector<Point>::const_iterator itp=target.begin();
    while(itp!=target.end()){
        circle(result, *itp, 5, Scalar(128));
        itp++;
    }
    
    itp=result_point.begin();
    Point tmp(-1,-1);
    
    while(itp!=result_point.end()){
        if(tmp.x==-1 && tmp.y==-1) tmp=*itp;
        else line(result, tmp, *itp, Scalar(200),2);
        circle(result, *itp, 3, Scalar(50));
        tmp=*itp;
        itp++;
    }
    line(result, tmp, result_point[0], Scalar(200),2);
    
    /*while(itc!=contours.end()){
        vector<Point>::const_iterator itp=(*itc).begin();
        
        while(itp!=(*itc).end()){
            circle(result, *itp, 5, Scalar(128));
            itp++;
        }
        itc++;
    }*/
    
    imshow("result",result);
}

void create_image(){
    int rat_cnt, gap, size;
    cout << "격자 수 : ";
    cin >> rat_cnt;
    cout << "격자 간격 : ";
    cin >> gap;
    cout << "격자 길이 : ";
    cin >> size;
    
    int y = gap, x = gap;
    
    Mat image = Mat(size*rat_cnt + gap*(rat_cnt + 1), size*rat_cnt + gap*(rat_cnt + 1), 0);
    int w = image.cols;
    int h = image.rows;
    
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            image.at<uchar>(i, j) = 255;
        }
    }
    
    for (int i = 0; i < rat_cnt; i++) {
        for (int j = 0; j < rat_cnt; j++) {
            for (int k = y; k < y + size; k++) {
                for (int l = x; l < x + size; l++) {
                    if (k >= h || l >= w) break;
                    image.at<uchar>(k, l) = 0;
                }
            }
            x += size + gap;
        }
        x = gap;
        y += size + gap;
    }
    
    //imshow("result", result);
    imwrite("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/result.bmp", image);
}

void test(){
    Mat image=imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/result.bmp");
    Mat gray_image;
    Mat th_image;
    
    vector<vector<Point>> contours;
    
    cvtColor(image, gray_image, CV_RGB2GRAY);
    
    threshold(gray_image, th_image, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    
    imshow("th",th_image);
    
    //dilate(th_image, th_image, Mat());
    //dilate(th_image, th_image, Mat());
    
    //th_image=~th_image;
    
    findContours(th_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    //int cmin=50;
    //int cmax=1000000;
    
    cout<<contours.size()<<endl;
    
    vector<Point> target;
    vector<vector<Point>>::const_iterator itc=contours.begin();
    
    target=contours[0];
    
    Mat result(gray_image.size(),CV_8U,Scalar(255));
    
    while(itc!=contours.end()){
        //if(itc->size()<cmin || itc->size()>cmax) itc=contours.erase(itc);
        //else itc++;
        if(target.size()<(*itc).size()) target=*itc;
        itc++;
    }
    
    vector<Point>::const_iterator itp=target.begin();
    
    while(itp!=target.end()){
        circle(result, *itp, 5, Scalar(128));
        itp++;
    }
    
    imshow("result",result);
    
    /*drawContours(result, contours, -1, Scalar(0),2);
     
     int i=0;
     
     while(i<contours.size()){
     //float radius;
     //Point2f center;
     //minEnclosingCircle(Mat(contours[i]), center, radius);
     //circle(result,Point(center),static_cast<int>(radius),Scalar(0),2);
     
     vector<Point> hull;
     convexHull(Mat(contours[i]), hull);
     
     vector<Point>::const_iterator itp=hull.begin();
     while(itp!=(hull.end()-1)){
     line(result,*itp,*(itp+1),Scalar(0));
     ++itp;
     }
     
     i++;
     }*/
    
    //imshow("0",image);
    //imshow("1",th_image);
    //imshow("2",result);
    
    waitKey(0);
}

void grabcut_test(){
    Mat image = imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/result.bmp");
    
    imshow("Original Image", image);
    
    erode(image,image,Mat(),Point(-1,-1),6);
    dilate(image, image, Mat(),Point(-1,-1),6);
    
    imshow("image",image);
    
    Rect rectangle(0, 0, 760, 760);
    
    Mat result; // 분할 (4자기 가능한 값)
    
    Mat bgModel, fgModel; // 모델 (초기 사용)
    
    grabCut(image,    // 입력 영상
                
                result,    // 분할 결과
                
                rectangle,   // 전경을 포함하는 직사각형
                
                bgModel, fgModel, // 모델
                
                5,     // 반복 횟수
                
                GC_INIT_WITH_RECT); // 직사각형 사용
    
    
    
    // 전경일 가능성이 있는 화소를 마크한 것을 가져오기
    
    compare(result, GC_PR_FGD, result, CMP_EQ);
    
    
    
    // 결과 영상 생성
    
    Mat foreground(image.size(), CV_8UC3,
                       
                       Scalar(255, 255, 255));
    
    
    
    image.copyTo(foreground, // 배경 화소는 복사되지 않음
                 
                 result);
    
    //imshow("Result", result);
    
    //imshow("Foreground", foreground);
    
    contour_proc_test(result);
    
    waitKey(0);
}

void watershed_test(){
    Mat image= imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/result.bmp");
    imshow("Original Image",image);
    
    Mat binary;
    Mat th_image;
    
    cvtColor(image, binary, CV_RGB2GRAY);
    
    threshold(binary, th_image, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    
    imshow("binary",binary);
    
    // 이진 영상은 영상의 여러 부분에 속하는 흰색 화소를 많이 포함
    // 중요한 객체에 속하는 화소만 남기기 위해 영상을 여러 번 침식
    Mat fg;
    erode(binary, fg, Mat(), Point(-1, -1), 6);
    // 잡음과 작은 객체 제거
    imshow("Foreground Image", fg);
    // 배경인 숲에 속하는 몇 화소는 여전히 존재
    // 관심 객체에 대응하는 부분을 고려
    
    // 원 이진 영상의 큰 팽창인 배경의 화소를 선택
    Mat bg;
    dilate(binary,bg,Mat(),Point(-1,-1),6);
    threshold(bg,bg,1,128,THRESH_BINARY_INV);
    // 객체 없는 영상 화소 식별
    imshow("Background Image",bg);
    // 결과인 검은 화소는 배경 화소와 일치
    // 255 레이블인 전경화소와 128 레이블인 배경화소를 마크
    // 팽창 시 128 값을 화소에 할당한 후 즉시 경계화 작업
    
    // 영상을 마커 영상과 조합
    Mat markers(binary.size(),CV_8U,Scalar(0));
    markers= fg+bg; // 마커 영상 생성
    // 영상 조합시 오버로드 된 operator+ 사용
    imshow("Markers",markers);
    // 워터쉐드 알고리즘에 입력하기 위해 사용하는 입력 영상
    
    // 영상 분할
    WatershedSegmenter segmenter; // 웨터쉐드 분할 객체 생성
    
    segmenter.setMarkers(markers);
    segmenter.process(image);
    // 마커를 설정한 후 처리
    
    // 발견된 경계선에 속하는 화소값이 -1이면
    // 각 제로 화소를 입력 레이블 중의 하나에 할당하는 방법으로 마커 영상을 갱신
    
    imshow("Segmentation",segmenter.getSegmentation());
    
    imshow("Watersheds",segmenter.getWatersheds());
    
    waitKey(0);
}

void test_final(){
    Mat org_image = imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/image55.jpeg");
    Mat bak_org_image=org_image.clone();
    Mat bak_org_image2=org_image.clone();
    int image_height=org_image.rows;
    int image_width=org_image.cols;
    
    imshow("Original Image", org_image);
    
    //침식 & 팽창 과정을 통해 흰색격자선을 뭉개서 검정배경에 신체형태만 남도록 처리
    erode(org_image, org_image, Mat(),Point(-1,-1),6);
    dilate(org_image, org_image, Mat(),Point(-1,-1),6);
    
    imshow("Erode & Dilate Image",org_image);
    
    //그랩컷 알고리즘을 이용해 전경(신체)과 배경(검정배경) 분할
    //Rect grabcut_image_range(0, 0, image_width-1, image_height-1);
    Rect grabcut_image_range(30, 30, image_width-31, image_height-31);
    
    Mat grabcut_result_image;
    Mat grabcut_background_model, grabcut_foreground_model;
    
    grabCut(org_image, grabcut_result_image, grabcut_image_range, grabcut_background_model, grabcut_foreground_model, 5, GC_INIT_WITH_RECT);
    
    compare(grabcut_result_image, GC_PR_FGD, grabcut_result_image, CMP_EQ);
    
    Mat grabcut_foreground_image(org_image.size(), CV_8UC3, Scalar(255, 255, 255));
    org_image.copyTo(grabcut_foreground_image, grabcut_result_image);
    
    imshow("GrabCut Result Image",grabcut_result_image);
    imshow("GrabCut Foreground Image",grabcut_foreground_image);
    
    //그랩컷 알고리즘을 적용시켜 얻은 결과이미지로 신체의 외곽선 추출
    vector<vector<Point>> grabcut_contours;
    
    findContours(grabcut_result_image, grabcut_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    vector<vector<Point>>::const_iterator itc=grabcut_contours.begin();
    
    Mat result(grabcut_result_image.size(),CV_8U,Scalar(255));

    vector<Point> grabcut_contour=grabcut_contours[0];
    
    //추출된 외곽선들 중 가장 길이가 긴 외곽선 선택
    while(itc!=grabcut_contours.end()){
        if(grabcut_contour.size()<(*itc).size()) grabcut_contour=*itc;
        itc++;
    }
    
    /*vector<Point> grabcut_result_point;
    Point p1,p2,p3;
    
    if(grabcut_contour.size()<=3) return;
    else{
        p1=grabcut_contour[0];
        p2=grabcut_contour[1];
        grabcut_result_point.push_back(p1);
    }
    
    for(int i=2; i<grabcut_contour.size(); i++){
        p3=grabcut_contour[i];
        
        if(ccw(p1,p2,p3,15)){
            p2=p3;
        }
        else{
            grabcut_result_point.push_back(p2);
            p1=p2;
            p2=p3;
        }
    }*/
    
    //신체 외곽점 단순화
    vector<Point> grabcut_result_point=point_proc(grabcut_contour,15,1);
    //vector<Point> grabcut_result_point=grabcut_contour;
    //단순화 과정 반복
    grabcut_result_point=point_proc(grabcut_result_point,5,2);
    
    grabcut_result_point=grabcut_contour;
    
    //단순화된 신체외곽점 이미지에 표시
    vector<Point>::const_iterator itp=grabcut_contour.begin();
    while(itp!=grabcut_contour.end()){
        circle(grabcut_result_image, *itp, 5, Scalar(128));
        //circle(bak_org_image, *itp, 5, Scalar(0,255,0));
        itp++;
    }
    
    //신체 특징점을 찾는 과정
    //목양끝점(neck_right/left_point), 어깨양끝점(shoulder_right/left_point) 순으로 추출
    //가슴둘레, 배둘레도 처리과정을 추가하면 추출 가능
    
    //목양끝점 추출
    int miny_point_index=0;
    int grabcut_result_point_size=(int)grabcut_result_point.size();
    for(int i=0; i<grabcut_result_point.size(); i++) if(grabcut_result_point[miny_point_index].y>grabcut_result_point[i].y) miny_point_index=i;
    
    int tmp=miny_point_index+1;
    int neck_right_point=-1;
    int neck_left_point=-1;
    while(tmp!=miny_point_index-2){
        int p1_index=(tmp>=grabcut_result_point_size)?tmp-grabcut_result_point_size:tmp;
        int p2_index=((tmp+1)>=grabcut_result_point_size)?tmp-grabcut_result_point_size+1:tmp+1;
        int p3_index=((tmp+2)>=grabcut_result_point_size)?tmp-grabcut_result_point_size+2:tmp+2;
        if(_ccw(grabcut_result_point[p1_index],grabcut_result_point[p2_index],grabcut_result_point[p3_index])>0){
            //drawExLine(bak_org_image,grabcut_result_point[p1_index],grabcut_result_point[p2_index],Scalar(255,0,0),2);
            //drawExLine(bak_org_image,grabcut_result_point[p2_index],grabcut_result_point[p3_index],Scalar(255,0,0),2);
            //circle(bak_org_image, grabcut_result_point[p2_index], 3, Scalar(0,255,0),3);
            neck_right_point=p2_index;
            break;
        }
        if(++tmp>=grabcut_result_point_size) tmp=0;
    }
    
    tmp=miny_point_index-1;
    while(tmp!=miny_point_index-2){
        int p1_index=(tmp<0)?grabcut_result_point_size+tmp:tmp;
        int p2_index=((tmp-1)<0)?tmp-grabcut_result_point_size+tmp-1:tmp-1;
        int p3_index=((tmp-2)<0)?tmp-grabcut_result_point_size+tmp-2:tmp-2;
        if(_ccw(grabcut_result_point[p1_index],grabcut_result_point[p2_index],grabcut_result_point[p3_index])<0){
            //drawExLine(bak_org_image,grabcut_result_point[p1_index],grabcut_result_point[p2_index],Scalar(255,0,0),2);
            //drawExLine(bak_org_image,grabcut_result_point[p2_index],grabcut_result_point[p3_index],Scalar(255,0,0),2);
            //circle(bak_org_image, grabcut_result_point[p2_index], 3, Scalar(0,255,0),3);
            neck_left_point=p2_index;
            break;
        }
        if(--tmp<0) tmp=grabcut_result_point_size;
    }
    
    if(neck_right_point!=-1 && neck_left_point!=-1) line(bak_org_image, grabcut_result_point[neck_left_point], grabcut_result_point[neck_right_point], Scalar(0,0,255),4);
    
    //어깨 양끝점 추출
    tmp=neck_right_point;
    int shoulder_right_point=-1;
    int shoulder_left_point=-1;
    while(tmp!=miny_point_index-2){
        int p1_index=(tmp>=grabcut_result_point_size)?tmp-grabcut_result_point_size:tmp;
        int p2_index=((tmp+1)>=grabcut_result_point_size)?tmp-grabcut_result_point_size+1:tmp+1;
        int p3_index=((tmp+2)>=grabcut_result_point_size)?tmp-grabcut_result_point_size+2:tmp+2;
        if(_ccw(grabcut_result_point[p1_index],grabcut_result_point[p2_index],grabcut_result_point[p3_index])<0){
            drawExLine(bak_org_image,grabcut_result_point[p1_index],grabcut_result_point[p2_index],Scalar(255,0,255),2);
            drawExLine(bak_org_image,grabcut_result_point[p2_index],grabcut_result_point[p3_index],Scalar(255,0,255),2);
            circle(bak_org_image, grabcut_result_point[p2_index], 3, Scalar(0,255,0),3);
            shoulder_right_point=p2_index;
            break;
        }
        if(++tmp>=grabcut_result_point_size) tmp=0;
    }
    
    tmp=neck_left_point;
    while(tmp!=miny_point_index-2){
        int p1_index=(tmp<0)?grabcut_result_point_size+tmp:tmp;
        int p2_index=((tmp-1)<0)?tmp-grabcut_result_point_size+tmp-1:tmp-1;
        int p3_index=((tmp-2)<0)?tmp-grabcut_result_point_size+tmp-2:tmp-2;
        if(_ccw(grabcut_result_point[p1_index],grabcut_result_point[p2_index],grabcut_result_point[p3_index])>0){
            drawExLine(bak_org_image,grabcut_result_point[p1_index],grabcut_result_point[p2_index],Scalar(255,0,255),2);
            drawExLine(bak_org_image,grabcut_result_point[p2_index],grabcut_result_point[p3_index],Scalar(255,0,255),2);
            circle(bak_org_image, grabcut_result_point[p2_index], 3, Scalar(0,255,0),3);
            shoulder_left_point=p2_index;
            break;
        }
        if(--tmp<0) tmp=grabcut_result_point_size;
    }
    
    if(shoulder_right_point!=-1 && shoulder_left_point!=-1) line(bak_org_image, grabcut_result_point[shoulder_left_point], grabcut_result_point[shoulder_right_point], Scalar(0,0,255),4);
    
    //추출한 어깨 양끝점과 목선 어깨선 이미지에 표시
    itp=grabcut_result_point.begin();
    Point tmp2(-1,-1);
    
    while(itp!=grabcut_result_point.end()){
        if(tmp2.x==-1 && tmp2.y==-1) tmp2=*itp;
        else{
            line(result, tmp2, *itp, Scalar(200),2);
            line(bak_org_image2, tmp2, *itp, Scalar(0,0,255),2);
        }
        circle(result, *itp, 3, Scalar(50));
        circle(bak_org_image2, *itp, 5, Scalar(0,255,0));
        tmp2=*itp;
        itp++;
    }
    line(result, tmp2, grabcut_result_point[0], Scalar(200),2);
    line(bak_org_image2, tmp2, grabcut_result_point[0], Scalar(0,0,255),2);
    
    imshow("Contours Result Image",result);
    imshow("Contours Original Image",bak_org_image2);
    imshow("Find Shoulder Point Image",bak_org_image);
    
    waitKey(0);
}

void Harris()
{
    Mat org_image = imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/res.jpeg");
    Mat bak_org_image = org_image.clone();
    Mat bak_org_image2 = org_image.clone();
    Mat gray;
    
    int image_height = org_image.rows;
    int image_width = org_image.cols;
    
    imshow("Original Image", org_image);
    
    //침식 & 팽창 과정을 통해 흰색격자선을 뭉개서 검정배경에 신체형태만 남도록 처리
    erode(org_image, org_image, Mat(), Point(-1, -1), 6);
    dilate(org_image, org_image, Mat(), Point(-1, -1), 6);
    
    imshow("Erode & Dilate Image", org_image);
    
    //그랩컷 알고리즘을 이용해 전경(신체)과 배경(검정배경) 분할
    Rect grabcut_image_range(30, 30, image_width - 31, image_height - 31);
    
    Mat grabcut_result_image;
    Mat grabcut_background_model, grabcut_foreground_model;
    
    grabCut(org_image, grabcut_result_image, grabcut_image_range, grabcut_background_model, grabcut_foreground_model, 5, GC_INIT_WITH_RECT);
    
    compare(grabcut_result_image, GC_PR_FGD, grabcut_result_image, CMP_EQ);
    
    Mat grabcut_foreground_image(org_image.size(), CV_8UC3, Scalar(255, 255, 255));
    org_image.copyTo(grabcut_foreground_image, grabcut_result_image);
    
    imshow("GrabCut Result Image", grabcut_result_image);
    imshow("GrabCut Foreground Image", grabcut_foreground_image);
    
    cvNamedWindow("lueseypid", CV_WINDOW_AUTOSIZE);
    //IplImage* src = cvLoadImage("result2.bmp", CV_LOAD_IMAGE_GRAYSCALE);
    cvtColor(grabcut_foreground_image, grabcut_foreground_image, CV_RGB2GRAY);
    //IplImage* plmg = &IplImage(grabcut_foreground_image);
    IplImage* displayImg = cvLoadImage("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/res.jpeg", CV_WINDOW_AUTOSIZE);
    
    //IplImage* eig_image = cvCreateImage(cvGetSize(plmg), IPL_DEPTH_32F, 1);
    //IplImage* temp_image = cvCreateImage(cvGetSize(plmg), IPL_DEPTH_32F, 1);
    Mat eig_image = Mat(grabcut_foreground_image.cols, grabcut_foreground_image.rows, 0);
    Mat temp_image = Mat(grabcut_foreground_image.cols, grabcut_foreground_image.rows, 0);
    //Mat image = Mat(size*rat_cnt + gap*(rat_cnt + 1), size*rat_cnt + gap*(rat_cnt + 1), 0);
    vector<Point> corners;
    
    int corner_count = 100;
    //goodFeaturesToTrack(grabcut_foreground_image, eig_image, temp_image, corners, &corner_count,0.05, 5.0, 0, 3, 0, 0.04);
    goodFeaturesToTrack(grabcut_foreground_image, corners, corner_count, 0.05, 5.0);
    //cvGoodFeaturesToTrack()
    
    for (int i = 0; i<corner_count; i++)
    {
        printf("{%f, %f}\n", corners[i].x, corners[i].y);
        cvCircle(displayImg, cvPoint(corners[i].x, corners[i].y), 5, CV_RGB(255, 0, 0), -1, 8);
    }
    
    cvShowImage("lueseypid", displayImg);
    cvWaitKey(0);
    //cvReleaseImage(&plmg);
    cvDestroyWindow("lueseypid");
}

void face_detection()
{
    VideoCapture capture(0);
    if( !capture.isOpened() )
    {
        cerr << "Could not open camera" << std::endl;
        return;
    }
    // create a window
    namedWindow("webcam",1);
    // ------------------------------------------------------------------------- // face detection configuration
    CascadeClassifier face_classifier;
    face_classifier.load("/Users/pjh/Documents/opencv_test/opencv_test/opencv-3.2.0/data/haarcascades/haarcascade_upperbody.xml");
    while (true)
    {
        bool frame_valid = true;
        Mat frame_original;
        Mat frame;
        try {
            capture >> frame_original; // get a new frame from webcam
            resize(frame_original,frame,cv::Size(frame_original.cols/2, frame_original.rows/2),0,0,CV_INTER_NN);
            // downsample 1/2x
        } catch(cv::Exception& e) {
            std::cerr << "Exception occurred. Ignoring frame... " << e.err << std::endl;
            frame_valid = false;
        }
        if (frame_valid) {
            try {
                // convert captured frame to gray scale & equalize
                Mat grayframe;
                cvtColor(frame, grayframe, CV_BGR2GRAY);
                equalizeHist(grayframe,grayframe);
                // ------------------------------------------------------------- // face detection routine // a vector array to store the face found
                vector<cv::Rect> faces;
                
                face_classifier.detectMultiScale(grayframe, faces,
                                                 1.1, // increase search scale by 10% each pass
                                                 3, // merge groups of three detections
                                                 CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, cv::Size(30,30));
                
                // -------------------------------------------------------------
                // draw the results
                for(int i=0; i<faces.size(); i++) {
                    Point lb(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
                    Point tr(faces[i].x, faces[i].y);
                    rectangle(frame, lb, tr, cv::Scalar(0,255,0), 3, 4, 0);
                }
                // print the output
                imshow("webcam", frame);
            } catch(cv::Exception& e) {
                std::cerr << "Exception occurred. Ignoring frame... " << e.err << std::endl;
            }
        }
        waitKey(0);
    }
    // VideoCapture automatically deallocate camera object
    return;
}

void Grabcut_Sample(){
    Mat org_image = imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/res.jpeg");
    Mat bak_org_image=org_image.clone();
    Mat bak_org_image2=org_image.clone();
    int image_height=org_image.rows;
    int image_width=org_image.cols;
    
    imshow("Original Image", org_image);
    
    //침식 & 팽창 과정을 통해 흰색격자선을 뭉개서 검정배경에 신체형태만 남도록 처리
    erode(org_image, org_image, Mat(),Point(-1,-1),6);
    dilate(org_image, org_image, Mat(),Point(-1,-1),6);
    
    imshow("Erode & Dilate Image",org_image);
    
    //그랩컷 알고리즘을 이용해 전경(신체)과 배경(검정배경) 분할
    //Rect grabcut_image_range(0, 0, image_width-1, image_height-1);
    Rect grabcut_image_range(30, 30, image_width-31, image_height-31);
    
    Mat grabcut_result_image;
    Mat grabcut_background_model, grabcut_foreground_model;
    
    grabCut(org_image, grabcut_result_image, grabcut_image_range, grabcut_background_model, grabcut_foreground_model, 5, GC_INIT_WITH_RECT);
    
    compare(grabcut_result_image, GC_PR_FGD, grabcut_result_image, CMP_EQ);
    
    Mat grabcut_foreground_image(org_image.size(), CV_8UC3, Scalar(255, 255, 255));
    org_image.copyTo(grabcut_foreground_image, grabcut_result_image);
    
    imshow("GrabCut Result Image",grabcut_result_image);
    imshow("GrabCut Foreground Image",grabcut_foreground_image);
    
    //그랩컷 알고리즘을 적용시켜 얻은 결과이미지로 신체의 외곽선 추출
    vector<vector<Point>> grabcut_contours;
    
    findContours(grabcut_result_image, grabcut_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    vector<vector<Point>>::const_iterator itc=grabcut_contours.begin();
    
    Mat result(grabcut_result_image.size(),CV_8U,Scalar(255));
    
    vector<Point> grabcut_contour=grabcut_contours[0];
    
    //추출된 외곽선들 중 가장 길이가 긴 외곽선 선택
    while(itc!=grabcut_contours.end()){
        if(grabcut_contour.size()<(*itc).size()) grabcut_contour=*itc;
        itc++;
    }
    
    waitKey(0);
}

void Watershed_Sample(){
    Mat image= imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/result.bmp");
    imshow("Original Image",image);
    
    Mat binary;
    Mat th_image;
    
    cvtColor(image, binary, CV_RGB2GRAY);
    
    threshold(binary, th_image, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    
    imshow("binary",binary);
    
    // 이진 영상은 영상의 여러 부분에 속하는 흰색 화소를 많이 포함
    // 중요한 객체에 속하는 화소만 남기기 위해 영상을 여러 번 침식
    Mat fg;
    erode(binary, fg, Mat(), Point(-1, -1), 6);
    // 잡음과 작은 객체 제거
    imshow("Foreground Image", fg);
    // 배경인 숲에 속하는 몇 화소는 여전히 존재
    // 관심 객체에 대응하는 부분을 고려
    
    // 원 이진 영상의 큰 팽창인 배경의 화소를 선택
    Mat bg;
    dilate(binary,bg,Mat(),Point(-1,-1),6);
    threshold(bg,bg,1,128,THRESH_BINARY_INV);
    // 객체 없는 영상 화소 식별
    imshow("Background Image",bg);
    // 결과인 검은 화소는 배경 화소와 일치
    // 255 레이블인 전경화소와 128 레이블인 배경화소를 마크
    // 팽창 시 128 값을 화소에 할당한 후 즉시 경계화 작업
    
    // 영상을 마커 영상과 조합
    Mat markers(binary.size(),CV_8U,Scalar(0));
    markers= fg+bg; // 마커 영상 생성
    // 영상 조합시 오버로드 된 operator+ 사용
    imshow("Markers",markers);
    // 워터쉐드 알고리즘에 입력하기 위해 사용하는 입력 영상
    
    // 영상 분할
    WatershedSegmenter segmenter; // 웨터쉐드 분할 객체 생성
    
    segmenter.setMarkers(markers);
    segmenter.process(image);
    // 마커를 설정한 후 처리
    
    // 발견된 경계선에 속하는 화소값이 -1이면
    // 각 제로 화소를 입력 레이블 중의 하나에 할당하는 방법으로 마커 영상을 갱신
    
    imshow("Segmentation",segmenter.getSegmentation());
    
    imshow("Watersheds",segmenter.getWatersheds());
    
    waitKey(0);
}

void dddd(){
    Mat org_image = imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/image55.jpeg");
    Mat bak_org_image=org_image.clone();
    Mat bak_org_image2=org_image.clone();
    int image_height=org_image.rows;
    int image_width=org_image.cols;
    
    cout<<image_width<<" , "<<image_height<<endl;
    
    imshow("Original Image", org_image);
    
    //침식 & 팽창 과정을 통해 흰색격자선을 뭉개서 검정배경에 신체형태만 남도록 처리
    erode(org_image, org_image, Mat(),Point(-1,-1),6);
    dilate(org_image, org_image, Mat(),Point(-1,-1),6);
    
    imshow("Erode & Dilate Image",org_image);
    
    //그랩컷 알고리즘을 이용해 전경(신체)과 배경(검정배경) 분할
    //Rect grabcut_image_range(0, 0, image_width-1, image_height-1);
    Rect grabcut_image_range(30, 30, image_width-31, image_height-31);
    
    Mat grabcut_result_image;
    Mat grabcut_background_model, grabcut_foreground_model;
    
    grabCut(org_image, grabcut_result_image, grabcut_image_range, grabcut_background_model, grabcut_foreground_model, 5, GC_INIT_WITH_RECT);
    
    compare(grabcut_result_image, GC_PR_FGD, grabcut_result_image, CMP_EQ);
    
    Mat grabcut_foreground_image(org_image.size(), CV_8UC3, Scalar(255, 255, 255));
    org_image.copyTo(grabcut_foreground_image, grabcut_result_image);
    
    imshow("GrabCut Result Image",grabcut_result_image);
    imshow("GrabCut Foreground Image",grabcut_foreground_image);
    
    //그랩컷 알고리즘을 적용시켜 얻은 결과이미지로 신체의 외곽선 추출
    vector<vector<Point>> grabcut_contours;
    
    findContours(grabcut_result_image, grabcut_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    vector<vector<Point>>::const_iterator itc=grabcut_contours.begin();
    
    Mat result(grabcut_result_image.size(),CV_8U,Scalar(255));
    
    vector<Point> grabcut_contour=grabcut_contours[0];
    
    //추출된 외곽선들 중 가장 길이가 긴 외곽선 선택
    while(itc!=grabcut_contours.end()){
        if(grabcut_contour.size()<(*itc).size()) grabcut_contour=*itc;
        itc++;
    }
    
    int contour_size=grabcut_contour.size();
    cout<<contour_size<<endl;
    
    //circle(grabcut_result_image, grabcut_contour[0], 5, Scalar(128));
    
    Mat tmp_img1=grabcut_result_image.clone();
    Mat tmp_img2=grabcut_result_image.clone();
    Mat tmp_img3=grabcut_result_image.clone();
    Mat tmp_img4=grabcut_result_image.clone();
    
    /*
     목 100
     어깨 150
     손1 400
     손2 420
     겨  570
     680
     
     820
     겨 930
     손2 1080
     손1 1100
     어깨 1340
     목 1400
     */
    
    //circle(bak_org_image,grabcut_contour[100],5,Scalar(255,0,0));
    //circle(bak_org_image,grabcut_contour[150],5,Scalar(255,0,0));
    new_line nline[10];
    nline[0]=drawRansac(bak_org_image,grabcut_contour,100,150,Scalar(255,0,0));
    nline[1]=drawRansac(bak_org_image,grabcut_contour,150,400,Scalar(0,255,0));
    nline[2]=drawRansac(bak_org_image,grabcut_contour,400,420,Scalar(0,0,255));
    nline[3]=drawRansac(bak_org_image,grabcut_contour,420,570,Scalar(255,255,0));
    nline[4]=drawRansac(bak_org_image,grabcut_contour,570,680,Scalar(0,255,255));
    
    
    nline[5]=drawRansac(bak_org_image,grabcut_contour,820,930,Scalar(0,255,255));
    nline[6]=drawRansac(bak_org_image,grabcut_contour,930,1080,Scalar(255,255,0));
    nline[7]=drawRansac(bak_org_image,grabcut_contour,1080,1100,Scalar(0,0,255));
    nline[8]=drawRansac(bak_org_image,grabcut_contour,1100,1340,Scalar(0,255,0));
    nline[9]=drawRansac(bak_org_image,grabcut_contour,1340,1400,Scalar(255,0,0));
    
    drawInterPoint(bak_org_image, nline[0], nline[1], Scalar(255,0,0));
    drawInterPoint(bak_org_image, nline[3], nline[4], Scalar(255,0,0));
    
    drawInterPoint(bak_org_image, nline[5], nline[6], Scalar(255,0,0));
    drawInterPoint(bak_org_image, nline[8], nline[9], Scalar(255,0,0));
    
    imshow("asdf", bak_org_image);
    
    waitKey(0);
}

void ransac_test(){
    Mat org_image = imread("/Users/pjh/Documents/opencv_test/opencv_test/opencv_test/res.jpeg");
    int image_width=org_image.cols;
    int image_height=org_image.rows;
    
    int no_data = 329;
    new_point *data = new new_point[no_data];
    
    // 759/329
    for (int i=0; i<no_data; ++i) {
        data[i].x=(i+1);
        data[i].y=(i+1)*758/328;
        data[i].y+=(data[i].y<50)?rand()%51:-50+rand()%101;
    }
    
    for(int i = 0; i<no_data; i++) {
        circle(org_image, Point(data[i].x,data[i].y), 3, Scalar(255,0,0));
        //pDC->SetPixel((int)data[i].x, (int)data[i].y, RGB(255,0,0));
    }
    
    new_line res_line;
    double cost = ransac_line_fitting (data, no_data, res_line, 30);

    new_point p1;
    p1.x=(int)(res_line.sp.x-500*res_line.mp.x);
    p1.y=(int)(res_line.sp.y-500*res_line.mp.y);
    
    new_point p2;
    p2.x=(int)(res_line.sp.x+500*res_line.mp.x);
    p2.y=(int)(res_line.sp.y+500*res_line.mp.y);
    
    if (100. < cost) {
        line(org_image, Point(p1.x,p1.y),Point(p2.x,p2.y), Scalar(0,255,0));
        //pDC->MoveTo((int)(line.sx-500*line.mx), (int)(line.sy-500*line.my));
        //pDC->LineTo((int)(line.sx+500*line.mx), (int)(line.sy+500*line.my));
    }
    
    delete [] data;
    
    imshow("res",org_image);
    waitKey(0);
}

int main()
{
    //create_image();
    //test();
    //grabcut_test();
    //watershed_test();
    //test_final();
    //Harris();
    
    //Grabcut_Sample();
    //Watershed_Sample();
    
    dddd();
    
    //ransac_test();
    
    //face_detection();
    return 0;
}