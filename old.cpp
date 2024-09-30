#include "funcA.h"
#include "funcB.h"
//----------全局变量声明在.h中，在某一个源文件唯一定义-------
int zbera_count = 0; // 识别到斑马线的帧数
int rhombus_count = 0; // 识别到菱形标的帧数
bool IfChangeSide = false;
bool IF_STOP = false;
bool IF_SLOW = false;
//-------------------------------------------------------------
#pragma once
#define LOWERB_R1 Scalar(0, 43, 46)
#define UPPERB_R1 Scalar(10, 255, 255)
#define LOWERB_R2 Scalar(156, 43, 46)
#define UPPERB_R2 Scalar(180, 255, 255)
#define LEFT false;
#define RIGHT true;

#include "opencv2/opencv.hpp"
#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>

using namespace cv;
using namespace std;

// --------定义|各状态的全局变量--------
extern int zbera_count; // 识别到斑马线的帧数
extern bool IfChangeSide; // 是否变道
extern bool IF_STOP; // 是否停止
extern bool IF_SLOW; // 是否减速
// -----------------------------------

//********基础通用模块*******

bool testImg(Mat img);

/* 已知斜率和直线上一点求截距
 *
 */
double getIntercept(double slope, cv::Point point);

/* 车道线函数
    @param frame: 要处理的帧
*/
int LaneLine(cv::Mat frame);

/* ROI区域提取
    @param inputFrame 输入图像
    @param points ROI区域的边界坐标点集

    @return ROI区域的Mat矩阵
*/
Mat ROI_extract(cv::Mat inputFrame, std::vector<cv::Point> points);
/* 两点间距离计算
    @param point0 起始点
    @param pointA 结束点

    @return distance 斜率
*/
double getDistance(cv::Point pointO, cv::Point pointA);
/* 绝对值计算
    @param input 输入值

    @return 输入值的绝对值
*/
double getAbs(double input);
/* 直线斜率计算
    @param pointA 直线上第一点
    @param pointB 直线上第二点

    @return slope 斜率
*/
double getSlope(cv::Point pointA, cv::Point pointB);

// 斑马线识别函数

void If_ZebraCrossing(cv::Mat frame);

class BasicFunc {
public:
    BasicFunc(bool CHOOSE_PATTERN)
        : PATTERN(CHOOSE_PATTERN)
    {
    }

    ~BasicFunc()
    {
    }
    bool PATTERN;
    // 添加图像帧到队列
    void AddFrame(const cv::Mat& frame)
    {
        if (!frame.empty()) {
            std::lock_guard<std::mutex> lock(image_mutex);
            image_queue.push(frame);
            frame_available.notify_one();
        }
    }
    void always(BasicFunc& basicFunc); // 摄像头版本

    std::atomic<bool> exit_flag;
    std::queue<cv::Mat> image_queue;
    std::mutex image_mutex;
    std::condition_variable frame_available;
};

#ifndef FUNC_A
#define FUNC_A
//*******************************************************************************
//**-------------------------------超车项目的函数头文件----------------------------**
//********************************************************************************
class AFunc {
public:
    AFunc()
        : capture()
    {
        try {
            capture.open(1);
            if (!capture.isOpened()) {
                throw std::runtime_error("Failed to open camera");
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }
    void processFunc();

private:
    VideoCapture capture;
};
#endif
#ifndef FUNC_B
#define FUNC_B
//*******************************************************************************
//**-------------------------------路标项目的函数头文件----------------------------**
//********************************************************************************

//*****锥桶识别相关******
void imgPreTreat(Mat& imgOriginal, Mat& imgThresholded, InputArray LOWERB, InputArray UPPERB);
bool RedBarrelFind(Mat& img, Mat& imgFind);
bool findBarrel(Mat image01);

//*****菱形识别*****
void If_Rhombus(cv::Mat frame);
Mat If_Rhombus_DEBUG(cv::Mat frame, cv::Mat draw);

// 摄像头版本
class BFunc {
public:
    BFunc()
        :
    {
    }
    void processFunc(BasicFunc& basicFunc); // 将BasicFunc作为参数传递
};
#endif
//********************************************************************************
//**-------------------------------通用功能函数-----------------------------------**
//********************************************************************************
bool testImg(Mat img)
{
    if (img.empty())
        return false;
    return true;
}

/** 串口通信函数
 *  @param DEVIATION
 *  @param TURN_DIRECTION (0不转弯  1左转  2右转)
 *  @param FLAG_SLOW (0不减速 1减速)
 *  @param FLAG_STOP (0不停止 1停止)
 *  @retval 1 或 0
 */
int uart_send(int DEVIATION, int TURN_DIRECTION, int FLAG_SLOW, int FLAG_STOP)
{
    // 初始化python接口
    Py_Initialize();
    if (!Py_IsInitialized()) {
        std::cout << "python init fail" << std::endl;
        return 0;
    }
    // 初始化使用的变量
    PyObject* pModule = NULL;
    PyObject* pFunc = NULL;
    PyObject* pName = NULL;

    // 初始化python系统文件路径，保证可以访问到 .py文件
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('..')");

    // 调用python文件名。
    pModule = PyImport_ImportModule("uart");

    // 调用函数
    pFunc = PyObject_GetAttrString(pModule, "uart_send");

    // 给python传参数
    PyObject* pArgs = PyTuple_New(4);

    // 0：第一个参数，传入 float 类型的值 2.242
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("i", DEVIATION));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", TURN_DIRECTION));
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("i", FLAG_SLOW));
    PyTuple_SetItem(pArgs, 3, Py_BuildValue("i", FLAG_STOP));

    // 使用C++的python接口调用该函数
    PyObject* pReturn = PyEval_CallObject(pFunc, pArgs);

    // 结束python接口初始化
    Py_Finalize();
    return 1;
}

double getAbs(double input)
{
    return ((input > 0) ? (input) : (-input));
}
double getSlope(cv::Point pointA, cv::Point pointB)
{
    double y_abs = pointA.y - pointB.y;
    double x_abs = pointA.x - pointB.x;
    if (x_abs == 0)
        x_abs += 0.00001; // 防止两点重合导致分母为0
    double slope = y_abs / x_abs;
    return slope;
}
Mat ROI_extract(cv::Mat inputFrame, std::vector<cv::Point> points)
{
    cv::Mat dst, src;
    src = inputFrame;
    cv::Mat ROI = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours; // 轮廓
    std::vector<cv::Point> pts;

    for (int i = 0; i < points.size(); i++) {
        pts.push_back(points[i]);
    }

    contours.push_back(pts);
    drawContours(ROI, contours, 0, cv::Scalar(255), -1); // 用白色填充多边形区域
    src.copyTo(dst, ROI); // 掩码运算

    return dst;
}
double getDistance(cv::Point pointO, cv::Point pointA)
{
    double distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);

    return distance;
}

/* 摄像头版本下的处理流程
 *
 */
void BasicFunc::always(BasicFunc& basicFunc)
{
    if (PATTERN) {
        BFunc carB;
        carB.processFunc(basicFunc);
    } else {
        AFunc carA;
        carA.processFunc();
    }
}

/* 已知斜率和直线上一点求截距
 *
 */
double getIntercept(double slope, cv::Point point)
{
    return (point.y - point.x * slope);
}

/* 车道线函数
    @param frame: 要处理的帧
*/
int LaneLine(cv::Mat frame)
{
    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element_erode, element_dilate, dilate;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point(max_X * (0.0 / 5.0), max_Y * (5.0 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (5.0 / 5.0), max_Y * (5.0 / 5.0))); // RD
    points.push_back(cv::Point(max_X * (5.0 / 5.0), max_Y * (3.0 / 5.0))); // RU
    points.push_back(cv::Point(max_X * (0.0 / 5.0), max_Y * (3.0 / 5.0))); // LU
    ROI = ROI_extract(frame, points);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 170, 255, cv::THRESH_BINARY);

    cv::Canny(thres, canny, 100, 200, 3);
    /* 识别前图像处理结束 */

    std::vector<cv::Vec4f> lines;
    // 创建一个包含直线斜率和截距的特征向量的数据集
    std::vector<std::vector<cv::Point2d>> features;
    std::vector<cv::Point2d> left;
    std::vector<cv::Point2d> right;
    features.push_back(left);
    features.push_back(right);
    cv::HoughLinesP(canny, lines, 1., CV_PI / 180, 100, 50, 30);
    for (int i = 0; i < lines.size(); ++i) {
        cv::Vec4i line_ = lines[i];
        cv::Point pointA = cv::Point(line_[0], line_[1]), pointB = cv::Point(line_[2], line_[3]);
        double slope = getSlope(pointA, pointB);
        double intercept = getIntercept(slope, pointA);
        if (getAbs(slope) >= 0.5 && getAbs(slope) < 500000) {
            if (slope > 0) {
                features[0].push_back(cv::Point2d(slope, intercept));
            } else {
                features[1].push_back(cv::Point2d(slope, intercept));
            }
        }
    }

    if (features[0].size() != 0 && features[1].size() != 0) {
        std::vector<cv::Point2d> lanelines;
        for (int i = 0; i < features.size(); i++) {
            double slope = 0, intercept = 0;

            for (int j = 0; j < features[i].size(); j++) {
                slope += features[i][j].x;
                intercept += features[i][j].y;
            }

            lanelines.push_back(cv::Point2d((slope / features[i].size()), (intercept / features[i].size())));
        }
        std::vector<int> center;
        for (int x = 0; x < max_X; x++) {
            if (getAbs((x * lanelines[0].x + lanelines[0].y) - (x * lanelines[1].x + lanelines[1].y)) <= 10) {
                center.push_back(x);
            }
        }
        double center_x = 0;
        for (int i = 0; i < center.size(); i++) {
            center_x += center[i];
        }
        center_x = center_x / center.size();

        int Deviation = (int)((double)center_x - (double)max_X / 2.0);

        return Deviation;
    } else {
        return 0;
    }
}

//******斑马线识别*****    TIPS：好像超车模式不需要斑马线前停车，只需要识别终点线？
void If_ZebraCrossing(cv::Mat frame)
{

    /**** 识别前 图像处理 ****/
    cv::Mat gray, ROI, bulr, thres, canny, element, dilate;
    bool Is_approach = false;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point(max_X * (0.01 / 5.0), max_Y * (4.95 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (4.99 / 5.0), max_Y * (4.95 / 5.0))); // RD
    points.push_back(cv::Point(max_X * (4.85 / 5.0), max_Y * (3.0 / 5.0))); // RU
    points.push_back(cv::Point(max_X * (0.15 / 5.0), max_Y * (3.0 / 5.0))); // LU
    ROI = ROI_extract(frame, points);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY);
    // 腐蚀处理(防止斑马线与停止线粘连)
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(thres, dilate, element);
    /**** 识别前图像处理 结束 ****/

    /**** 识别斑马线块 ****/
    // 定义-存储轮廓
    std::vector<std::vector<cv::Point>> contours;
    // 定义-存储轮廓层级信息
    std::vector<cv::Vec4i> hierachy;
    // 查找轮廓
    cv::findContours(dilate, contours, hierachy, cv::RETR_TREE,
        cv::CHAIN_APPROX_NONE);
    // 定义-存储满足条件的四边形角点集合
    std::vector<cv::Point> all_points;

    // 定义-斑马线白块的总面积
    double white_area = 0;
    // 定义-斑马线白块的补偿面积(应用: （凸包面积 + 补偿面积） 应等于 （斑马线白块的总面积 * 2） )
    double white_extern_area = 0;

    // 定义-最小轮廓面积
    int min_area = 1000;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > min_area) {
            // 定义-存储多边形逼近的角点
            std::vector<cv::Point2f> points;
            // 多边形逼近
            cv::approxPolyDP(contours[i], points, 10.0, true);

            // 筛选出四边形
            if (points.size() == 4) {

                // 按长宽比例筛选四边形

                // 定义-长宽比(利用问号表达式使其大于一)
                double len_to_wid_ratio = (getDistance(points[0], points[1]) >= getDistance(points[1], points[2])) ? (getDistance(points[0], points[1]) / getDistance(points[1], points[2])) : (getDistance(points[1], points[2]) / getDistance(points[0], points[1]));

                if (len_to_wid_ratio < 6) {
                    for (int j = 0; j < 4; j++) {
                        all_points.push_back(points[j]);
                    }
                    if (white_extern_area == 0) {
                        white_extern_area = cv::contourArea(contours[i]);
                    }
                    white_area += cv::contourArea(contours[i]);
                }
            }
        }
    }
    /**** 识别斑马线块 结束 ****/

    /**** 识别斑马线 ****/
    // 如果四边形数量大于3认为有可能识别到斑马线
    if (all_points.size() >= 12) {
        // 定义-凸包点集
        std::vector<cv::Point> hull_points;
        // 求凸包点集
        cv::convexHull(all_points, hull_points);
        // 求凸包面积
        double hull_area = cv::contourArea(hull_points);

        // 定义-最小外接矩形
        cv::RotatedRect box;
        cv::Point2f rect[4];
        // 计算每个轮廓最小外接矩形
        box = cv::minAreaRect(cv::Mat(all_points));
        // 把最小外接矩形四个端点复制给rect数组
        box.points(rect);
        // 求外接矩形长宽
        unsigned int width = (getDistance(rect[0], rect[1]) > getDistance(rect[1], rect[2])) ? (getDistance(rect[0], rect[1])) : (getDistance(rect[1], rect[2])),
                     length = (getDistance(rect[0], rect[1]) < getDistance(rect[1], rect[2])) ? (getDistance(rect[0], rect[1])) : (getDistance(rect[1], rect[2]));
        // 求上边斜率
        double up_slope = 0;
        if (getDistance(rect[0], rect[1]) > getDistance(rect[1], rect[2])) {
            up_slope = getAbs(getSlope(rect[0], rect[1]));
        } else {
            up_slope = getAbs(getSlope(rect[1], rect[2]));
        }
        // 求最靠近视频流底部的点
        double y_max = 0;
        for (int i = 0; i < 4; i++) {
            if (y_max < rect[i].y)
                y_max = rect[i].y;
        }

        // 定义斑马线总长宽比
        double zebra_ratio = 3;
        // 求置信度(凸包面积 + 补偿面积 ?= 斑马线块总面积*2.3)
        double confidence_level = ((hull_area + white_extern_area) <= (white_area * 2.3)) ? ((hull_area + white_extern_area) / (white_area * 2.5)) : ((white_area * 2.5) / (hull_area + white_extern_area));
        /* 最后的筛选：
            1.判断外接矩形长宽是否符合比例
            2.判断置信度是否大于0.8
            3.判断斑马线距离视频流底部距离是否小于500px
           如果满足：
            识别到斑马线的帧数（zbera_count）+1
        */
        if ((length * zebra_ratio < width) && (confidence_level >= 0.8) && (frame.size().height - y_max < 1000) && (up_slope <= 0.05)) {
            zbera_count++;
            Is_approach = true;
        }
    }
    /**** 识别斑马线 结束 ****/

    /* 判断是否停止：
        当 zbera_count > 5 且 斑马线距离视频流底部距离满足条件 时 输出停止信号
    */
    if ((zbera_count > 10) && Is_approach == true) {
        IF_STOP = true;
    }
}

//*******************************************************************************
//**-------------------------------超车项目的功能函数---------------------------**
//********************************************************************************
void AFunc::processFunc()
{
    Mat frame;
    int countBarre = 0;
    while (capture.read(frame)) {
        //****斑马线停止****
        If_ZebraCrossing(frame);
        if (IF_STOP)
            IF_SLOW = true;
        cboardA.SendA(true, LaneLine(frame), IF_SLOW, IF_STOP);

        if (fpsA.Count()) {
            std::cout << "Fps: " << fpsA.GetFPS() << '\n';
        }
    }
    std::cout << "endVideo" << std::endl;
    cv::destroyAllWindows(); // 视频播放结束，销毁窗口,否则报错
    capture.release();
}

//*******************************************************************************
//**-------------------------------路标项目的功能函数------------------------------**
//********************************************************************************

//*****锥桶识别的功能实现********
void imgPreTreat(Mat& imgOriginal, Mat& imgThresholded, InputArray LOWERB, InputArray UPPERB)
{
    vector<vector<Point>> contour;
    vector<Vec4i> hierarchy;
    vector<Point> hull;
    // ROI提取
    Mat ROI;
    std::vector<cv::Point> points;
    double max_X = imgOriginal.size().width;
    double max_Y = imgOriginal.size().height;
    points.push_back(cv::Point(max_X * (0.1 / 5.0), max_Y * (4.95 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (4.9 / 5.0), max_Y * (4.95 / 5.0))); // RD  4.9,4.95
    points.push_back(cv::Point(max_X * (4.0 / 5.0), max_Y * (1.8 / 5.0))); // RU
    points.push_back(cv::Point(max_X * (1.0 / 5.0), max_Y * (1.8 / 5.0))); // LU
    ROI = ROI_extract(imgOriginal, points);
    // imshow("ROI", ROI);
    // waitKey(1);//debug
    // 将图像转换成HSV格式
    Mat imgHSV;
    cvtColor(ROI, imgHSV, COLOR_BGR2HSV);

    // 直方图均衡化(三个通道各自均衡化再组合)
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);

    // 阈值处理
    inRange(imgHSV, LOWERB, UPPERB, imgThresholded);
    // 得到一个矩形卷积核
    Mat element = getStructuringElement(MORPH_RECT, Size(8, 8));
    // 闭操作，连接一些连通域
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
    dilate(imgThresholded, imgThresholded, 300 * 300, Point(-1, -1), 1);
    // 开操作，去除一些噪点
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    // 形状修复
    findContours(imgThresholded, contour, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_TC89_KCOS);
    for (int i = 0; i < contour.size(); i++) {
        // 凸包检测
        convexHull(contour[i], hull, false, true);
        // 凸包填充
        fillPoly(imgThresholded, hull, Scalar(255, 255, 255));
        // 闭操作，连接一些连通域
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
        dilate(imgThresholded, imgThresholded, 770 * 770, Point(-1, -1), 1);
    }

    return;
}
bool RedBarrelFind(Mat& img, Mat& imgFind)
{
    vector<vector<Point>> contour;
    vector<Vec4i> hierarchy;
    findContours(img, contour, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_TC89_KCOS);
    vector<Rect> boundRect(contour.size());
    vector<vector<Point>> conPoly(contour.size());
    bool ifFIND = false;
    // cout << "原始轮廓数：" << contour.size() <<endl;

    if (!contour.empty()) {

        vector<vector<Point>>::iterator it = contour.begin();
        for (int i = 0; i < contour.size(); i++) {
            float peri = arcLength(contour[i], true);
            approxPolyDP(contour[i], conPoly[i], 0.02 * peri, true); // 光滑曲线折线化
            boundRect[i] = boundingRect(conPoly[i]);

            // 判断是否为锥桶,不是则删掉轮廓
            float aspRatio = (float)boundRect[i].width / boundRect[i].height;
            int area = contourArea(contour[i]);
            // cout << "长宽比：" << aspRatio << endl;
            // cout << "面积：" << area << endl;
            if (aspRatio < 2.5 || (area > 1000 && area < 1400)) {
                it = contour.erase(it);
                i--;
            } else {
                ++it;
                rectangle(imgFind, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 255), 5);
            }
        }

        // cout << "筛除后轮廓个数：" << contour.size() << endl;
        if (contour.size()) {
            ifFIND = true;
            try {
                // 轮廓合并
                vector<Point> contours_merge = contour[0];
                for (int i = 1; i < contour.size(); i++) {
                    contours_merge.insert(contours_merge.end(), contour[i].begin(), contour[i].end());
                }

                RotatedRect rect2 = minAreaRect(contours_merge);
                Point2f box2[4];
                rect2.points(box2);
                for (int i = 0; i < 4; ++i) {
                    line(imgFind, box2[i], box2[(i + 1) % 4], Scalar(0, 255, 0), 2);
                }

            }

            catch (const cv::Exception& e) {
                std::cerr << "OpenCV Exception: " << e.what() << std::endl;
            }
        }
    }
    return ifFIND;
}
bool findBarrel(Mat image01)
{
    Mat image02;
    int countBarre = 0;
    if (testImg(image01))
        imgPreTreat(image01, image02, Scalar(156, 43, 46), Scalar(180, 255, 255));
    if (RedBarrelFind(image02, image01)) {
        return true;
    }
    return false;
}

/* 菱形标识别函数
    @param frame: 要处理的帧
*/
/* 菱形标识别函数
    @param frame: 要处理的帧
    @param draw: 要绘制轮廓的图像
*/
void If_Rhombus(cv::Mat frame)
{
    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point(max_X * (1.0 / 5.0), max_Y * (4.95 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (4.0 / 5.0), max_Y * (4.95 / 5.0))); // RD
    points.push_back(cv::Point(max_X * (3.0 / 5.0), max_Y * (3.0 / 5.0))); // RU
    points.push_back(cv::Point(max_X * (2.0 / 5.0), max_Y * (3.0 / 5.0))); // LU
    ROI = ROI_extract(frame, points);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY);
    // 膨胀处理
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    cv::dilate(thres, dilate, element);
    // Canny边缘检测
    cv::Canny(dilate, canny, 100, 200, 3);
    /* 识别前图像处理结束 */

    /**** 识别图像轮廓 ****/
    // 存储轮廓
    std::vector<std::vector<cv::Point>> contours;
    // 存储轮廓层级信息
    std::vector<cv::Vec4i> hierachy;
    // 查找轮廓
    cv::findContours(canny, contours, hierachy, cv::RETR_TREE,
        cv::CHAIN_APPROX_NONE);

    // 定义-存储满足条件的四边形角点集合
    std::vector<std::vector<cv::Point2f>> all_point_sets;

    int min_area = 2000;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > min_area) {
            // 多边形逼近
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            bool If_coincide = false;
            // 添加筛选条件防止点重合
            for (int i = 0; i < points.size(); i++) {
                if (getDistance(points[i], points[(i + 1) % 4]) <= 25)
                    If_coincide = true;
            }

            if (If_coincide == true)
                continue;

            // 筛选出四边形
            if (points.size() == 4) {
                // 定义-长宽比(利用问号表达式使其大于一)
                double len_to_wid_ratio = (getDistance(points[0], points[1]) >= getDistance(points[1], points[2])) ? (getDistance(points[0], points[1]) / getDistance(points[1], points[2])) : (getDistance(points[1], points[2]) / getDistance(points[0], points[1]));

                if (len_to_wid_ratio < 2) {
                    double diag_slope_1, diag_slope_2;
                    if ((getAbs(getSlope(points[0], points[2])) < 1) && (getAbs(getSlope(points[1], points[3])) > 1)) {
                        diag_slope_1 = getAbs(getSlope(points[0], points[2])),
                        diag_slope_2 = getAbs(getSlope(points[1], points[3]));
                    } else if ((getAbs(getSlope(points[0], points[2])) > 1) && (getAbs(getSlope(points[1], points[3])) < 1)) {
                        diag_slope_2 = getAbs(getSlope(points[0], points[2])),
                        diag_slope_1 = getAbs(getSlope(points[1], points[3]));
                    } else {
                        continue;
                    }

                    double product = diag_slope_1 * diag_slope_2;

                    if ((diag_slope_1 < 0.5) && (diag_slope_2 > 2.5)) {
                        rhombus_count++;
                    }
                }
            }
        }
    }

    /* 判断是否减速：
        当 rhombus_count > 5时 输出停止信号
    */
    if (rhombus_count > 15) {
        IF_SLOW = true;
    }
}

Mat If_Rhombus_DEBUG(cv::Mat frame, cv::Mat draw)
{
    /* 识别前 图像处理 */
    cv::Mat gray, ROI, bulr, thres, canny, erode, element, dilate;

    // ROI提取
    std::vector<cv::Point> points;
    double max_X = frame.size().width;
    double max_Y = frame.size().height;
    points.push_back(cv::Point(max_X * (0.5 / 5.0), max_Y * (4.95 / 5.0))); // LD
    points.push_back(cv::Point(max_X * (4.5 / 5.0), max_Y * (4.95 / 5.0))); // RD
    points.push_back(cv::Point(max_X * (4.0 / 5.0), max_Y * (3.0 / 5.0))); // RU
    points.push_back(cv::Point(max_X * (1.0 / 5.0), max_Y * (3.0 / 5.0))); // LU
    ROI = ROI_extract(frame, points);
    line(draw, points[1], points[2], cv::Scalar(0, 255, 0), 1, 8);
    line(draw, points[2], points[3], cv::Scalar(0, 255, 0), 1, 8);
    line(draw, points[3], points[0], cv::Scalar(0, 255, 0), 1, 8);
    // 转灰度
    cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
    // 高斯滤波
    cv::GaussianBlur(gray, bulr, cv::Size(7, 7), 0, 0);
    // 阈值化处理
    cv::threshold(bulr, thres, 150, 255, cv::THRESH_BINARY);
    // 膨胀处理
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    cv::dilate(thres, dilate, element);
    // Canny边缘检测
    cv::Canny(dilate, canny, 100, 200, 3);
    /* 识别前图像处理结束 */

    /**** 识别图像轮廓 ****/
    // 存储轮廓
    std::vector<std::vector<cv::Point>> contours;
    // 存储轮廓层级信息
    std::vector<cv::Vec4i> hierachy;
    // 查找轮廓
    cv::findContours(canny, contours, hierachy, cv::RETR_LIST,
        cv::CHAIN_APPROX_NONE);

    // 定义-存储满足条件的四边形角点集合
    std::vector<std::vector<cv::Point2f>> all_point_sets;

    int min_area = 5000;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > min_area) {
            // 多边形逼近
            std::vector<cv::Point2f> points;
            cv::approxPolyDP(contours[i], points, 10.0, true);

            bool If_coincide = false;
            // 添加筛选条件防止点重合
            for (int i = 0; i < points.size(); i++) {
                if (getDistance(points[i], points[(i + 1) % 4]) <= 25)
                    If_coincide = true;
            }

            if (If_coincide == true)
                continue;

            // 筛选出四边形
            if (points.size() == 4) {
                // 绘制角点
                for (int i = 0; i < points.size(); i++) {
                    std::string text = std::to_string(i + 1);
                    cv::putText(draw, text, points[i], 1, 3, cv::Scalar(255, 0, 0), 2, 8);
                    cv::circle(draw, points[i], 4, cv::Scalar(255, 0, 0), -1, 8,
                        0);
                }
                // 绘制轮廓
                for (int j = 0; j < contours[i].size(); j++) {
                    cv::circle(draw, contours[i][j], 1, cv::Scalar(0, 255, 0), -1, 8,
                        0);
                }

                // 定义-长宽比(利用问号表达式使其大于一)
                double len_to_wid_ratio = (getDistance(points[0], points[1]) >= getDistance(points[1], points[2])) ? (getDistance(points[0], points[1]) / getDistance(points[1], points[2])) : (getDistance(points[1], points[2]) / getDistance(points[0], points[1]));

                if (len_to_wid_ratio < 2) {
                    double diag_slope_1, diag_slope_2;
                    if ((getAbs(getSlope(points[0], points[2])) < 1) && (getAbs(getSlope(points[1], points[3])) > 1)) {
                        diag_slope_1 = getAbs(getSlope(points[0], points[2])),
                        diag_slope_2 = getAbs(getSlope(points[1], points[3]));
                    } else if ((getAbs(getSlope(points[0], points[2])) > 1) && (getAbs(getSlope(points[1], points[3])) < 1)) {
                        diag_slope_2 = getAbs(getSlope(points[0], points[2])),
                        diag_slope_1 = getAbs(getSlope(points[1], points[3]));
                    } else {
                        continue;
                    }

                    double product = diag_slope_1 * diag_slope_2;

                    if ((diag_slope_1 < 0.5) && (diag_slope_2 > 2.5)) {
                        rhombus_count++;
                        std::string result;
                        result.append(std::to_string(diag_slope_1));
                        result.append("|");
                        result.append(std::to_string(diag_slope_2));
                        result.append("|");
                        result.append(std::to_string(product));
                        all_point_sets.push_back(points);
                        cv::putText(draw, result, points[0], 1, 3, cv::Scalar(255, 255, 0), 2, 8);
                        cv::putText(draw, std::to_string(cv::contourArea(contours[i])), cv::Point(points[0].x, points[0].y - 50), 1, 3, cv::Scalar(255, 0, 255), 2, 8);
                    }
                }
            }
        }
    }

    /* 判断是否减速：
        当 rhombus_count > 5时 输出停止信号
    */
    cv::String text = "IF_SLOW:";
    if (rhombus_count > 10) {
        IF_SLOW = true;
    }

    if (IF_SLOW == true) {
        text.append("YES");
        cv::putText(draw, std::to_string(rhombus_count), cv::Point(50, 100), 1, 4, cv::Scalar(0, 255, 0), 4, 8);
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 255, 0), 4, 8);
    } else {
        text.append("NO");
        cv::putText(draw, std::to_string(rhombus_count), cv::Point(50, 100), 1, 4, cv::Scalar(0, 0, 255), 4, 8);
        cv::putText(draw, text, cv::Point(50, 50), 1, 4, cv::Scalar(0, 0, 255), 4, 8);
    }

    return draw;
}

void BFunc::processFunc(BasicFunc& basicFunc)
{
    Mat frame;
    Mat frame2;
    int countBarre = 0;

    while (!basicFunc.exit_flag) {

        // 使用互斥锁保护队列的访问
        std::unique_lock<std::mutex> lock(basicFunc.image_mutex);
        basicFunc.frame_available.wait(lock, [&basicFunc] {
            return !basicFunc.image_queue.empty();
        });

        if (!basicFunc.image_queue.empty()) {
            cout << "get frame！" << endl;
            frame = basicFunc.image_queue.front();
            basicFunc.image_queue.pop();
        }

        if (!frame.empty()) {
            frame.copyTo(frame2);
            // imshow("imgtest", frame);
            // waitKey(1);

            //****锥桶****
            if (findBarrel(frame))
                countBarre++;
            // else cout << "FIND NO BARREL" << endl;
            if (countBarre == 10) {
                IfChangeSide = true; /// 连续10帧发现锥桶
                // std::cout << "FIND_BARRE" << std::endl;
                countBarre = 0;
            }
            if (!countBarre && !IfChangeSide) {
                //****菱形减速****
                // If_Rhombus(frame2);
                If_Rhombus_DEBUG(frame, frame2);
                // imshow("DEBUG", frame2);
                // waitKey(1);
            }
            if (IF_SLOW) {
                //****斑马线停止****
                If_ZebraCrossing(frame2);
            }
            // cboard.SendB(false, Deviation(), IfChangeSide, IF_SLOW, IF_STOP);

        } else
            std::cout << "Empty Frame!!" << std::endl;
    }
    std::cout << "END PROGRESS" << std::endl;
}

//********************************************************************************
//**-------------------------------main()函数-----------------------------------------**
//********************************************************************************

int main()
{
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR!! Unable to open camera\n";
        return -1;
    }

    try {
        BasicFunc basic(true);

        // 启动图像处理线程
        std::thread imageProcessingThread([&basic]() {
            basic.always(basic);
        });

        // 图像捕获循环
        while (true) {
            Mat frame;
            cap >> frame;
            if (!frame.empty()) {
                cout << "SUCCESS! Send frame to image_queue" << frame.size() << endl;
                basic.AddFrame(frame);
            }
        }

        // 设置退出标志，以便图像处理线程退出
        basic.exit_flag = true;
        // 等待图像处理线程结束
        imageProcessingThread.join();
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception: " << e.what() << std::endl;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}