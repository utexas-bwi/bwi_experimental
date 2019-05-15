#include "lib/CameraInterface.h"
#include <cmath>

#define SCALE 2

//Finds the area of the triangle.
double area(vector<Point> *);
//Returns which double the first is closest to.
int isCloserTo(double, double, double);
//Returns whether the triangle is reasonably isosceles.
bool isReasonablyIsosceles(vector<Point> *);
//Distance formula.
double distance(double, double, double, double);
//Whether the double is in range of the set defined by the other two.
bool inRangeValues(double, double, double);

void CameraInterface::image_callback(
    const sensor_msgs::Image::ConstPtr &msg) {
    CameraImage *ptr = lastImage;
    lastImage = new CameraImage(msg);
    process();
    if (ptr != NULL)
        delete ptr;
}

void CameraInterface::process() {

    if (abs(controller->getCurrentPosition()) < 0.40)
        return;

    int cols = lastImage->width;
    int rows = lastImage->height;

    Mat hsv_frame(cols, rows, CV_8UC4);
    Mat thresholded(cols, rows, CV_8UC1);

    Mat image = lastImage -> image;

    cvtColor(image, hsv_frame, CV_RGB2HSV);
    inRange(hsv_frame, hsv_min, hsv_max, thresholded);

    vector <vector <Point> > contours, approximation;
    vector <Vec4i> hierarchy;
    findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    approximation.resize(contours.size());
    for (int k = 0; k < contours.size(); k++) {
        approxPolyDP(Mat(contours[k]), approximation[k], sigma, true);
    }
    for ( int i = 0; i < approximation.size(); i++ ) {
        if (approximation[i].size() == 3) {
            verifyTriangle(&approximation[i]);
            Scalar color = Scalar(255, 255, 255);
            drawContours( thresholded, approximation, i, color, 2, 8, hierarchy, 0, Point() );
        }
    }
    namedWindow("contours", 1);
    imshow("contours", thresholded);
    waitKey(1);
}

void CameraInterface::generateMinMax(Scalar ideal, Scalar range, enum hsv_type type) {
    double h = max((int)(ideal.val[H] - range.val[H]), 0);
    double s = max((int)(ideal.val[S] - range.val[S]), 0);
    double v = max((int)(ideal.val[V] - range.val[V]), 0);

    hsv_min = Scalar(h, s, v);

    h = min((int)(ideal.val[H] + range.val[H]), 255);
    s = min((int)(ideal.val[S] + range.val[S]), 255);
    v = min((int)(ideal.val[V] + range.val[V]), 255);

    hsv_max = Scalar(h, s, v);
}

enum Direction CameraInterface::getDirection(vector<Point> *vertices) {
    double A[3] = {
        (*vertices)[0].y,
        (*vertices)[1].y,
        (*vertices)[2].y
    };

    double minimum = min(A[0], min(A[1], A[2]));
    double maximum = max(A[0], max(A[1], A[2]));

    int closeToMin = 0;

    closeToMin += isCloserTo(A[0], minimum, maximum) == 1 ? 1 : 0;
    closeToMin += isCloserTo(A[1], minimum, maximum) == 1 ? 1 : 0;
    closeToMin += isCloserTo(A[2], minimum, maximum) == 1 ? 1 : 0;

    if (closeToMin == 1)
        return flip ? DIR_DOWN : DIR_UP;

    return flip ? DIR_UP : DIR_DOWN;
}

double area(vector<Point> *vertices) {
    if (vertices->size() != 3)
        return -1;
    double A[2] = {(*vertices)[0].x, (*vertices)[0].y};
    double B[2] = {(*vertices)[1].x, (*vertices)[1].y};
    double C[2] = {(*vertices)[2].x, (*vertices)[2].y};

    return abs(A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0;
}

int isCloserTo(double subject, double one, double two) {
    double diffOne = abs(one - subject);
    double diffTwo = abs(two - subject);
    return diffTwo < diffOne ? 2 : 1;
}

bool isReasonablyIsosceles(vector<Point> *vertices) {
    double A[2] = {(*vertices)[0].x, (*vertices)[0].y};
    double B[2] = {(*vertices)[1].x, (*vertices)[1].y};
    double C[2] = {(*vertices)[2].x, (*vertices)[2].y};

    double sides[3] = {
        distance(A[0], A[1], B[0], B[1]),
        distance(B[0], B[1], A[0], A[1]),
        distance(B[0], B[1], C[0], C[1])
    };

    sides[1] /= sides[0];
    sides[2] /= sides[0];
    sides[0] = 1;

    return (abs(sides[1] - sides[2]) <= 0.1 || abs(sides[1] - sides[0]) <= 0.1 || abs(sides[2] - sides[0]) <= 0.1);
}

double distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
}

bool CameraInterface::verifyTriangle(vector<Point> *vertices) {
    double areaNum = area(vertices);
    if (tri[0] <= areaNum && areaNum <= tri[1] && inRegion(vertices)) { // && isReasonablyIsosceles(vertices)) {

        if (verbose) {
            cout << *vertices << endl;
            cout << "Found triangle of area: " << areaNum << endl;
        }

        if (controller->getCurrentPosition() > 0)
            cout << "Elevator on the left has arrived" << endl;
        else
            cout << "Elevator on the right has arrived" << endl;

        enum Direction dir = getDirection(vertices);
        cout << "Direction: ";
        switch (dir) {
        case DIR_UP : cout << "Down" << endl;
            break;
        case DIR_DOWN : cout << "Up" << endl;
            break;
        case DIR_UNKNOWN : cout << "Not enough info!" << endl;
        }
        if (found++ >= 1) {
            cout << "Stopping scan" << endl;
            exit(0);
        }
        return true;
    }
    return false;
}

bool CameraInterface::inRegion(vector<Point> *vertices) {

    double X[3] = {
        (*vertices)[0].x,
        (*vertices)[1].x,
        (*vertices)[2].x
    };
    double Y[3] = {
        (*vertices)[0].y,
        (*vertices)[1].y,
        (*vertices)[2].y
    };

    for (int k = 0; k < 3; k++) {
        if ((controller->getCurrentPosition() > 0 && !inRangeValues(X[k], 400, 600)) ||
                (controller->getCurrentPosition() < 0 && !inRangeValues(X[k], 0, 200)) || Y[k] < 300) {
            if (verbose)
                cout << "Rejecting triangle " << *vertices << endl;
            return false;
        }
    }
    return true;

}

bool inRangeValues(double a, double s, double e) {
    return (s < a && a < e);
}
