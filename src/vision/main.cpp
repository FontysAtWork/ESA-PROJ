#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;

Mat src,src_gray;
Mat src_roi;
Mat dst, cdst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 400;
int ratio = 3;
int kernel_size = 3;

Rect roi(2750, 2600, 800, 800); 

struct LineThing {
    LineThing(double a_, double b_) : a(a_), b(b_) { }
    double a;
    double b;
};

bool isAboutEqual(LineThing a, LineThing b) {
    if  ((a.a > b.a - 0.10 && a.a < b.a + 0.10) &&
        (a.b > b.b - 10.0 && a.b < b.b + 10.0)) {
        return true;
    }
    return false;
}

void findLines(Mat *input, Mat *output) {
    vector<Vec4i> lines;
    vector<LineThing> lineThings;

    HoughLinesP(*input, lines, 1, CV_PI/180, 50, 50, 10 );

    // We use HoughLinesP to make OpenCV detect line pieces. This results in multiple lines
    // however, so we need to find out if the line pieces are on the line.
    for( size_t i = 0; i < lines.size(); i++ ) {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);

        double x1 = lines[i][0];
        double y1 = lines[i][1];
        double x2 = lines[i][2];
        double y2 = lines[i][3];

        double a = (y2-y1)/(x2-x1);
        double b = y2 - (a * x2);

        lineThings.push_back(LineThing(a, b));
    }

    printf("LineThings: %d\n", (int)lineThings.size());

    // We make a new vector with lines that occur just once. This is FinalLines.
    vector<int> okayIndexes;
    for ( int i = 0; i < lineThings.size(); i++ ) {
        for (int j = 0; j < lineThings.size(); j++) {
            if (i == j) continue;
            if (isAboutEqual(lineThings[i], lineThings[j])) {
                if (std::find(okayIndexes.begin(), okayIndexes.end(), i) == okayIndexes.end() &&
                    std::find(okayIndexes.begin(), okayIndexes.end(), j) == okayIndexes.end()) {
                    okayIndexes.push_back(i);
                }
            }
        }
    }

    printf("okayIndexes: %d\n", (int)okayIndexes.size());

    vector<LineThing> finalLines;
    for (int i = 0; i < okayIndexes.size(); i++) {
        finalLines.push_back(lineThings[okayIndexes[i]]);
    }

    std::vector<Vec2i> finalLineVectors;    
    // And then we draw them.
    for( size_t i = 0; i < finalLines.size(); i++ ) {
        LineThing l = finalLines[i];
        // y = a*x+b
        double y1 = l.a*0.0+l.b;
        double y2 = l.a*(*output).cols+l.b;

        line( *output, Point(0, (int)y1), Point(cdst.cols, (int)y2), Scalar(255,0,0), 1, CV_AA);
        finalLineVectors.push_back(Vec2i(cdst.cols-0, (int)y2-(int)y1));
    }

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( *output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
}

void CannyThreshold(int, void*) {
    /// Reduce noise with a kernel 3x3
    //blur( src_gray, detected_edges, Size(3,3) );

    GaussianBlur( src_gray, detected_edges, Size(9,9), 2, 2 );

    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    cvtColor(detected_edges, cdst, CV_GRAY2BGR);

    findLines(&detected_edges, &cdst);

    imshow( "Edge Map", cdst );
}

int main( int argc, char** argv ) {
    /// Load an image
    src = imread( argv[1] );

    if( !src.data ) {
        return -1;
    }

    src_roi = src(roi);

    /// Create a matrix of the same type and size as src (for dst)
    dst.create( src.size(), src.type() );

    /// Convert the image to grayscale
    cvtColor( src_roi, src_gray, CV_BGR2GRAY );

    /// Create a window
    namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );

    /// Create a Trackbar for user to enter threshold
    createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

    /// Show the image
    CannyThreshold(0, 0);

    /// Wait until user exit program by pressing a key
    waitKey(0);

    return 0;
}