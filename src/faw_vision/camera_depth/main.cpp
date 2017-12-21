#include <sstream>
#include <string>
#include <iostream>
#include <math.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <librealsense2/rs.hpp>

using namespace cv;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 223;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//names that will appear at the top of each window
const std::string windowName = "Thing";

int erosion_elem = 0;
int erosion_size = 10;
int dilation_elem = 0;
int dilation_size = 10;
int const max_elem = 2;
int const max_kernel_size = 21;

int main(int argc, char* argv[]) try {
	//some boolean variables for different functionality within this
	//program
    bool trackObjects = false;
    bool useMorphOps = false;

	//Matrices
	//Mat cameraFeed;
	Mat HSV;
	Mat threshold;
	Mat canny_output;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        data = pipe.wait_for_frames();
    }

	while(1) {


        rs2::frame depth_frame = color_map(data.get_depth_frame()); // Find and colorize the depth data
        rs2::frame color_frame = data.get_color_frame();            // Find the color data

        Mat cameraFeed(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
		
		{
			int erosion_type;
			if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
			else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
			else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
		
			Mat element = getStructuringElement( erosion_type,
												Size( 2*erosion_size + 1, 2*erosion_size+1 ),
												Point( erosion_size, erosion_size ) );
		
			/// Apply the erosion operation
			erode( threshold, threshold, element );
		}

		GaussianBlur( threshold, threshold, Size(21,21), 2, 2 );		

		{
			int dilation_type;
			if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
			else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
			else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
		
			Mat element = getStructuringElement( dilation_type,
												Size( 2*dilation_size + 1, 2*dilation_size+1 ),
												Point( dilation_size, dilation_size ) );
			/// Apply the dilation operation
			dilate( threshold, threshold, element );
		}

	    std::vector<Vec4i> hierarchy;
	    std::vector<std::vector<Point> > contours;
	    findContours(threshold, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	    for (size_t i = 0; i < contours.size(); ++i)
	    {
	        // Calculate the area of each contour
	        double area = contourArea(contours[i]);
	        // Ignore contours that are too small or too large
	        if (area < 1e4 || 1e5 < area) continue;
	        // Draw each contour only for visualisation purposes
	        drawContours(cameraFeed, contours, static_cast<int>(i), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
	        // Find the orientation of each shape
	        //getOrientation(contours[i], cameraFeed);
	    }
        
		//show frames 
		imshow(windowName,cameraFeed);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(30);
	}

	return 0;
}
catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
