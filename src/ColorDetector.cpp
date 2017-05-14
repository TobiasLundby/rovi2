#include "ColorDetector.hpp"
#include <iostream>
#define DEBUG false
#define DEBUG_SHIFTED_IMAGE false
#define DEBUG_RESULT true
#define ENABLE_FILTER_TRACKBARS false

ColorDetector::ColorDetector(){
    // All the parameters below are used for the blob detector - only some of them are used because motion blur introduces different shapes etc.
    // Change thresholds
    //params.minThreshold = 0;
    //params.maxThreshold = 100;
    params.blobColor = 255; // This parameter is used instead of the threadholds to detect the white color
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = CV_PI*pow(20,2); // Radius 50px
    params.maxArea = CV_PI*pow(500,2); // Radius 100px
    // Filter by Circularity - we do not do this parameter due to motion blur
    params.filterByCircularity = true;
    params.minCircularity = 0.5;
    // Filter by Convexity - we do not use this parameter to ensure detection
    params.filterByConvexity = false;
    //params.minConvexity = 0.87;
    // Filter by Inertia - not used but it means "the inertial resistance of the blob to rotation about its principal axes"
    params.filterByInertia = false;
    //params.minInertiaRatio = 0.01;
    //params.maxInertiaRatio = 0.5;
    detector = SimpleBlobDetector::create(params); // Set up detector with params
    if (ENABLE_FILTER_TRACKBARS) {
        namedWindow("Trackbars");
        createTrackbar("HSV h low", "Trackbars", &hsv_h_low_MR, 180); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value
        createTrackbar("HSV h high", "Trackbars", &hsv_h_upper_MR, 180); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value

        createTrackbar("HSV s low", "Trackbars", &hsv_s_low_MR, 255); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value
        createTrackbar("HSV s high", "Trackbars", &hsv_s_upper_MR, 255); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value

        createTrackbar("HSV v low", "Trackbars", &hsv_v_low_MR, 255); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value
        createTrackbar("HSV v high", "Trackbars", &hsv_v_upper_MR, 255); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value

        createTrackbar("Erode", "Trackbars", &erode_color_iterations, 5); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value
        createTrackbar("Dilate", "Trackbars", &dilate_color_iterations, 5); // 1st arg: name; 2nd arg: window; 3rd arg: pointer to the variabel (must be int); 4th arg: max value
    }
}

std::vector<Point2f> ColorDetector::FindMarker(Mat &image) {
    // Create / convert images to color spaces for processing
    Mat image_hsv, image_gray;
    cvtColor(image, image_hsv,  COLOR_BGR2HSV); // Convert from BGR to HSV
    cvtColor(image, image_gray, COLOR_BGR2GRAY); //Convert the captured frame from BGR to GRAY

    // Shift hue values to color seperate better for red detection
    Mat image_hsv_shifted;
    image_hsv.copyTo(image_hsv_shifted);
    for(int y=0; y < image_hsv_shifted.rows; y++)
    {
        for(int x=0; x < image_hsv_shifted.cols; x++)
        {
            int tmp_color = image_hsv_shifted.at<Vec3b>(Point(x,y))[0];
            tmp_color += 60;
            if (tmp_color >= 360)
                tmp_color = tmp_color%360;
            image_hsv_shifted.at<Vec3b>(Point(x,y))[0] = tmp_color;
        }
    }
    if (DEBUG_SHIFTED_IMAGE) {
        Mat image_bgr_shifted;
        cvtColor(image_hsv_shifted, image_bgr_shifted,  COLOR_HSV2BGR); // Convert from BGR to HSV
        imshow("shifted",image_bgr_shifted);
    }


    // Mask red parts
    Mat mask_MR, image_masked_MR;
    inRange(image_hsv_shifted, Scalar(hsv_h_low_MR, hsv_s_low_MR, hsv_v_low_MR), Scalar(hsv_h_upper_MR, hsv_s_upper_MR, hsv_v_upper_MR), mask_MR); // Find the areas which contain the color. 1st arg: inpur frame; 2nd arg: the lower HSV limits; 3rd arg: the upper HSV limits; 4th arg: the output mask.

    for (size_t i = 0; i < erode_color_iterations; i++)
        erode(mask_MR, mask_MR, Mat(), Point(-1,-1)); // Enhance the areas in the image

    for (size_t i = 0; i < dilate_color_iterations; i++)
        dilate(mask_MR, mask_MR, Mat(), Point(-1,-1)); // Enhance the areas in the image

    image_gray.copyTo(image_masked_MR, mask_MR);
    if(DEBUG)
    {
        //imshow("img_hsv",image_hsv_shifted);
        imshow("mask_MR",mask_MR);
        //imshow("image_mask_MR",image_masked_MR);
    }

    // Detect blobs.
    std::vector<KeyPoint> keypoints_MR;
    detector->detect( mask_MR, keypoints_MR);
    if(DEBUG_RESULT)
    {
        Mat image_local;
        image.copyTo(image_local);
        drawKeypoints(image_local,keypoints_MR,image_local,Scalar(0,255,0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow(window_name,image_local);
    }

    /* Calculate error; only for the largest blob */
    // for( int i=0; i<keypoints_MR.size(); i++){
    //     std::stringstream buffer_right;
    //     buffer_right << "size:" << keypoints_MR.at(i).size << " u/x:" << keypoints_MR.at(i).pt.x << " v/y:" << keypoints_MR.at(i).pt.y << std::endl;
    //     ROS_ERROR("Info %i: %s", i, buffer_right.str().c_str());
    // }
    /* Calculate error; only for the largest blob - end */

    std::vector< Point2f > output_points;
    for( int i=0; i<keypoints_MR.size(); i++)
        output_points.push_back(keypoints_MR.at(i).pt);

    return output_points;
}

std::vector<double> estimate_variance(Mat &image)
{

}

void ColorDetector::set_result_window_name(std::string name)
{
    window_name = name;
}
