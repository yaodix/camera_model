#include "ocamcalib_undistort/ocam_functions.h"
#include "ocamcalib_undistort/Parameters.h"

#include <string>
#include <exception>
// #include <ros/ros.h>
// #include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <opencv2/core/core.hpp>

// Parameters getParameters(ros::NodeHandle& nh)
// {
//     Parameters params;

//     nh.param<std::string>("camera_type", params.cameraType, "fisheye");
//     nh.param<std::string>("base_in_topic", params.inTopic, "/camera/image");
//     nh.param<std::string>("base_out_topic", params.outTopic, "/ocamcalib_undistorted");
//     nh.param<std::string>("calibration_file_path", params.calibrationFile, "include/calib_results.txt");
//     nh.param<std::string>("transport_hint", params.transportHint, "raw");

//     nh.param<double>("scale_factor", params.scaleFactor, 4);
//     nh.param<int>("left_bound", params.leftBound, 0);
//     nh.param<int>("right_bound", params.rightBound, 0);
//     nh.param<int>("top_bound", params.topBound, 0);
//     nh.param<int>("bottom_bound", params.bottomBound, 0);

//     return params;
// }



void printOcamModel(const struct ocam_model& model)
{
    std::cout << "OCamCalib model parameters" << std::endl
              << "pol: " << std::endl;
    for (int i=0; i < model.length_pol; i++)
    {
        std::cout << "\t" << model.pol[i] << "\n";
    }

    std::cout << "\ninvpol: " << std::endl;
    for (int i=0; i < model.length_invpol; i++)
    {
        std::cout << "\t" << model.invpol[i] << "\n";
    };
    std::cout << std::endl;

    std::cout << "xc:\t" << model.xc << std::endl
              << "yc:\t" << model.yc << std::endl
              << "width:\t" << model.width << std::endl
              << "height:\t" << model.height << std::endl;
}


int main(int argc, char **argv) {
    // If starts with a / consider it an absolute path
    std::string calibration = ;

    ocam_model model; // our ocam_models for the fisheye and catadioptric cameras
    if(!get_ocam_model(&model, calibration.c_str()))
    {
        return 2;
    }

    printOcamModel(model);
    params.rightBound = params.rightBound == 0 ? model.width : params.rightBound;
    params.bottomBound = params.bottomBound == 0 ? model.height : params.bottomBound;


    CvMat* cmapx_persp = cvCreateMat(model.height, model.width, CV_32FC1);
    CvMat* cmapy_persp = cvCreateMat(model.height, model.width, CV_32FC1);

    create_perspecive_undistortion_LUT(cmapx_persp, cmapy_persp, &model, params.scaleFactor);

    // Need to convert to C++ style to play nice with ROS
    cv::Mat mapx_persp = cv::cvarrToMat(cmapx_persp);
    cv::Mat mapy_persp = cv::cvarrToMat(cmapy_persp);


    // cv::Rect roi(params.leftBound,
    //              params.topBound,
    //              params.rightBound-params.leftBound,
    //              params.bottomBound-params.topBound);

    cv::Mat undistorted(inImage.size(), inImage.type());

    cv::remap(
            inImage,
            undistorted,
            mapx_persp,
            mapy_persp,
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT,
            cv::Scalar(0));

    // cv::Mat cropped(undistorted, roi);

    cvReleaseMat(&cmapx_persp);
    cvReleaseMat(&cmapy_persp);

    return 0;
}
