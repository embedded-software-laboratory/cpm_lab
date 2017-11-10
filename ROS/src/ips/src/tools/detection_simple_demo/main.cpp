
#include "utils/default.h"
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include "algorithms/DetectionDispatcherLogic/DetectionDispatcherLogic.h"
#include "algorithms/AprilTagDetector/AprilTagDetector.h"

using namespace Pylon;
using namespace GenApi;


class CMyImageEventPrinter : public CImageEventHandler
{
public:

    virtual void OnImagesSkipped( CInstantCamera& camera, size_t countOfSkippedImages)
    {
        std::cout << "OnImagesSkipped event for device " << camera.GetDeviceInfo().GetModelName();
        std::cout << countOfSkippedImages  << " images have been skipped." << std::endl;
        std::cout << std::endl;
    }


    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
        std::cout << "OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;

        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            std::cout << "SizeX: " << ptrGrabResult->GetWidth() << " -- ";
            std::cout << "SizeY: " << ptrGrabResult->GetHeight() << " -- ";
            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            std::cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << " ------ Timestamp " << ptrGrabResult->GetTimeStamp() << std::endl;
            std::cout << std::endl;
        }
        else
        {
            std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
        }
    }
};


int main(int argc, char* argv[])
{
    int exitCode = 0;
    PylonInitialize();

    try
    {
        CInstantCameraArray cameras( 1 );
        //cameras[0].Attach(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21704342")));
        cameras[0].Attach(CTlFactory::GetInstance().CreateDevice(CDeviceInfo().SetSerialNumber("21967260")));

        for ( size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cout << "Using device " << cameras[ i ].GetDeviceInfo().GetModelName()  << "   SN  "  << cameras[i].GetDeviceInfo().GetSerialNumber() << endl;

            //cameras[ i ].RegisterImageEventHandler( new CMyImageEventPrinter, RegistrationMode_Append, Cleanup_Delete);
            cameras[ i ].RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
            cameras[ i ].StartGrabbing();

            INodeMap& nodemap = cameras[ i ].GetNodeMap();

            CEnumerationPtr gainAuto( nodemap.GetNode( "GainAuto"));
            gainAuto->FromString("Off");

            CEnumerationPtr expauto( nodemap.GetNode( "ExposureAuto"));
            expauto->FromString("Off");

            CFloatPtr gain( nodemap.GetNode( "Gain"));
            gain->SetValue(5);

            CFloatPtr exp( nodemap.GetNode( "ExposureTime"));
            exp->SetValue(1000);
        }

        CameraParameters params;

        params.fx = 1;
        params.fy = 1;
        params.cx = 0;
        params.cy = 0;
        params.k1 = 0;
        params.k2 = 0;
        params.k3 = 0;
        params.p1 = 0;
        params.p2 = 0;
        params.R = cv::Matx33d(1,0,0,0,1,0,0,0,1);
        params.T = cv::Vec3d(0,0,0);

        DetectionDispatcherLogic detectionDispatcherLogic(params);

        vector<AprilTagDetectionStamped> detections;

        vector<cpm_msgs::VehicleState> vehicle_states;
        {
            cpm_msgs::VehicleState v;
            v.id = 1;
            v.pose.position.x = NaN;
            vehicle_states.push_back(v);;
        }
        {
            cpm_msgs::VehicleState v;
            v.id = 50;
            v.pose.position.x = NaN;
            vehicle_states.push_back(v);;
        }

        AprilTagDetector aprilTagDetector(AprilTagFamily::Tag36h11);

        ros::Time::init();

        bool loop = true;
        while(loop)
        {

            for ( size_t i = 0; i < cameras.GetSize(); ++i) {
                cameras[ i ].ExecuteSoftwareTrigger();
            }

            for ( size_t i = 0; i < cameras.GetSize(); ++i)
            {
                cv::Mat image_copy;
                string serial_no;

                {
                    CGrabResultPtr ptrGrabResult;
                    if (!cameras[i].RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException)) {
                        cout << "RetrieveResult failed" << endl;
                        return 1;
                    }

                    intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();

                    auto SN = cameras[cameraContextValue].GetDeviceInfo().GetSerialNumber();
                    serial_no = string(SN.c_str());

                    int rows = ptrGrabResult->GetHeight();
                    int cols = ptrGrabResult->GetWidth();
                    void *data = ptrGrabResult->GetBuffer();
                    size_t stride;
                    assert(ptrGrabResult->GetStride(stride));
                    assert(ptrGrabResult->GetPixelType() == PixelType_Mono8);
                    cv::Mat image_tmp(rows, cols, CV_8UC1, data, stride);
                    image_tmp.copyTo(image_copy);
                }

                //cv::resize(image_copy, image_copy, cv::Size(), 0.5, 0.5);



                vector<cv::Rect> ROIs;
                bool full_frame_detection;
                tie(ROIs, full_frame_detection) = detectionDispatcherLogic.apply( detections, vehicle_states );

                detections.clear();

                cv::Rect full_frame_ROI(0,0,image_copy.cols,image_copy.rows);

                if(full_frame_detection) ROIs.push_back(full_frame_ROI);


                for(auto ROI:ROIs) {
                    ROI &= full_frame_ROI;
                    if(ROI.area() > 0) {
                        cv::Mat crop = image_copy(ROI);
                        auto crop_detections = aprilTagDetector.detect(crop, ROI.tl());
                        for (auto const &crop_detection : crop_detections) {
                            for(auto const &vehicle:vehicle_states) {
                                if(vehicle.id == crop_detection.id) {
                                    detections.emplace_back(crop_detection, ros::Time::now());
                                }
                            }

                        }
                    }
                }


                cout << ROIs.size() << " ---  " << detections.size() << endl;

                cv::cvtColor(image_copy, image_copy, CV_GRAY2BGR);

                for (auto const & detection:detections) {

                    for (int k = 0; k < 4; ++k) {
                        int m = (k + 1)%4;
                        cv::line(image_copy,
                                 cv::Point(detection.points[k][0], detection.points[k][1]),
                                 cv::Point(detection.points[m][0], detection.points[m][1]),
                                 k?cv::Scalar(0,255,0):cv::Scalar(0,0,255), 5);
                    }
                }

                // smaller than my fullHD screen -> no scaling
                image_copy = image_copy(cv::Rect(0,0,1600,950));
                cv::imshow(serial_no, image_copy);

                int key = cv::waitKey(1);
                if(key == 27) loop = false;
            }
        }
    }
    catch (const GenericException &e)
    {
        // Error handling
        cout << "An exception occurred." << endl
             << e.GetDescription() << endl;
        exitCode = 1;
    }

    PylonTerminate();

    return exitCode;
}