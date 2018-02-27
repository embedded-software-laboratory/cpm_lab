#include "rclcpp/rclcpp.hpp"
#include "mex.h"
#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "cpm_tools/Subscriber.hpp"
#include "cpm_tools/CpmNode.hpp"
using namespace cpm_tools;

uint32_t odom_count = 42;


/*

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<cpm_msgs::msg::VehicleSensors>(
        "topic",
        [this](cpm_msgs::msg::VehicleSensors::UniquePtr msg) {
            odom_count = msg->odometer_count;
            rclcpp::shutdown();
        });
    }

private:
  rclcpp::Subscription<cpm_msgs::msg::VehicleSensors>::SharedPtr subscription_;
};*/



void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


    /* check for proper number of arguments */
    if(nrhs!=2) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","Two inputs required.");
    }
    if(nlhs!=1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","One output required.");
    }

    if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1 ) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input must be a scalar.");
    }
    if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || mxGetNumberOfElements(prhs[1])!=1 ) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input must be a scalar.");
    }

    rclcpp::shutdown();

    auto a = mxGetScalar(prhs[0]);
    auto b = mxGetScalar(prhs[1]);

    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* out = mxGetPr(plhs[0]);

    int argc = 1;
    char* argv0 = "asdf";
    rclcpp::init(argc, &argv0);
    //rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();


    *out = odom_count;
}

