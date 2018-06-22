
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/Domain.h>
#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/subscriber/Subscriber.h>


#include "VehicleStatePubSubTypes.h"
#include "VehicleInputPubSubTypes.h"

#include <cmath>
#include <unistd.h>

using namespace std;
using namespace eprosima;
using namespace eprosima::fastrtps;


VehicleInput control_law(VehicleState state) {
    // implement a simple controller, drive around a square
    static int mode = 0;  

    double x_ref = 0;
    double y_ref = 0;
    double yaw_ref = 0;

    cout << mode << endl;

    const double d = 10;

    if(mode == 0) {
        if(state.x() > d) {
            mode = 1;
        }
        x_ref = 0;
        y_ref = 0;
        yaw_ref = 0;
    }
    else if(mode == 1) {
        if(state.y() > d) {
            mode = 2;
        }
        x_ref = d;
        y_ref = 0;
        yaw_ref = M_PI/2;
    }
    else if(mode == 2) {
        if(state.x() < 0) {
            mode = 3;
        }
        x_ref = d;
        y_ref = d;
        yaw_ref = M_PI;
    }
    else {
        if(state.y() < 0) {
            mode = 0;
        }
        x_ref = 0;
        y_ref = d;
        yaw_ref = M_PI * 1.5;
    }

    double x = state.x();
    double y = state.y();
    double yaw = state.yaw();


    double lateral_error = -sin(yaw_ref) * (x-x_ref)  +cos(yaw_ref) * (y-y_ref);
    double yaw_error = sin(yaw-yaw_ref);


    lateral_error = fmin(0.9,fmax(-0.9, lateral_error));
    yaw_error = fmin(0.9,fmax(-0.9, yaw_error));

    const double curvature = -4.0 * lateral_error -8.0 * yaw_error;

    VehicleInput input;
    input.curvature(curvature);
    return input;
}


int main() {

    // Create RTPSParticipant    
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("controller");
    Participant* participant = Domain::createParticipant(PParam);
    if(participant == nullptr)
        return 1;
    

    //Register the types
    VehicleStatePubSubType stateType;
    VehicleInputPubSubType inputType;
    Domain::registerType(participant, &stateType);
    Domain::registerType(participant, &inputType);
    

    // Create Publisher
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = inputType.getName();  //This type MUST be registered
    Wparam.topic.topicName = "vehicleInput";
    Publisher* publisher = Domain::createPublisher(participant, Wparam);
    if(publisher == nullptr)
        return 1;


    // Create Subscriber
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = stateType.getName(); //Must be registered before the creation of the subscriber
    Rparam.topic.topicName = "vehicleState";
    Subscriber* subscriber = Domain::createSubscriber(participant,Rparam);
    if(subscriber == nullptr)
        return 1;




    std::cout << "Start..." << std::endl;
    while(true) {
        VehicleState state;
        SampleInfo_t info;

        subscriber->waitForUnreadMessage();
        if(subscriber->takeNextData((void *) &state, &info)) {
            VehicleInput input = control_law(state);
            std::cout << " x " << state.x() << "  y " << state.y() << " curv " << input.curvature() << std::endl;
            publisher->write(&input);
        }

        usleep(20000); // Add some delay to make it more interesting
    }



    std::cout << "The end." << std::endl;    
    return 0;
}