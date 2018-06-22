
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

using namespace eprosima;
using namespace eprosima::fastrtps;

int main() {

    // Create RTPSParticipant    
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("dynamics");  //You can put here the name you want
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
    Wparam.topic.topicDataType = stateType.getName();  //This type MUST be registered
    Wparam.topic.topicName = "vehicleState";
    Publisher* publisher = Domain::createPublisher(participant, Wparam);
    if(publisher == nullptr)
        return 1;


    // Create Subscriber
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = inputType.getName(); //Must be registered before the creation of the subscriber
    Rparam.topic.topicName = "vehicleInput";
    Subscriber* subscriber = Domain::createSubscriber(participant,Rparam);
    if(subscriber == nullptr)
        return 1;


    VehicleState state;
    state.x(0);
    state.y(0);
    state.yaw(0);
    state.speed(2);

    while(true) {

        VehicleInput input;
        SampleInfo_t info;
        subscriber->takeNextData((void *) &input, &info);

        const double dt = 0.1;
        const double wheelbase = 0.15;
        const double steering_angle = fmin(.3,fmax(-.3,atan(input.curvature() * wheelbase)));

        state.x(     state.x()      + dt * state.speed() * cos(state.yaw()) );
        state.y(     state.y()      + dt * state.speed() * sin(state.yaw()) );
        state.yaw(   state.yaw()    + dt * state.speed() * tan(steering_angle) / wheelbase );
        state.speed( state.speed()  + dt * input.throttle() );

        publisher->write(&state);
        usleep(50000);
    }



    std::cout << "The end." << std::endl;    
    return 0;
}