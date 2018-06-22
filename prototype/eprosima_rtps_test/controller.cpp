
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/Domain.h>
#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/subscriber/Subscriber.h>


#include "VehicleStatePubSubTypes.h"
#include "VehicleInputPubSubTypes.h"

#include <cmath>
#include <unistd.h>
#include <fastrtps/subscriber/SampleInfo.h>

using namespace eprosima;
using namespace eprosima::fastrtps;

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
        VehicleInput input;
        VehicleState state;
        SampleInfo_t info;

        subscriber->waitForUnreadMessage();
        if(subscriber->takeNextData((void *) &state, &info)) {
            std::cout << " x " << state.x() << "  y " << state.y() << std::endl;   
        }
    }



    std::cout << "The end." << std::endl;    
    return 0;
}