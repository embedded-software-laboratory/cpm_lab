#include "cpm_tools/CpmNode.hpp"


namespace cpm_tools { 

CpmNode::CpmNode(const std::string &node_name, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool allow_early_execution)
:Node(node_name)
,period_nanoseconds_(period_nanoseconds)
,offset_nanoseconds_(offset_nanoseconds)
,allow_early_execution_(allow_early_execution)
,absoluteTimer_(
    time_t(period_nanoseconds / NANOSEC_PER_SEC),
    long(period_nanoseconds % NANOSEC_PER_SEC),
    time_t(offset_nanoseconds / NANOSEC_PER_SEC),
    long(offset_nanoseconds % NANOSEC_PER_SEC),
    [this](){ semaphore.signal(); }
){}

void CpmNode::start_loop() {
    // Set the first deadline: Round up to the next period, then add the offset.
    uint64_t deadline_nanoseconds = ((clock_gettime_nanoseconds()/period_nanoseconds_)+1)*period_nanoseconds_ + offset_nanoseconds_;
    while(rclcpp::ok()) {
        semaphore.wait();

        bool deadline_arrived = clock_gettime_nanoseconds() >= deadline_nanoseconds;

        bool all_messages_arrived = true;
        for(auto subscriber : subscribers) {
            if(!subscriber->message_arrived(deadline_nanoseconds)) {
                all_messages_arrived = false;
                break;
            }
        }

        if(deadline_arrived && !all_messages_arrived) {
            RCLCPP_WARN(this->get_logger(), "New messages missing, using old messages, in node '%s'", get_name())
        }

        if(deadline_arrived || (allow_early_execution_ && all_messages_arrived)) {
            RCLCPP_INFO(this->get_logger(), "starting update() for deadline %.6f at time %.6f", deadline_nanoseconds/double(NANOSEC_PER_SEC), clock_gettime_nanoseconds()/double(NANOSEC_PER_SEC))
            this->update(deadline_nanoseconds);
            deadline_nanoseconds += period_nanoseconds_;
            while(clock_gettime_nanoseconds() >= deadline_nanoseconds) {
                RCLCPP_WARN(this->get_logger(), "Missed deadline %.6f", deadline_nanoseconds/double(NANOSEC_PER_SEC))
                deadline_nanoseconds += period_nanoseconds_;
            }
        }
    }
}


}// end namespace