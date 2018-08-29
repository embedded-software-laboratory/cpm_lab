#include "rclcpp/rclcpp.hpp"
#include "cpm_msgs/msg/vehicle_command.hpp"
#include "cpm_tools/Subscriber.hpp"       
#include "cpm_tools/CpmNode.hpp"

using namespace cpm_tools;


#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
void die(string s)
{
    perror(s.c_str());
    exit(1);
}

class VehicleRelayNode : public CpmNode {

    cpm_tools::Subscriber<cpm_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_subscriber;

    struct sockaddr_in si_other;
    int sock;
    int slen;

public:
    VehicleRelayNode()
    :CpmNode("VehicleRelayNode", 40 * NANOSEC_PER_MILLISEC, 0, false)
    {
        vehicle_command_subscriber = subscribe<cpm_msgs::msg::VehicleCommand>(
            "vehicle04/command", 20 * NANOSEC_PER_MILLISEC);





        /** UDP  setup **/

        slen=sizeof(si_other);
     
        if ( (sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
            die("socket");
        }
     
        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(6783);


        string hostname = "esp_6028349.local";

        struct hostent * hst = gethostbyname(hostname.c_str());
        if(!hst) {
            die("gethostbyname");
        }

        if(hst->h_addr_list == NULL) {
            die("hst->h_addr_list");        
        }

        char addr_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, hst->h_addr_list[0], addr_str, INET_ADDRSTRLEN);
        cout << hostname << " == " << addr_str << endl;

        //char addr_str[] = "192.168.0.103";

        if (inet_aton(addr_str , &si_other.sin_addr) == 0)  {
            die("inet_aton");
        }
    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        bool old_message_flag = false;
        auto vehicle_command = vehicle_command_subscriber->get(deadline_nanoseconds, old_message_flag);
        if(!old_message_flag) {


            float speed = vehicle_command.speed;
            float curvature = vehicle_command.curvature;


            char message[9];
            message[0] = 'c';
            memcpy(message+1, &speed, 4);
            memcpy(message+5, &curvature, 4);
            
            


            if (sendto(sock, message, 9 , 0 , (struct sockaddr *) &si_other, slen)==-1) {
                die("sendto()");
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleRelayNode>();
    std::thread node_thread([&](){node->start_loop();});
    rclcpp::spin(node);
    rclcpp::shutdown();
    node_thread.join();
    return 0;
}