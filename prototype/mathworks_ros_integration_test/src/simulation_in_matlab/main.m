function main

    clc
    rosshutdown;
    rosinit('localhost',11311);
    
    acceleration = 0;
    position = 0;
    speed = 1;
    dt = 1/50;
    
    function set_acceleration_callback(src,msg)
        acceleration = msg.Data;
    end

    acceleration_subscriber = rossubscriber('acceleration', 'std_msgs/Float64', @set_acceleration_callback);
%     speed_publisher = rospublisher('speed', 'std_msgs/Float64');
    position_publisher = rospublisher('position', 'std_msgs/Float64');

    rate = robotics.Rate(50);
    while true
        speed    = speed    + dt * acceleration;
        position = position + dt * speed;

%         speed_message = rosmessage(speed_publisher);
%         speed_message.Data = speed;
%         send(speed_publisher, speed_message);

        position_message = rosmessage(position_publisher);
        position_message.Data = position;
        send(position_publisher, position_message);

        rate.waitfor();    
    end



    rosshutdown
end