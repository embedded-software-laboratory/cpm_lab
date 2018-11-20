#pragma once
#include <iostream>
#include <string>

#include <dds/pub/ddspub.hpp>

template<class MessageType> class Publisher
{
private:
	dds::pub::Publisher pub;
	dds::pub::DataWriter<MessageType> writer;
	std::string t_name;
public:
	Publisher(int domain_id, std::string topic_name, dds::domain::DomainParticipant& _participant, dds::topic::Topic<MessageType>& topic);
	void send(MessageType message);
};

template<class MessageType> Publisher<MessageType>::Publisher(int domain_id, std::string topic_name, dds::domain::DomainParticipant& _participant, dds::topic::Topic<MessageType>& topic) :
	pub(_participant),
	writer(pub, topic)
{
	t_name = topic_name;
}

template<class MessageType> void Publisher<MessageType>::send(MessageType message)
{
	writer.write(message);
}
