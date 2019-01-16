

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ParameterRequest.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#include <iosfwd>
#include <iomanip>
#include "ParameterRequest.hpp"
#include "ParameterRequestImplPlugin.h"

#include <rti/util/ostream_operators.hpp>

// ---- ParameterRequest: 

ParameterRequest::ParameterRequest() {
}   

ParameterRequest::ParameterRequest (
    const dds::core::string& name_param)
    :
        m_name_( name_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
ParameterRequest::ParameterRequest(ParameterRequest&& other_) OMG_NOEXCEPT  :m_name_ (std::move(other_.m_name_))
{
} 

ParameterRequest& ParameterRequest::operator=(ParameterRequest&&  other_) OMG_NOEXCEPT {
    ParameterRequest tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void ParameterRequest::swap(ParameterRequest& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_name_, other_.m_name_);
}  

bool ParameterRequest::operator == (const ParameterRequest& other_) const {
    if (m_name_ != other_.m_name_) {
        return false;
    }
    return true;
}
bool ParameterRequest::operator != (const ParameterRequest& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
dds::core::string& ParameterRequest::name() OMG_NOEXCEPT {
    return m_name_;
}

const dds::core::string& ParameterRequest::name() const OMG_NOEXCEPT {
    return m_name_;
}

void ParameterRequest::name(const dds::core::string& value) {
    m_name_ = value;
}

std::ostream& operator << (std::ostream& o,const ParameterRequest& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "name: " << sample.name() ;
    o <<"]";
    return o;
}

// --- Type traits: -------------------------------------------------

namespace rti { 
    namespace topic {

        const dds::core::xtypes::StructType& dynamic_type<ParameterRequest>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(ParameterRequest_c_get_typecode())));
        }

    }
}  

namespace dds { 
    namespace topic {
        void topic_type_support<ParameterRequest>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                ParameterRequest_cPlugin_new,
                ParameterRequest_cPlugin_delete);
        }

        void topic_type_support<ParameterRequest>::initialize_sample(ParameterRequest& sample){

            ParameterRequest_c* native_sample=reinterpret_cast<ParameterRequest_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            ParameterRequest_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=ParameterRequest_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<ParameterRequest>::to_cdr_buffer(
            std::vector<char>& buffer, const ParameterRequest& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = ParameterRequest_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const ParameterRequest_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = ParameterRequest_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const ParameterRequest_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<ParameterRequest>::from_cdr_buffer(ParameterRequest& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = ParameterRequest_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<ParameterRequest_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create ParameterRequest from cdr buffer");
        }

    }
}  

