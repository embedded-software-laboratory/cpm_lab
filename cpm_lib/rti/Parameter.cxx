

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from Parameter.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#include <iosfwd>
#include <iomanip>
#include "Parameter.hpp"
#include "ParameterImplPlugin.h"

#include <rti/util/ostream_operators.hpp>

std::ostream& operator << (std::ostream& o,const ParameterType& sample){
    rti::util::StreamFlagSaver flag_saver (o);
    switch(sample.underlying()){
        case ParameterType::Invalid:
        o << "ParameterType::Invalid" << " ";
        break;
        case ParameterType::Bool:
        o << "ParameterType::Bool" << " ";
        break;
        case ParameterType::Int32:
        o << "ParameterType::Int32" << " ";
        break;
        case ParameterType::Double:
        o << "ParameterType::Double" << " ";
        break;
        case ParameterType::String:
        o << "ParameterType::String" << " ";
        break;
        case ParameterType::Vector_Int32:
        o << "ParameterType::Vector_Int32" << " ";
        break;
        case ParameterType::Vector_Double:
        o << "ParameterType::Vector_Double" << " ";
        break;
        case ParameterType::Vector_String:
        o << "ParameterType::Vector_String" << " ";
        break;
    }
    return o;
}

// ---- Parameter: 

Parameter::Parameter() :
    m_type_(ParameterType::get_default()) ,
    m_value_bool_ (false) {
}   

Parameter::Parameter (
    const dds::core::string& name_param,
    const ParameterType& type_param,
    bool value_bool_param,
    const dds::core::vector<int32_t>& values_int32_param,
    const dds::core::vector<double>& values_double_param,
    const dds::core::string& values_string_param)
    :
        m_name_( name_param ),
        m_type_( type_param ),
        m_value_bool_( value_bool_param ),
        m_values_int32_( values_int32_param ),
        m_values_double_( values_double_param ),
        m_values_string_( values_string_param ) {
}
#ifdef RTI_CXX11_RVALUE_REFERENCES
#ifdef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
Parameter::Parameter(Parameter&& other_) OMG_NOEXCEPT  :m_name_ (std::move(other_.m_name_))
,
m_type_ (std::move(other_.m_type_))
,
m_value_bool_ (std::move(other_.m_value_bool_))
,
m_values_int32_ (std::move(other_.m_values_int32_))
,
m_values_double_ (std::move(other_.m_values_double_))
,
m_values_string_ (std::move(other_.m_values_string_))
{
} 

Parameter& Parameter::operator=(Parameter&&  other_) OMG_NOEXCEPT {
    Parameter tmp(std::move(other_));
    swap(tmp); 
    return *this;
}
#endif
#endif   

void Parameter::swap(Parameter& other_)  OMG_NOEXCEPT 
{
    using std::swap;
    swap(m_name_, other_.m_name_);
    swap(m_type_, other_.m_type_);
    swap(m_value_bool_, other_.m_value_bool_);
    swap(m_values_int32_, other_.m_values_int32_);
    swap(m_values_double_, other_.m_values_double_);
    swap(m_values_string_, other_.m_values_string_);
}  

bool Parameter::operator == (const Parameter& other_) const {
    if (m_name_ != other_.m_name_) {
        return false;
    }
    if (m_type_ != other_.m_type_) {
        return false;
    }
    if (m_value_bool_ != other_.m_value_bool_) {
        return false;
    }
    if (m_values_int32_ != other_.m_values_int32_) {
        return false;
    }
    if (m_values_double_ != other_.m_values_double_) {
        return false;
    }
    if (m_values_string_ != other_.m_values_string_) {
        return false;
    }
    return true;
}
bool Parameter::operator != (const Parameter& other_) const {
    return !this->operator ==(other_);
}

// --- Getters and Setters: -------------------------------------------------
dds::core::string& Parameter::name() OMG_NOEXCEPT {
    return m_name_;
}

const dds::core::string& Parameter::name() const OMG_NOEXCEPT {
    return m_name_;
}

void Parameter::name(const dds::core::string& value) {
    m_name_ = value;
}

ParameterType& Parameter::type() OMG_NOEXCEPT {
    return m_type_;
}

const ParameterType& Parameter::type() const OMG_NOEXCEPT {
    return m_type_;
}

void Parameter::type(const ParameterType& value) {
    m_type_ = value;
}

bool Parameter::value_bool() const OMG_NOEXCEPT{
    return m_value_bool_;
}

void Parameter::value_bool(bool value) {
    m_value_bool_ = value;
}

dds::core::vector<int32_t>& Parameter::values_int32() OMG_NOEXCEPT {
    return m_values_int32_;
}

const dds::core::vector<int32_t>& Parameter::values_int32() const OMG_NOEXCEPT {
    return m_values_int32_;
}

void Parameter::values_int32(const dds::core::vector<int32_t>& value) {
    m_values_int32_ = value;
}

dds::core::vector<double>& Parameter::values_double() OMG_NOEXCEPT {
    return m_values_double_;
}

const dds::core::vector<double>& Parameter::values_double() const OMG_NOEXCEPT {
    return m_values_double_;
}

void Parameter::values_double(const dds::core::vector<double>& value) {
    m_values_double_ = value;
}

dds::core::string& Parameter::values_string() OMG_NOEXCEPT {
    return m_values_string_;
}

const dds::core::string& Parameter::values_string() const OMG_NOEXCEPT {
    return m_values_string_;
}

void Parameter::values_string(const dds::core::string& value) {
    m_values_string_ = value;
}

std::ostream& operator << (std::ostream& o,const Parameter& sample)
{
    rti::util::StreamFlagSaver flag_saver (o);
    o <<"[";
    o << "name: " << sample.name()<<", ";
    o << "type: " << sample.type()<<", ";
    o << "value_bool: " << sample.value_bool()<<", ";
    o << "values_int32: " << sample.values_int32()<<", ";
    o << "values_double: " << sample.values_double()<<", ";
    o << "values_string: " << sample.values_string() ;
    o <<"]";
    return o;
}

// --- Type traits: -------------------------------------------------

namespace rti { 
    namespace topic {

        const dds::core::xtypes::EnumType& dynamic_type<ParameterType>::get()
        {
            return static_cast<const dds::core::xtypes::EnumType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(ParameterType_c_get_typecode())));
        }

        const dds::core::xtypes::StructType& dynamic_type<Parameter>::get()
        {
            return static_cast<const dds::core::xtypes::StructType&>(
                rti::core::native_conversions::cast_from_native<dds::core::xtypes::DynamicType>(
                    *(Parameter_c_get_typecode())));
        }

    }
}  

namespace dds { 
    namespace topic {
        void topic_type_support<Parameter>:: register_type(
            dds::domain::DomainParticipant& participant,
            const std::string& type_name){

            rti::domain::register_type_plugin(
                participant,
                type_name,
                Parameter_cPlugin_new,
                Parameter_cPlugin_delete);
        }

        void topic_type_support<Parameter>::initialize_sample(Parameter& sample){

            Parameter_c* native_sample=reinterpret_cast<Parameter_c*> (&sample);

            struct DDS_TypeDeallocationParams_t deAllocParams = {RTI_FALSE, RTI_FALSE};
            Parameter_c_finalize_w_params(native_sample,&deAllocParams);

            struct DDS_TypeAllocationParams_t allocParams = {RTI_FALSE, RTI_FALSE, RTI_TRUE}; 
            RTIBool ok=Parameter_c_initialize_w_params(native_sample,&allocParams);
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to initialize_w_params");

        } 

        std::vector<char>& topic_type_support<Parameter>::to_cdr_buffer(
            std::vector<char>& buffer, const Parameter& sample)
        {
            // First get the length of the buffer
            unsigned int length = 0;
            RTIBool ok = Parameter_cPlugin_serialize_to_cdr_buffer(
                NULL, &length,reinterpret_cast<const Parameter_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to calculate cdr buffer size");

            // Create a vector with that size and copy the cdr buffer into it
            buffer.resize(length);
            ok = Parameter_cPlugin_serialize_to_cdr_buffer(
                &buffer[0], &length, reinterpret_cast<const Parameter_c*>(&sample));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to copy cdr buffer");

            return buffer;

        }

        void topic_type_support<Parameter>::from_cdr_buffer(Parameter& sample, 
        const std::vector<char>& buffer)
        {

            RTIBool ok  = Parameter_cPlugin_deserialize_from_cdr_buffer(
                reinterpret_cast<Parameter_c*> (&sample), &buffer[0], 
                static_cast<unsigned int>(buffer.size()));
            rti::core::check_return_code(
                ok ? DDS_RETCODE_OK : DDS_RETCODE_ERROR,
                "Failed to create Parameter from cdr buffer");
        }

    }
}  

