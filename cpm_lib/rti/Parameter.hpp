

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from Parameter.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef Parameter_418408554_hpp
#define Parameter_418408554_hpp

#include <iosfwd>
#include "ParameterImpl.h"

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef RTIUSERDllExport
#define RTIUSERDllExport __declspec(dllexport)
#endif

#include "dds/domain/DomainParticipant.hpp"
#include "dds/topic/TopicTraits.hpp"
#include "dds/core/SafeEnumeration.hpp"
#include "dds/core/String.hpp"
#include "dds/core/array.hpp"
#include "dds/core/vector.hpp"
#include "dds/core/Optional.hpp"
#include "dds/core/xtypes/DynamicType.hpp"
#include "dds/core/xtypes/StructType.hpp"
#include "dds/core/xtypes/UnionType.hpp"
#include "dds/core/xtypes/EnumType.hpp"
#include "dds/core/xtypes/AliasType.hpp"
#include "rti/core/array.hpp"
#include "rti/util/StreamFlagSaver.hpp"
#include "rti/domain/PluginSupport.hpp"
#include "rti/core/LongDouble.hpp"
#include "rti/core/Pointer.hpp"
#include "rti/topic/TopicTraits.hpp"
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef RTIUSERDllExport
#define RTIUSERDllExport
#endif

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

struct ParameterType_def {
    enum type {
        Invalid = 0,      
        Bool,      
        Int32,      
        Double,      
        String,      
        Vector_Int32,      
        Vector_Double,      
        Vector_String     
    };
    static type get_default(){ return Invalid;}
};

typedef dds::core::safe_enum<ParameterType_def> ParameterType;
NDDSUSERDllExport std::ostream& operator << (std::ostream& o,const ParameterType& sample);

class NDDSUSERDllExport Parameter {

  public:
    Parameter();

    Parameter(
        const dds::core::string& name_param,
        const ParameterType& type_param,
        bool value_bool_param,
        const dds::core::vector<int32_t>& values_int32_param,
        const dds::core::vector<double>& values_double_param,
        const dds::core::string& values_string_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    Parameter (Parameter&&) = default;
    Parameter& operator=(Parameter&&) = default;
    Parameter& operator=(const Parameter&) = default;
    Parameter(const Parameter&) = default;
    #else
    Parameter(Parameter&& other_) OMG_NOEXCEPT;  
    Parameter& operator=(Parameter&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    dds::core::string& name() OMG_NOEXCEPT; 
    const dds::core::string& name() const OMG_NOEXCEPT;
    void name(const dds::core::string& value);

    ParameterType& type() OMG_NOEXCEPT; 
    const ParameterType& type() const OMG_NOEXCEPT;
    void type(const ParameterType& value);

    bool value_bool() const OMG_NOEXCEPT;
    void value_bool(bool value);

    dds::core::vector<int32_t>& values_int32() OMG_NOEXCEPT; 
    const dds::core::vector<int32_t>& values_int32() const OMG_NOEXCEPT;
    void values_int32(const dds::core::vector<int32_t>& value);

    dds::core::vector<double>& values_double() OMG_NOEXCEPT; 
    const dds::core::vector<double>& values_double() const OMG_NOEXCEPT;
    void values_double(const dds::core::vector<double>& value);

    dds::core::string& values_string() OMG_NOEXCEPT; 
    const dds::core::string& values_string() const OMG_NOEXCEPT;
    void values_string(const dds::core::string& value);

    bool operator == (const Parameter& other_) const;
    bool operator != (const Parameter& other_) const;

    void swap(Parameter& other_) OMG_NOEXCEPT;

  private:

    dds::core::string m_name_;
    ParameterType m_type_;
    bool m_value_bool_;
    dds::core::vector<int32_t> m_values_int32_;
    dds::core::vector<double> m_values_double_;
    dds::core::string m_values_string_;

};

inline void swap(Parameter& a, Parameter& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const Parameter& sample);

namespace dds { 
    namespace topic {

        template<>
        struct topic_type_name<Parameter> {
            NDDSUSERDllExport static std::string value() {
                return "Parameter";
            }
        };

        template<>
        struct is_topic_type<Parameter> : public dds::core::true_type {};

        template<>
        struct topic_type_support<Parameter> {

            NDDSUSERDllExport static void initialize_sample(Parameter& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const Parameter& sample);

            NDDSUSERDllExport static void from_cdr_buffer(Parameter& sample, const std::vector<char>& buffer);

            static const rti::topic::TypePluginKind::type type_plugin_kind = 
            rti::topic::TypePluginKind::NON_STL;
        };

    }
}

namespace rti { 
    namespace topic {
        template<>
        struct dynamic_type<ParameterType> {
            typedef dds::core::xtypes::EnumType type;
            NDDSUSERDllExport static const dds::core::xtypes::EnumType& get();
        };

        template<>
        struct dynamic_type<Parameter> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<Parameter> {
            typedef Parameter_c type;
        };

    }
}

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif // Parameter_418408554_hpp

