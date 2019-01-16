

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ParameterRequest.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef ParameterRequest_1867777981_hpp
#define ParameterRequest_1867777981_hpp

#include <iosfwd>
#include "ParameterRequestImpl.h"

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

class NDDSUSERDllExport ParameterRequest {

  public:
    ParameterRequest();

    explicit ParameterRequest(
        const dds::core::string& name_param);

    #ifdef RTI_CXX11_RVALUE_REFERENCES
    #ifndef RTI_CXX11_NO_IMPLICIT_MOVE_OPERATIONS
    ParameterRequest (ParameterRequest&&) = default;
    ParameterRequest& operator=(ParameterRequest&&) = default;
    ParameterRequest& operator=(const ParameterRequest&) = default;
    ParameterRequest(const ParameterRequest&) = default;
    #else
    ParameterRequest(ParameterRequest&& other_) OMG_NOEXCEPT;  
    ParameterRequest& operator=(ParameterRequest&&  other_) OMG_NOEXCEPT;
    #endif
    #endif 

    dds::core::string& name() OMG_NOEXCEPT; 
    const dds::core::string& name() const OMG_NOEXCEPT;
    void name(const dds::core::string& value);

    bool operator == (const ParameterRequest& other_) const;
    bool operator != (const ParameterRequest& other_) const;

    void swap(ParameterRequest& other_) OMG_NOEXCEPT;

  private:

    dds::core::string m_name_;

};

inline void swap(ParameterRequest& a, ParameterRequest& b)  OMG_NOEXCEPT 
{
    a.swap(b);
}

NDDSUSERDllExport std::ostream& operator<<(std::ostream& o,const ParameterRequest& sample);

namespace dds { 
    namespace topic {

        template<>
        struct topic_type_name<ParameterRequest> {
            NDDSUSERDllExport static std::string value() {
                return "ParameterRequest";
            }
        };

        template<>
        struct is_topic_type<ParameterRequest> : public dds::core::true_type {};

        template<>
        struct topic_type_support<ParameterRequest> {

            NDDSUSERDllExport static void initialize_sample(ParameterRequest& sample);

            NDDSUSERDllExport static void register_type(
                dds::domain::DomainParticipant& participant,
                const std::string & type_name);

            NDDSUSERDllExport static std::vector<char>& to_cdr_buffer(
                std::vector<char>& buffer, const ParameterRequest& sample);

            NDDSUSERDllExport static void from_cdr_buffer(ParameterRequest& sample, const std::vector<char>& buffer);

            static const rti::topic::TypePluginKind::type type_plugin_kind = 
            rti::topic::TypePluginKind::NON_STL;
        };

    }
}

namespace rti { 
    namespace topic {
        template<>
        struct dynamic_type<ParameterRequest> {
            typedef dds::core::xtypes::StructType type;
            NDDSUSERDllExport static const dds::core::xtypes::StructType& get();
        };

        template<>
        struct impl_type<ParameterRequest> {
            typedef ParameterRequest_c type;
        };

    }
}

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif // ParameterRequest_1867777981_hpp

