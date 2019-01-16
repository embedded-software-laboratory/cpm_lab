

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ParameterImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef ParameterImplPlugin_418408554_h
#define ParameterImplPlugin_418408554_h

#include "ParameterImpl.h"

struct RTICdrStream;

#ifndef pres_typePlugin_h
#include "pres/pres_typePlugin.h"
#endif

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */

NDDSUSERDllExport extern RTIBool 
ParameterType_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const ParameterType_c *sample,
    struct RTICdrStream *stream, 
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
ParameterType_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    ParameterType_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
ParameterType_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream, 
    RTIBool skip_encapsulation,  
    RTIBool skip_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern unsigned int 
ParameterType_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);    

NDDSUSERDllExport extern unsigned int 
ParameterType_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
ParameterType_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int
ParameterType_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const ParameterType_c * sample);

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern unsigned int 
ParameterType_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
ParameterType_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern RTIBool 
ParameterType_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const ParameterType_c *sample,
    struct RTICdrStream *stream,
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
ParameterType_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    ParameterType_c * sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
ParameterType_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    ParameterType_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos);

/* ----------------------------------------------------------------------------
Support functions:
* ---------------------------------------------------------------------------- */

NDDSUSERDllExport extern void
ParameterType_cPluginSupport_print_data(
    const ParameterType_c *sample, const char *desc, int indent_level);

/* The type used to store keys for instances of type struct
* AnotherSimple.
*
* By default, this type is struct Parameter
* itself. However, if for some reason this choice is not practical for your
* system (e.g. if sizeof(struct Parameter)
* is very large), you may redefine this typedef in terms of another type of
* your choosing. HOWEVER, if you define the KeyHolder type to be something
* other than struct AnotherSimple, the
* following restriction applies: the key of struct
* Parameter must consist of a
* single field of your redefined KeyHolder type and that field must be the
* first field in struct Parameter.
*/
typedef  struct Parameter_c Parameter_cKeyHolder;

#define Parameter_cPlugin_get_sample PRESTypePluginDefaultEndpointData_getSample 
#define Parameter_cPlugin_get_buffer PRESTypePluginDefaultEndpointData_getBuffer 
#define Parameter_cPlugin_return_buffer PRESTypePluginDefaultEndpointData_returnBuffer 

#define Parameter_cPlugin_get_key PRESTypePluginDefaultEndpointData_getKey 
#define Parameter_cPlugin_return_key PRESTypePluginDefaultEndpointData_returnKey

#define Parameter_cPlugin_create_sample PRESTypePluginDefaultEndpointData_createSample 
#define Parameter_cPlugin_destroy_sample PRESTypePluginDefaultEndpointData_deleteSample 

/* --------------------------------------------------------------------------------------
Support functions:
* -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern Parameter_c*
Parameter_cPluginSupport_create_data_w_params(
    const struct DDS_TypeAllocationParams_t * alloc_params);

NDDSUSERDllExport extern Parameter_c*
Parameter_cPluginSupport_create_data_ex(RTIBool allocate_pointers);

NDDSUSERDllExport extern Parameter_c*
Parameter_cPluginSupport_create_data(void);

NDDSUSERDllExport extern RTIBool 
Parameter_cPluginSupport_copy_data(
    Parameter_c *out,
    const Parameter_c *in);

NDDSUSERDllExport extern void 
Parameter_cPluginSupport_destroy_data_w_params(
    Parameter_c *sample,
    const struct DDS_TypeDeallocationParams_t * dealloc_params);

NDDSUSERDllExport extern void 
Parameter_cPluginSupport_destroy_data_ex(
    Parameter_c *sample,RTIBool deallocate_pointers);

NDDSUSERDllExport extern void 
Parameter_cPluginSupport_destroy_data(
    Parameter_c *sample);

NDDSUSERDllExport extern void 
Parameter_cPluginSupport_print_data(
    const Parameter_c *sample,
    const char *desc,
    unsigned int indent);

NDDSUSERDllExport extern Parameter_c*
Parameter_cPluginSupport_create_key_ex(RTIBool allocate_pointers);

NDDSUSERDllExport extern Parameter_c*
Parameter_cPluginSupport_create_key(void);

NDDSUSERDllExport extern void 
Parameter_cPluginSupport_destroy_key_ex(
    Parameter_cKeyHolder *key,RTIBool deallocate_pointers);

NDDSUSERDllExport extern void 
Parameter_cPluginSupport_destroy_key(
    Parameter_cKeyHolder *key);

/* ----------------------------------------------------------------------------
Callback functions:
* ---------------------------------------------------------------------------- */

NDDSUSERDllExport extern PRESTypePluginParticipantData 
Parameter_cPlugin_on_participant_attached(
    void *registration_data, 
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration, 
    void *container_plugin_context,
    RTICdrTypeCode *typeCode);

NDDSUSERDllExport extern void 
Parameter_cPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data);

NDDSUSERDllExport extern PRESTypePluginEndpointData 
Parameter_cPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *container_plugin_context);

NDDSUSERDllExport extern void 
Parameter_cPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data);

NDDSUSERDllExport extern void    
Parameter_cPlugin_return_sample(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c *sample,
    void *handle);    

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c *out,
    const Parameter_c *in);

/* ----------------------------------------------------------------------------
(De)Serialize functions:
* ------------------------------------------------------------------------- */

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const Parameter_c *sample,
    struct RTICdrStream *stream, 
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Parameter_cPlugin_serialize_to_cdr_buffer(
    char * buffer,
    unsigned int * length,
    const Parameter_c *sample); 

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Parameter_cPlugin_deserialize_from_cdr_buffer(
    Parameter_c *sample,
    const char * buffer,
    unsigned int length);    
NDDSUSERDllExport extern DDS_ReturnCode_t
Parameter_cPlugin_data_to_string(
    const Parameter_c *sample,
    char *str,
    DDS_UnsignedLong *str_size, 
    const struct DDS_PrintFormatProperty *property);    

NDDSUSERDllExport extern RTIBool
Parameter_cPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream, 
    RTIBool skip_encapsulation,  
    RTIBool skip_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern unsigned int 
Parameter_cPlugin_get_serialized_sample_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);    

NDDSUSERDllExport extern unsigned int 
Parameter_cPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
Parameter_cPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int
Parameter_cPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const Parameter_c * sample);

/* --------------------------------------------------------------------------------------
Key Management functions:
* -------------------------------------------------------------------------------------- */
NDDSUSERDllExport extern PRESTypePluginKeyKind 
Parameter_cPlugin_get_key_kind(void);

NDDSUSERDllExport extern unsigned int 
Parameter_cPlugin_get_serialized_key_max_size_ex(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool * overflow,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern unsigned int 
Parameter_cPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const Parameter_c *sample,
    struct RTICdrStream *stream,
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c * sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c ** sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool
Parameter_cPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_instance_to_key(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_cKeyHolder *key, 
    const Parameter_c *instance);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_key_to_instance(
    PRESTypePluginEndpointData endpoint_data,
    Parameter_c *instance, 
    const Parameter_cKeyHolder *key);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_instance_to_keyhash(
    PRESTypePluginEndpointData endpoint_data,
    DDS_KeyHash_t *keyhash,
    const Parameter_c *instance);

NDDSUSERDllExport extern RTIBool 
Parameter_cPlugin_serialized_sample_to_keyhash(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream, 
    DDS_KeyHash_t *keyhash,
    RTIBool deserialize_encapsulation,
    void *endpoint_plugin_qos); 

/* Plugin Functions */
NDDSUSERDllExport extern struct PRESTypePlugin*
Parameter_cPlugin_new(void);

NDDSUSERDllExport extern void
Parameter_cPlugin_delete(struct PRESTypePlugin *);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif /* ParameterImplPlugin_418408554_h */

