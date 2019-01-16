

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ParameterImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef ParameterImpl_418408554_h
#define ParameterImpl_418408554_h

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_c_h
#include "ndds/ndds_c.h"
#endif
#else
#include "ndds_standalone_type.h"
#endif

typedef enum ParameterType_c
{
    ParameterType_c_Invalid  = 0,      
    ParameterType_c_Bool ,      
    ParameterType_c_Int32 ,      
    ParameterType_c_Double ,      
    ParameterType_c_String ,      
    ParameterType_c_Vector_Int32 ,      
    ParameterType_c_Vector_Double ,      
    ParameterType_c_Vector_String      
} ParameterType_c;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* ParameterType_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(ParameterType_cSeq, ParameterType_c);

NDDSUSERDllExport
RTIBool ParameterType_c_initialize(
    ParameterType_c* self);

NDDSUSERDllExport
RTIBool ParameterType_c_initialize_ex(
    ParameterType_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool ParameterType_c_initialize_w_params(
    ParameterType_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void ParameterType_c_finalize(
    ParameterType_c* self);

NDDSUSERDllExport
void ParameterType_c_finalize_ex(
    ParameterType_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void ParameterType_c_finalize_w_params(
    ParameterType_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void ParameterType_c_finalize_optional_members(
    ParameterType_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool ParameterType_c_copy(
    ParameterType_c* dst,
    const ParameterType_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

extern const char *Parameter_cTYPENAME;

typedef struct Parameter_c {

    DDS_Char *   name ;
    ParameterType_c   type ;
    DDS_Boolean   value_bool ;
    struct    DDS_LongSeq  values_int32 ;
    struct    DDS_DoubleSeq  values_double ;
    DDS_Char *   values_string ;

    Parameter_c() {}

} Parameter_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* Parameter_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(Parameter_cSeq, Parameter_c);

NDDSUSERDllExport
RTIBool Parameter_c_initialize(
    Parameter_c* self);

NDDSUSERDllExport
RTIBool Parameter_c_initialize_ex(
    Parameter_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool Parameter_c_initialize_w_params(
    Parameter_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void Parameter_c_finalize(
    Parameter_c* self);

NDDSUSERDllExport
void Parameter_c_finalize_ex(
    Parameter_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void Parameter_c_finalize_w_params(
    Parameter_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void Parameter_c_finalize_optional_members(
    Parameter_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool Parameter_c_copy(
    Parameter_c* dst,
    const Parameter_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif /* ParameterImpl */

