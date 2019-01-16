

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ParameterRequestImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef ParameterRequestImpl_1867777981_h
#define ParameterRequestImpl_1867777981_h

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_c_h
#include "ndds/ndds_c.h"
#endif
#else
#include "ndds_standalone_type.h"
#endif

extern const char *ParameterRequest_cTYPENAME;

typedef struct ParameterRequest_c {

    DDS_Char *   name ;

    ParameterRequest_c() {}

} ParameterRequest_c ;
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif

NDDSUSERDllExport DDS_TypeCode* ParameterRequest_c_get_typecode(void); /* Type code */

DDS_SEQUENCE(ParameterRequest_cSeq, ParameterRequest_c);

NDDSUSERDllExport
RTIBool ParameterRequest_c_initialize(
    ParameterRequest_c* self);

NDDSUSERDllExport
RTIBool ParameterRequest_c_initialize_ex(
    ParameterRequest_c* self,RTIBool allocatePointers,RTIBool allocateMemory);

NDDSUSERDllExport
RTIBool ParameterRequest_c_initialize_w_params(
    ParameterRequest_c* self,
    const struct DDS_TypeAllocationParams_t * allocParams);  

NDDSUSERDllExport
void ParameterRequest_c_finalize(
    ParameterRequest_c* self);

NDDSUSERDllExport
void ParameterRequest_c_finalize_ex(
    ParameterRequest_c* self,RTIBool deletePointers);

NDDSUSERDllExport
void ParameterRequest_c_finalize_w_params(
    ParameterRequest_c* self,
    const struct DDS_TypeDeallocationParams_t * deallocParams);

NDDSUSERDllExport
void ParameterRequest_c_finalize_optional_members(
    ParameterRequest_c* self, RTIBool deletePointers);  

NDDSUSERDllExport
RTIBool ParameterRequest_c_copy(
    ParameterRequest_c* dst,
    const ParameterRequest_c* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif

#endif /* ParameterRequestImpl */

