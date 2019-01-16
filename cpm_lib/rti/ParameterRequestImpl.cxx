

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ParameterRequestImpl.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_c_h
#include "ndds/ndds_c.h"
#endif

#ifndef cdr_type_h
#include "cdr/cdr_type.h"
#endif    

#ifndef osapi_heap_h
#include "osapi/osapi_heap.h" 
#endif
#else
#include "ndds_standalone_type.h"
#endif

#include "ParameterRequestImpl.h"

/* ========================================================================= */
const char *ParameterRequest_cTYPENAME = "ParameterRequest";

DDS_TypeCode* ParameterRequest_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode ParameterRequest_c_g_tc_name_string = DDS_INITIALIZE_STRING_TYPECODE((255));
    static DDS_TypeCode_Member ParameterRequest_c_g_tc_members[1]=
    {

        {
            (char *)"name",/* Member name */
            {
                0,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_KEY_MEMBER , /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode ParameterRequest_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"ParameterRequest", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            1, /* Number of members */
            ParameterRequest_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for ParameterRequest_c*/

    if (is_initialized) {
        return &ParameterRequest_c_g_tc;
    }

    ParameterRequest_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&ParameterRequest_c_g_tc_name_string;

    is_initialized = RTI_TRUE;

    return &ParameterRequest_c_g_tc;
}

RTIBool ParameterRequest_c_initialize(
    ParameterRequest_c* sample) {
    return ParameterRequest_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool ParameterRequest_c_initialize_ex(
    ParameterRequest_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return ParameterRequest_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool ParameterRequest_c_initialize_w_params(
    ParameterRequest_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }

    if (allocParams->allocate_memory){
        sample->name= DDS_String_alloc ((255));
        if (sample->name == NULL) {
            return RTI_FALSE;
        }

    } else {
        if (sample->name!= NULL) { 
            sample->name[0] = '\0';
        }
    }

    return RTI_TRUE;
}

void ParameterRequest_c_finalize(
    ParameterRequest_c* sample)
{

    ParameterRequest_c_finalize_ex(sample,RTI_TRUE);
}

void ParameterRequest_c_finalize_ex(
    ParameterRequest_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    ParameterRequest_c_finalize_w_params(
        sample,&deallocParams);
}

void ParameterRequest_c_finalize_w_params(
    ParameterRequest_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

    if (sample->name != NULL) {
        DDS_String_free(sample->name);
        sample->name=NULL;

    }
}

void ParameterRequest_c_finalize_optional_members(
    ParameterRequest_c* sample, RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParamsTmp =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
    struct DDS_TypeDeallocationParams_t * deallocParams =
    &deallocParamsTmp;

    if (sample==NULL) {
        return;
    } 
    if (deallocParams) {} /* To avoid warnings */

    deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
    deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

}

RTIBool ParameterRequest_c_copy(
    ParameterRequest_c* dst,
    const ParameterRequest_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_copyStringEx (
        &dst->name, src->name, 
        (255) + 1,RTI_TRUE)){
        return RTI_FALSE;
    }

    return RTI_TRUE;

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'ParameterRequest_c' sequence class.
*/
#define T ParameterRequest_c
#define TSeq ParameterRequest_cSeq

#define T_initialize_w_params ParameterRequest_c_initialize_w_params

#define T_finalize_w_params   ParameterRequest_c_finalize_w_params
#define T_copy       ParameterRequest_c_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#else
#include "dds_c_sequence_TSeq.gen"
#endif

#undef T_copy
#undef T_finalize_w_params

#undef T_initialize_w_params

#undef TSeq
#undef T

