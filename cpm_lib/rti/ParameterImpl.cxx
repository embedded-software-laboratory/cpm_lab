

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ParameterImpl.idl using "rtiddsgen".
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

#include "ParameterImpl.h"

/* ========================================================================= */
const char *ParameterType_cTYPENAME = "ParameterType";

DDS_TypeCode* ParameterType_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode_Member ParameterType_c_g_tc_members[8]=
    {

        {
            (char *)"Invalid",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_Invalid, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"Bool",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_Bool, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"Int32",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_Int32, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"Double",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_Double, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"String",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_String, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"Vector_Int32",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_Vector_Int32, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"Vector_Double",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_Vector_Double, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"Vector_String",/* Member name */
            {
                0, /* Ignored */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            ParameterType_c_Vector_String, 
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Member visibility */ 

            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode ParameterType_c_g_tc =
    {{
            DDS_TK_ENUM,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"ParameterType", /* Name */
            NULL,     /* Base class type code is assigned later */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            8, /* Number of members */
            ParameterType_c_g_tc_members, /* Members */
            DDS_VM_NONE   /* Type Modifier */        
        }}; /* Type code for ParameterType_c*/

    if (is_initialized) {
        return &ParameterType_c_g_tc;
    }

    is_initialized = RTI_TRUE;

    return &ParameterType_c_g_tc;
}

RTIBool ParameterType_c_initialize(
    ParameterType_c* sample) {
    *sample = ParameterType_c_Invalid;
    return RTI_TRUE;
}

RTIBool ParameterType_c_initialize_ex(
    ParameterType_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return ParameterType_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool ParameterType_c_initialize_w_params(
    ParameterType_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    if (sample == NULL) {
        return RTI_FALSE;
    }
    if (allocParams == NULL) {
        return RTI_FALSE;
    }
    *sample = ParameterType_c_Invalid;
    return RTI_TRUE;
}

void ParameterType_c_finalize(
    ParameterType_c* sample)
{

    if (sample==NULL) {
        return;
    }
}

void ParameterType_c_finalize_ex(
    ParameterType_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    ParameterType_c_finalize_w_params(
        sample,&deallocParams);
}

void ParameterType_c_finalize_w_params(
    ParameterType_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
{

    if (sample==NULL) {
        return;
    }

    if (deallocParams == NULL) {
        return;
    }

}

void ParameterType_c_finalize_optional_members(
    ParameterType_c* sample, RTIBool deletePointers)
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

RTIBool ParameterType_c_copy(
    ParameterType_c* dst,
    const ParameterType_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    return RTICdrType_copyEnum((RTICdrEnum *)dst, (RTICdrEnum *)src);

}

/**
* <<IMPLEMENTATION>>
*
* Defines:  TSeq, T
*
* Configure and implement 'ParameterType_c' sequence class.
*/
#define T ParameterType_c
#define TSeq ParameterType_cSeq

#define T_initialize_w_params ParameterType_c_initialize_w_params

#define T_finalize_w_params   ParameterType_c_finalize_w_params
#define T_copy       ParameterType_c_copy

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

/* ========================================================================= */
const char *Parameter_cTYPENAME = "Parameter";

DDS_TypeCode* Parameter_c_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode Parameter_c_g_tc_name_string = DDS_INITIALIZE_STRING_TYPECODE((255));
    static DDS_TypeCode Parameter_c_g_tc_values_int32_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE((100),NULL);
    static DDS_TypeCode Parameter_c_g_tc_values_double_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE((100),NULL);
    static DDS_TypeCode Parameter_c_g_tc_values_string_string = DDS_INITIALIZE_STRING_TYPECODE((255));
    static DDS_TypeCode_Member Parameter_c_g_tc_members[6]=
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
        }, 
        {
            (char *)"type",/* Member name */
            {
                1,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"value_bool",/* Member name */
            {
                2,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"values_int32",/* Member name */
            {
                3,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"values_double",/* Member name */
            {
                4,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }, 
        {
            (char *)"values_string",/* Member name */
            {
                5,/* Representation ID */          
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
            DDS_PUBLIC_MEMBER,/* Member visibility */
            1,
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode Parameter_c_g_tc =
    {{
            DDS_TK_STRUCT,/* Kind */
            DDS_BOOLEAN_FALSE, /* Ignored */
            -1, /*Ignored*/
            (char *)"Parameter", /* Name */
            NULL, /* Ignored */      
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            6, /* Number of members */
            Parameter_c_g_tc_members, /* Members */
            DDS_VM_NONE  /* Ignored */         
        }}; /* Type code for Parameter_c*/

    if (is_initialized) {
        return &Parameter_c_g_tc;
    }

    Parameter_c_g_tc_values_int32_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;

    Parameter_c_g_tc_values_double_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_double;

    Parameter_c_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&Parameter_c_g_tc_name_string;

    Parameter_c_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)ParameterType_c_get_typecode();

    Parameter_c_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_boolean;

    Parameter_c_g_tc_members[3]._representation._typeCode = (RTICdrTypeCode *)& Parameter_c_g_tc_values_int32_sequence;
    Parameter_c_g_tc_members[4]._representation._typeCode = (RTICdrTypeCode *)& Parameter_c_g_tc_values_double_sequence;
    Parameter_c_g_tc_members[5]._representation._typeCode = (RTICdrTypeCode *)&Parameter_c_g_tc_values_string_string;

    is_initialized = RTI_TRUE;

    return &Parameter_c_g_tc;
}

RTIBool Parameter_c_initialize(
    Parameter_c* sample) {
    return Parameter_c_initialize_ex(sample,RTI_TRUE,RTI_TRUE);
}

RTIBool Parameter_c_initialize_ex(
    Parameter_c* sample,RTIBool allocatePointers, RTIBool allocateMemory)
{

    struct DDS_TypeAllocationParams_t allocParams =
    DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

    allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
    allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

    return Parameter_c_initialize_w_params(
        sample,&allocParams);

}

RTIBool Parameter_c_initialize_w_params(
    Parameter_c* sample, const struct DDS_TypeAllocationParams_t * allocParams)
{

    void* buffer = NULL;
    if (buffer) {} /* To avoid warnings */

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

    if (!ParameterType_c_initialize_w_params(&sample->type,
    allocParams)) {
        return RTI_FALSE;
    }

    if (!RTICdrType_initBoolean(&sample->value_bool)) {
        return RTI_FALSE;
    }

    if (allocParams->allocate_memory) {
        DDS_LongSeq_initialize(&sample->values_int32  );
        DDS_LongSeq_set_absolute_maximum(&sample->values_int32 , (100));
        if (!DDS_LongSeq_set_maximum(&sample->values_int32 , (100))) {
            return RTI_FALSE;
        }
    } else { 
        DDS_LongSeq_set_length(&sample->values_int32, 0);
    }
    if (allocParams->allocate_memory) {
        DDS_DoubleSeq_initialize(&sample->values_double  );
        DDS_DoubleSeq_set_absolute_maximum(&sample->values_double , (100));
        if (!DDS_DoubleSeq_set_maximum(&sample->values_double , (100))) {
            return RTI_FALSE;
        }
    } else { 
        DDS_DoubleSeq_set_length(&sample->values_double, 0);
    }
    if (allocParams->allocate_memory){
        sample->values_string= DDS_String_alloc ((255));
        if (sample->values_string == NULL) {
            return RTI_FALSE;
        }

    } else {
        if (sample->values_string!= NULL) { 
            sample->values_string[0] = '\0';
        }
    }

    return RTI_TRUE;
}

void Parameter_c_finalize(
    Parameter_c* sample)
{

    Parameter_c_finalize_ex(sample,RTI_TRUE);
}

void Parameter_c_finalize_ex(
    Parameter_c* sample,RTIBool deletePointers)
{
    struct DDS_TypeDeallocationParams_t deallocParams =
    DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

    if (sample==NULL) {
        return;
    } 

    deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

    Parameter_c_finalize_w_params(
        sample,&deallocParams);
}

void Parameter_c_finalize_w_params(
    Parameter_c* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
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
    ParameterType_c_finalize_w_params(&sample->type,deallocParams);

    DDS_LongSeq_finalize(&sample->values_int32);

    DDS_DoubleSeq_finalize(&sample->values_double);

    if (sample->values_string != NULL) {
        DDS_String_free(sample->values_string);
        sample->values_string=NULL;

    }
}

void Parameter_c_finalize_optional_members(
    Parameter_c* sample, RTIBool deletePointers)
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

    ParameterType_c_finalize_optional_members(&sample->type, deallocParams->delete_pointers);
}

RTIBool Parameter_c_copy(
    Parameter_c* dst,
    const Parameter_c* src)
{

    if (dst == NULL || src == NULL) {
        return RTI_FALSE;
    }

    if (!RTICdrType_copyStringEx (
        &dst->name, src->name, 
        (255) + 1,RTI_TRUE)){
        return RTI_FALSE;
    }
    if (!ParameterType_c_copy(
        &dst->type,(const ParameterType_c*)&src->type)) {
        return RTI_FALSE;
    } 
    if (!RTICdrType_copyBoolean (
        &dst->value_bool, &src->value_bool)) { 
        return RTI_FALSE;
    }
    if (!DDS_LongSeq_copy(&dst->values_int32 ,
    &src->values_int32 )) {
        return RTI_FALSE;
    }
    if (!DDS_DoubleSeq_copy(&dst->values_double ,
    &src->values_double )) {
        return RTI_FALSE;
    }
    if (!RTICdrType_copyStringEx (
        &dst->values_string, src->values_string, 
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
* Configure and implement 'Parameter_c' sequence class.
*/
#define T Parameter_c
#define TSeq Parameter_cSeq

#define T_initialize_w_params Parameter_c_initialize_w_params

#define T_finalize_w_params   Parameter_c_finalize_w_params
#define T_copy       Parameter_c_copy

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

