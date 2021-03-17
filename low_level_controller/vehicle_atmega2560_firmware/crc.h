/**********************************************************************
 *
 * Filename:    crc.h
 * 
 * Description: A header file describing the various CRC standards.
 *
 * Notes:       
 *
 * 
 * Copyright (c) 2000 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/

#ifndef _crc_h
#define _crc_h

/**
 * \file crc.h
 *
 * \author Copyright (c) 2000 by Michael Barr
 * 
 * \ingroup low_level_controller
 */ 

#include <stdint.h>


typedef uint16_t crc;

void  crcInit(void);
crc   crcFast(uint8_t const message[], int nBytes);


#endif /* _crc_h */