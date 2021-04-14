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
 * \brief This module provides an interface for computing the cyclic redundancy check (crc).
 *        By computing the crc and sending it with the original message the receiver is able
 *        to detect transmission errors by recomputing the crc on the received message.
 * 
 * \ingroup low_level_controller
 */ 

#include <stdint.h>


typedef uint16_t crc;

/**
 * \brief Initializes crc
 */
void  crcInit(void);

/**
 * \brief Computes crc on a message
 * 
 * \param message message for which the crc should be computed
 * \param nBytes length of message
 * 
 * \return the message's crc
 */
crc   crcFast(uint8_t const message[], int nBytes);


#endif /* _crc_h */