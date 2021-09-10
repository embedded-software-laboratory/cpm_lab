/**
 * \file watchdog.h
 *
 * \date Created: 6/21/2019 13:35:35
 * \author: cfrauzem
 * 
 * \brief The watchdog of the low_level_controller, which can be accessed via this module,
 *        controls the state of the vehicle. If the watchdog is enabled and the
 *        low_level_controller didn't get up-to-date information of the mid_level_controller
 *        for too long, the watchdog will provoke the low_level_controller to be set into
 *        safe-mode.
 * 
 * \ingroup low_level_controller
 */ 

#ifndef WATCHDOG_H_
#define WATCHDOG_H_


#include "spi_packets.h"

extern volatile uint8_t safe_mode_flag;

/**
 * \brief Disable the functionality of the watchdog.
 *
 * \author cfrauzem
 * \ingroup low_level_controller
 */ 
void watchdog_disable();

/**
 * \brief Enable the functionality of the watchdog.
 *
 * \author cfrauzem
 * \ingroup low_level_controller
 */ 
void watchdog_enable();

/**
 * \brief Reset the watchdog to the initial value. Has to be done regularily such that
 *        the safe-mode is not provoked when low_level_controller and mid_level_controller
 *        work together in a proper way.
 *
 * \author cfrauzem
 * \ingroup low_level_controller
 */ 
void watchdog_reset();

#endif /* WATCHDOG_H_ */