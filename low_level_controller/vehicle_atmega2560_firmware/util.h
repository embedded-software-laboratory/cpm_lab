/**
 * \file util.h
 *
 * \author maczijewski
 * \date Created: 20.09.2018 09:39:30
 * 
 * \brief This file provides some utility defines.
 * 
 * \ingroup low_level_controller
 */

#ifndef UTIL_H_
#define UTIL_H_

/**
 * \brief Given a byte p, this define sets the n-th bit.
 * \ingroup low_level_controller
 */
#define SET_BIT(p,n) ((p) |= (1 << (n)))

/**
 * \brief Given a byte p, this define resets the n-th bit.
 * \ingroup low_level_controller
 */
#define CLEAR_BIT(p,n) ((p) &= ~((1) << (n)))

/**
 * \brief Given a byte p, this define toggles the n-th bit.
 * \ingroup low_level_controller
 */
#define TOGGLE_BIT(p,n) ((p) ^= (1 << (n)))


#endif /* UTIL_H_ */