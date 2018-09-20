/*
 * util.h
 *
 * Created: 20.09.2018 09:39:30
 *  Author: maczijewski
 */ 


#ifndef UTIL_H_
#define UTIL_H_



#define SET_BIT(p,n) ((p) |= (1 << (n)))
#define CLEAR_BIT(p,n) ((p) &= ~((1) << (n)))
#define TOGGLE_BIT(p,n) ((p) ^= (1 << (n)))



#endif /* UTIL_H_ */