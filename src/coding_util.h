//
//  coding_util.h
//  Coded-TCP-Multipath-Xcode
//
//  Created by Jason Cloud on 1/20/15.
//  Copyright (c) 2015 MIT. All rights reserved.
//

#ifndef CODING_UTIL_H
#define CODING_UTIL_H

#include <stdio.h>

#define MIN(x,y) (y)^(((x) ^ (y)) &  - ((x) < (y)))
#define MAX(x,y) (y)^(((x) ^ (y)) & - ((x) > (y)))

/* Function declarations */
uint8_t FFmult(uint8_t x, uint8_t y);
uint8_t xFFlog(uint8_t x);
uint8_t fastFFmult(uint8_t x, uint8_t logy);
void seedfastrand(uint32_t seed);
uint32_t fastrand();
int getRandomNumber();

#endif /* defined(CODING_UTIL_H) */
