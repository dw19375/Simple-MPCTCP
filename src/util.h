/*
 * util.h
 * 
 * Various utility functions.
 */

#ifndef UTIL_H
#define UTIL_H

#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <string.h>
#include <stdint.h>
#ifdef __MACH__
#include <sys/time.h>
#endif
#include "default_config.h"

#define TRUE 1
#define FALSE 0

typedef int bool;
typedef struct timeval timeval_t;

/* Function declarations */
void print_help();
int read_config();
double getTime(void);


//portable clock_gettime
#define CLOCK_REALTIME  0 // dummy constant
int clock_gettime(int dummy, struct timespec* t);

#endif /* UTIL_H */ 
