#ifndef MPCTCP_SRV_H
#define MPCTCP_SRV_H 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <net/if.h>
#include <ifaddrs.h>

#define IFDOWN 0
#define IFUP 1

struct addr_ll {
  int active;
  char if_name[10];
  struct sockaddr_in addr;
  struct addr_ll * next;
  int sk;
};

/*
 * Function declarations
 */
struct addr_ll* discover_interfaces();
void free_interface_list( struct addr_ll *addr_list );

#endif
