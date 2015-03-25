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
#include "net_util.h"

#define IFDOWN 0
#define IFUP 1

typedef struct addr_ll {
  int active;
  char if_name[10];
  struct sockaddr_in addr;
  struct addr_ll * next;
  int sk;
}addr_ll;

/*
 * Function declarations
 */
void sendpkt( Data_Pckt* pkt, int sk, struct sockaddr_in remote, struct sockaddr_in path );
addr_ll* discover_interfaces();
void free_interface_list( struct addr_ll *addr_list );

#endif
