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
#include <pthread.h>
#include "net_util.h"
#include "pslist.h"

#define IFDOWN 0
#define IFUP 1

/*
 * Keeps information about a particular path.
 * 
 * NOTE: If you must get both mutexes, lock qlock first!!!
 */
typedef struct path_ll
{
  struct path_ll * next;
  
  // Thread management data
  pthread_t sender;
  pthread_mutex_t qlock;      // Mutex for pktq
  pthread_cond_t qcond;       // Condition variable for packet queue
  pthread_mutex_t data_lock;  // Mutex for other data
  
  // Interface data
  int active;
  char if_name[10];
  struct sockaddr_in addr;
  int sk;         // Socket to send from
  int listen_sk;  // Socket to listen on
  
  // Packet data
  struct sockaddr_in remote;
  Data_Pckt *pkt;     // Packet to send
  int busy;           // Busy flag
  int in_flight;      // Number of packets in flight
  int sent;           // Total number of packets sent
  int lost;           // Total number of packets lost
  double rtt;         // Measured round trip time
  double loss_prob;   // Measured loss probability
} path_ll;

/*
 * Function declarations
 */
int sendpkt( Data_Pckt* pkt, int sk, struct sockaddr_in remote, struct sockaddr_in path );
path_ll* discover_interfaces();
void free_interface_list( struct path_ll *addr_list );
pslist_elem* queue_next_packet( int fd, pslist_elem** q, int n );

#endif
