/*
 * net_util.h 
 * 
 * Contains various network related utilities.
 */

#ifndef NET_UTIL_H
#define NET_UTIL_H

#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <ifaddrs.h>
#include <string.h>
#include <stdint.h>

typedef int socket_t;

#define NETIFDOWN	0
#define NETIFUP		1

#ifndef MSS
#define MSS 1500
#endif

// We have to take into account the IP+UDP header
// Max IP header is 24B, UDP header is 8B 
#define IPUDP_HEADER_SIZE 32 

// Flags for Data_Pckt
// Not using an enum so we can specify our data size on all platforms
#define   NORMAL 0 
#define   SYN 1
#define   SYN_ACK 2
#define   FIN 3
#define   FIN_ACK 4
#define   CODED 5
#define   ACK 6


/*
 * Data struct for packets.
 */
typedef struct
{
  double tstamp;        // Time stamp
  uint32_t flag;        // Flag indicating packet type
  uint32_t seqno;       // Normal packet - sequence number
                        // ACK - packet being acknowledged
                        // Coded - beginning of coding window
  uint32_t payload_len; // Length of payload in bytes
  
  uint16_t num_packets; // Coded - Number of packets in coding window
                        // ACK - Number of dofs requested
                        // Normal Packet - unused
  
  uint16_t coeff_seed;  // Seed for coefficients of coded packets
                        // Used only for coded packets
                        
  char* buf;            // Payload - allocate MSS - sizeof(Pckt_Header)
                        // Don't statically allocate here so we can 
                        // dynamically set MSS
} Data_Pckt;


/* Function declarations */
void create_tcp_socket(int*, char*, int);
void create_udp_socket(int*, char*, int, int);
void htonpData(Data_Pckt *msg);
void ntohpData(Data_Pckt *msg);
Data_Pckt* create_pkt( uint32_t buflen );
void delete_pkt( Data_Pckt* pkt );
int pkt2iovec( Data_Pckt *pkt, struct iovec *iov );
int iovec2pkt( Data_Pckt *pkt, struct iovec *iov );
//void prettyPrint(char** coeffs, int window);

#endif
