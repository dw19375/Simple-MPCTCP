/*
 * net_util.c
 * 
 * Contains various network related utility functions
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/time.h>
#include <assert.h>
#include <math.h>
#include "net_util.h"
#include "util.h"

void create_tcp_socket(int* sk, char* ip_addr, int port)
{
    struct addrinfo hints, *servinfo, *p;
    ssize_t rv;
    char port_str[6];
    
    sprintf(port_str, "%d",port);
    
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    
    if ((rv = getaddrinfo(ip_addr, port_str, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: Address: %s, Port: %d\n",config.local_ip,config.socks_port);
        exit(EXIT_FAILURE);
    }
    
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((*sk = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            perror("Failure to create TCP socket");
            continue;
        }
        
        if ((bind(*sk, p->ai_addr, p->ai_addrlen)) == -1) {
            close(*sk);
            perror("Failure to bind TCP socket");
        }
        break;
    }
    
    if (p == NULL) {
        fprintf(stderr, "Failed to bind tcp socket on %s:%d\n",ip_addr,port);
        close(*sk);
        exit(EXIT_FAILURE);
    }
    
    freeaddrinfo(servinfo);
    return;
}

void create_udp_socket(int* sk, char* ip_addr, int port)
{
    struct addrinfo hints, *servinfo, *p;
    ssize_t rv;
    char port_str[6];
    const int on=1;
    
    sprintf(port_str, "%d",port);
    
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    
    if ((rv = getaddrinfo(ip_addr, port_str, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: Address: %s, Port %d\n",config.local_ip,config.socks_port);
        exit(EXIT_FAILURE);
    }
    
    for (p = servinfo; p != NULL; p = p->ai_next) {
        if ((*sk = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            perror("Failure to create UDP socket");
            continue;
        }
        
        setsockopt(*sk, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
        setsockopt(*sk, IPPROTO_IP, IP_PKTINFO, &on, sizeof(on));
        
//         if ((bind(*sk, p->ai_addr, p->ai_addrlen)) == -1) {
//             close(*sk);
//             perror("Failure to bind UDP socket");
//         }
        break;
    }
    
    if (p == NULL) {
        fprintf(stderr, "Failed to bind UDP socket on %s:%d\n",ip_addr,port);
        close(*sk);
        exit(EXIT_FAILURE);
    }
    
    freeaddrinfo(servinfo);
    return;
}

/*
 * Creates a new, empty packet in dynamic memory.  The packet
 * and payload are malloc'ed together with pkt->buf pointing
 * to the memory address right after it.
 * 
 * NOTE: This uses malloc, so please call delete_pkt somewhere on 
 * the Data_Pckt allocated.
 * 
 * NOTE: If you intend to use this packet in a pslist, then 
 * use the create_pslist_elem function instead, which allocates
 * all the memory for the packet, buffer, and list structs all
 * at once.
 * 
 * Inputs:
 *  buflen - Length of payload buffer to allocate in bytes.  If 
 *           buflen == 0, then Data_Pckt->buf is NULL.
 * 
 * Returns a new Data_Pckt which is empty except for buf, which
 * points to the payload buffer.
 */
Data_Pckt* create_pkt( uint32_t buflen )
{
  Data_Pckt* ret = NULL;
  
  ret = (Data_Pckt*)malloc(sizeof(Data_Pckt) + buflen);
  
  if( NULL != ret )
  {
    ret->payload_len = buflen;
    
    if( buflen > 0 )
    {
      ret->buf = ((char*)ret) + sizeof(Data_Pckt);
    }
    else
    {
      ret->buf = NULL;
    }
  }
  
  return ret;
}

/*
 * Frees all memory associated with a packet.
 * 
 * Input:
 *  pkt - Data_Pckt to free.
 */
void delete_pkt( Data_Pckt* pkt )
{
  if( NULL != pkt )
  {
    free( pkt );
  }
}


void
htonpData(Data_Pckt* msg)
{
  if( NULL != msg )
  {
    // Let's not worry about doubles, since they should be in
    // IEEE 754 standard format anyway.
    msg->flag         = htonl(msg->flag);
    msg->seqno        = htonl(msg->seqno);
    msg->num_packets  = htons(msg->num_packets);
    msg->coeff_seed   = htons(msg->coeff_seed);
  }
  // Data is all in bytes, so don't need to do anything
}

void
ntohpData(Data_Pckt* msg)
{
  if( NULL != msg )
  {
    // Let's not worry about doubles, since they should be in
    // IEEE 754 standard format anyway.
    msg->flag         = ntohl(msg->flag);
    msg->seqno        = ntohl(msg->seqno);
    msg->num_packets  = ntohs(msg->num_packets);
    msg->coeff_seed   = ntohs(msg->coeff_seed);
  }
  // Data is all in bytes, so don't need to do anything
}

/*
 * pkt2iovec
 * 
 * Fills the elements of iov with appropriate data from pkt.
 * 
 * Parameters:
 *    pkt   - Pointer to Data_Pckt to be read from
 *    iov   - Pointer to start of struct iovec array to write to.
 * 
 * If pkt is NULL, nothing is written into elements of iov 
 * except the lengths of the data to be filled in.  In essence,
 * this writes a struct iovec for use with readv.
 * 
 * The array iov must contain as many elements as there are in
 * the Data_Pckt struct.  However, if pkt->buf is NULL, iov may
 * contain one fewer element.
 * 
 * Does not change byte order.
 * 
 * Returns number of elements written.
 */
int pkt2iovec( Data_Pckt *pkt, struct iovec *iov )
{
  int idx = 0;
  
  if( NULL != iov )
  {
    /*
     * Write appropriate data lengths into iov
     */
    (iov[idx++]).iov_len = sizeof(pkt->tstamp);
    (iov[idx++]).iov_len = sizeof(pkt->flag);
    (iov[idx++]).iov_len = sizeof(pkt->seqno);
    (iov[idx++]).iov_len = sizeof(pkt->payload_len);
    (iov[idx++]).iov_len = sizeof(pkt->num_packets);
    (iov[idx++]).iov_len = sizeof(pkt->coeff_seed);
    
    // If generating an empty iovec for reading, fill payload
    // length with large number - it may not all be used.
#warning Using statically defined MSS
    (iov[idx++]).iov_len = MSS;
    
    idx = 0;
    
    if( NULL != pkt )
    {
      (iov[idx++]).iov_base = &(pkt->tstamp);
      (iov[idx++]).iov_base = &(pkt->flag);
      (iov[idx++]).iov_base = &(pkt->seqno);
      (iov[idx++]).iov_base = &(pkt->payload_len);
      (iov[idx++]).iov_base = &(pkt->num_packets);
      (iov[idx++]).iov_base = &(pkt->coeff_seed);
      
      if( NULL != pkt->buf )
      {
        (iov[idx]).iov_base = pkt->buf;
        (iov[idx++]).iov_len = pkt->payload_len;
      }
    }
  }
  
  return idx;
}

/*
 * iovec2pkt
 * 
 * Reads an iovec into a Data_Pckt
 * 
 * Inputs:
 *    pkt   - pointer to Data_Pckt to write into.
 *    iov   - pointer to array of iovec to read from.
 * 
 * Returns number of elements read from iov
 * 
 * The array iov must contain as many elements as the struct 
 * Data_Pckt has, however if payload_len is zero, iov may contain 
 * one fewer element.
 * 
 * Does not change byte order.
 */
int iovec2pkt( Data_Pckt *pkt, struct iovec *iov )
{
  int idx = 0;
  
  if( NULL != pkt && NULL != iov )
  {
    pkt->tstamp = *((double*)(iov[idx++].iov_base));
    pkt->flag = *((uint32_t*)(iov[idx++].iov_base));
    pkt->seqno = *((uint32_t*)(iov[idx++].iov_base));
    pkt->payload_len = *((uint32_t*)(iov[idx++].iov_base));
    pkt->num_packets = *((uint16_t*)(iov[idx++].iov_base));
    pkt->coeff_seed = *((uint16_t*)(iov[idx++].iov_base));
    
    if( pkt->payload_len > 0 )
    {
      pkt->buf = (char*)(iov[idx++].iov_base);
    }
  }
  
  return idx;
}

/* 
 * Old Code from Coded-TCP
 */
/*
void
prettyPrint(char** coeffs, int window){
  int i,j;
  printf("\n\n**Matrix**\n");
  for(i = 0; i < BLOCK_SIZE; i++)
  {
    for(j = 0; j < BLOCK_SIZE; j++)
    {
      if(j < i || j >= i + window)
      {
        printf("0 ");
      }
      else
      {
        printf("%d ", (uint8_t) coeffs[i][j-i]);
      }
    }
    printf("\n");
  }
}
*/
