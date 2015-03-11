/*
 * mpctcp_cli.c
 * 
 * While the proxy is bidirectional, this handles the "client" side
 * of things.
 * 
 * Functions to receive and decode packets.
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "net_util.h"
#include "mpctcp_cli.h"

/*
 * handle_packet
 * 
 * Handles incoming packets.  Packets should include the CTCP header
 * but be stripped of any IP or UDP headers.  Everything should be in
 * network byte order.
 * 
 * This converts the struct iovec into a data packet and converts 
 * everything to host byte order, then calls the appropriate
 * handler function.
 * 
 * Parameters:
 *    iov     Pointer to iovec containing packet (header + payload)
 *            Assumed to be in network byte order
 */
void handle_packet( struct iovec *iov )
{
  Data_Pckt pkt;
  int ret;
  
  // Convert iovec to packet
  ret = iovec2pkt( &pkt, iov );
  
  // Switch everything to host byte order
  ntohpData( &pkt );
  
  // Make sure we have a packet to read from
  if( ret )
  {
    switch (pkt.flag)
    {
      case NORMAL:
        handle_data_pkt( &pkt );
        break;
      case SYN:
        handle_syn( &pkt );
        break;
      case SYN_ACK:
        handle_syn_ack( &pkt );
        break;
      case FIN:
        handle_fin( &pkt );
        break;
      case FIN_ACK:
        handle_fin_ack( &pkt );
        break;
      case CODED:
        handle_coded_pkt( &pkt );
        break;
      case ACK:
        handle_ack( &pkt );
        break;
      default:
        printf("Unknown packet type: %u\n", pkt.flag);
        break;
    }
  } 
}

void handle_data_pkt( Data_Pckt* pkt )
{
  
}

void handle_syn( Data_Pckt* pkt )
{
  
}

void handle_syn_ack( Data_Pckt* pkt )
{
  
}

void handle_fin( Data_Pckt* pkt )
{
  
}

void handle_fin_ack( Data_Pckt* pkt )
{
  
}

void handle_coded_pkt( Data_Pckt* pkt )
{
  
}

void handle_ack( Data_Pckt* pkt )
{
  
}


