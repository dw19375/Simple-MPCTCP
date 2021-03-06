/*
 * Entry point for SMPCTCP, a simplified MPCTCP implementation, which only
 * sends a file to the client.
 * 
 * Author: Dan Whisman (whisman@mit.edu)
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "smpctcp_srv.h"
#include "smpctcp_cli.h"
#include "util.h"

#define REMOTE_IP "18.62.30.46"
#define REMOTE_PORT 8888

int main( int argc, char *argv[] )
{
  int retval = 0;
  //  int sock;
  int bytes;
  
  char remote_ip[INET6_ADDRSTRLEN];
  int remote_port = REMOTE_PORT;

  char str[16] = "Hello World!";
  
  Data_Pckt* pkt;
  
  struct path_ll *addr_list = NULL;
  struct path_ll *curr = NULL;
  char ipstr[INET6_ADDRSTRLEN];
  
  struct sockaddr_in remote;
  
  /*
   * Parse arguments.
   */
  if( argc == 3 )
  {
    strncpy( remote_ip, argv[1], INET6_ADDRSTRLEN );
    remote_port = atoi( argv[2] );
  }
  else if( argc == 2 )
  {
    strncpy( remote_ip, argv[1], INET6_ADDRSTRLEN );
  }
  else
  {
    printf( "Usage: %s Host Port\n", argv[0] );
    exit(0);
  }
  
  // Set up remote sockaddr_in struct.
  inet_pton(AF_INET, remote_ip, &(remote.sin_addr));
  remote.sin_port = htons(remote_port);
  remote.sin_family = AF_INET;
  
  // Get list of interfaces
  addr_list = discover_interfaces( );
  
  if( NULL != addr_list )
  {
    // Print out list of addresses
    for( curr = addr_list; curr != NULL; curr = curr->next )
    {
      if (curr->active == IFUP)
      {
        inet_ntop( curr->addr.sin_family, &(curr->addr.sin_addr), ipstr, sizeof(ipstr) );
        printf("%s: %s\n",curr->if_name,ipstr);
      }
    }
      
    // Get an empty packet.
    pkt = create_pkt( 16 );
    
    if( NULL != pkt )
    {
      // Set up packet to send.
      pkt->tstamp = getTime();
      pkt->flag = NORMAL;
      pkt->seqno = 0;
      pkt->buf = str;
      
      // Send packet over every interface in addr_list
      for( curr = addr_list; curr != NULL; curr = curr->next )
      {
        // Create a UDP socket
        create_udp_socket( &(curr->sk), remote_ip, remote_port, 0 );

        bytes = sendpkt( pkt, curr->sk, remote, curr->addr );
        printf("Sending %d bytes from %s\n", bytes, 
               inet_ntop( AF_INET, &((curr->addr).sin_addr), ipstr, INET6_ADDRSTRLEN ));

	close( curr->sk );
      }
    }
    
    //    close( sock );
    delete_pkt( pkt );
  }
  
  free_interface_list( addr_list );
  
  return retval;
}
