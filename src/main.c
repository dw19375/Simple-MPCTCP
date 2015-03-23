/*
 * Entry point for SMPCTCP, a simplified MPCTCP implementation, which only
 * sends a file to the client.
 * 
 * Author: Dan Whisman (whisman@mit.edu)
 */

#include <stdlib.h>
#include <stdio.h>
#include "smpctcp_srv.h"
#include "smpctcp_cli.h"

int main( int argc, char *argv[] )
{
  int retval = 0; 
  
  struct addr_ll *addr_list = NULL;
  struct addr_ll *curr = NULL;
  char ipstr[INET6_ADDRSTRLEN];
  
  addr_list = discover_interfaces( );
  
  if( NULL != addr_list )
  {
    curr = addr_list;
    while ( addr_list != NULL )
    {
      if (curr->active == IFUP)
      {
        inet_ntop(curr->addr.sin_family,&(curr->addr.sin_addr),ipstr,sizeof(ipstr));
        printf("%s: %s\n",curr->if_name,ipstr);
      }
      
      curr = curr->next;
      if (curr == NULL)
        break;
    }
  }
  
  free_interface_list( addr_list );
  
  return retval;
}
