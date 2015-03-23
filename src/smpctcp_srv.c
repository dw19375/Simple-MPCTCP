/*
 * mpctcp_srv.c
 * 
 * Functions to listen for incoming connections, send packets, and
 * handle ACKs.
 */

#include "smpctcp_srv.h"

/*
 * Discoveres usable IP addresses on the host machine.  This
 * uses malloc, so call free_interface_list sometime.
 * 
 * Return value:
 *    Pointer to address list created
 * 
 */
struct addr_ll* discover_interfaces()
{
  struct addr_ll addr_list; // List uses sentinel node.
  
  struct ifaddrs *ifaddr, *p;
  int status;
  char ipstr[INET6_ADDRSTRLEN];
  struct addr_ll *curr = NULL;
  struct addr_ll *next = NULL;

  addr_list.active = IFDOWN;
  addr_list.next = NULL;
  
  if ((status = getifaddrs(&ifaddr)) != 0)
  {
    fprintf(stderr, "getifaddrs: %s\n",gai_strerror(status));
  }
  else
  {
    // Go through each of the available IP addresses and store them.
    for (p = ifaddr; p!=NULL; p=p->ifa_next)
    {
      if (p->ifa_addr != NULL)
      {
        if (p->ifa_addr->sa_family == AF_INET)
        {
          //Determine if address is the loopback or is already registered as active
          int ignore = 0;
          
          struct sockaddr_in *ipv4 = (struct sockaddr_in *)p->ifa_addr;
          inet_ntop(AF_INET, &ipv4->sin_addr, ipstr, sizeof(ipstr));
          
          if (strcmp(ipstr, "127.0.0.1"))
          {
            for (curr = &addr_list; curr->next != NULL; curr = curr->next)
            {
              if (curr->active == IFUP && &(curr->addr.sin_addr) == &(ipv4->sin_addr))
              {
                ignore = 1;
                break;
              }
            }
          }
          else
          {
            ignore = 1;
          }
          
          //If it is not already registered
          if (ignore == 0)
          {
            
            if (curr->active == IFUP)
            {
              next = (struct addr_ll *)malloc(sizeof(struct addr_ll));
              curr->next = next;
            }
            else
            {
              next = curr;
            }
            
            next->active = IFUP;
            next->addr = *ipv4;
            strcpy(next->if_name,p->ifa_name);
            next->next = NULL;
          }
          
        }
      }
    }
  }
  
  freeifaddrs(ifaddr);
  
  return addr_list.next;
}

/*
 * Frees dynamic memory allocated by discover_interfaces.
 * 
 * Parameter:
 *    addr_list - pointer to head of list.
 */
void free_interface_list( struct addr_ll *addr_list )
{
  if( NULL != addr_list )
  {
    if( NULL == addr_list->next )
    {
      free( addr_list );
    }
    else
    {
      free_interface_list( addr_list->next );
    }
  }
}
