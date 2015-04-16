/*
 * mpctcp_srv.c
 * 
 * Functions to listen for incoming connections, send packets, and
 * handle ACKs.
 */

#include "smpctcp_srv.h"
#include "pslist.h"
#include "util.h"
#include <unistd.h>
#include <pthread.h>


/*
 * Sender Thread
 * This thread sends packets in its queue and sleeps if the
 * queue is empty.
 * 
 * Input is a pointer to a path_ll struct.  Does not return
 * anything.
 */
void *sender_thread( void* arg )
{
  path_ll* data = (path_ll*)arg;
  
  
  
  pthread_exit( NULL );
}

/*
 * Read from a file and place packet in the given queue.  This is not 
 * thread safe yet!  Note that fd must refer to an open file.
 * 
 * Inputs:
 *    fd - File to read from
 *    q - Pointer to head of list at the beginning of the queue
 *    n - Maximum number of bytes to read from file
 *    
 * Return Value:
 *    Number of bytes read from fd.  Returns -1 on error.
 */
int queue_next_packet( int fd, pslist_elem** q, int n )
{
  int retval = -1;
  pslist_elem* elem;
  static int seqno = 0;
  
  if( NULL != q )
  {
    // Create a new packet - we'll allocate n bytes in the buffer,
    // even if we don't need them.  We'll change it later.
    elem = create_pslist_elem( n );
    
    if( NULL != elem )
    {
      // Read up to n bytes from fd, put in packet buffer
      retval = read( fd, (elem->pkt).buf, n );
      
      if( retval > 0 )
      {
        // We have read some bytes, go ahead and make the packet
        elem->pkt.tstamp = getTime();
        elem->pkt.flag = NORMAL;
        elem->pkt.seqno = seqno++;    // Increments after
        elem->pkt.payload_len = retval;
        elem->pkt.num_packets = 0;
        elem->pkt.coeff_seed = 0;
        
#warning Make this thread safe!
        // Add the element to the queue.
        ins_pslist_elem( q, elem );
      }
    }
  }
  
  return retval;
}

/*
 * Sends a Data_Pckt on a given path to the address and port provided in
 * remote.
 * 
 * Inputs:
 *    pkt - Data_Pckt to send.  pkt should be in host byte order
 *    sk -  Socket to send on
 *    remote - Address/Port to send to.  Currently, only IPv4 is used
 *    path - Address of interface to send from
 */
int sendpkt( Data_Pckt* pkt, int sk, struct sockaddr_in remote, struct sockaddr_in path )
{
  int retval;
  char ipstr[INET6_ADDRSTRLEN];
  
  struct iovec iov[10];
  struct msghdr msg = {0};
  struct cmsghdr *cmsg;
  struct in_pktinfo *pktinfo;
  char control[sizeof(struct cmsghdr)+12];
  
  // Put the packet in network byte order
  htonpData(pkt);
  
  // Put the data into the message iov.
  msg.msg_iovlen = pkt2iovec( pkt, iov );
  msg.msg_iov = iov;
  
  msg.msg_name = &remote;
  msg.msg_namelen = sizeof(remote);
  
  msg.msg_control = control;
  msg.msg_controllen = CMSG_SPACE(sizeof(struct in_pktinfo));
  
  msg.msg_flags = 0;
  
  cmsg = CMSG_FIRSTHDR(&msg);
  cmsg->cmsg_level = IPPROTO_IP;
  cmsg->cmsg_type = IP_PKTINFO;
  cmsg->cmsg_len = CMSG_LEN(sizeof(struct in_pktinfo));
  
  pktinfo = (struct in_pktinfo *)CMSG_DATA(cmsg);
  //pktinfo->ipi_ifindex = if_nametoindex(curr->if_name);
  pktinfo->ipi_ifindex = 0;
  pktinfo->ipi_spec_dst = path.sin_addr;
  pktinfo->ipi_addr = remote.sin_addr;
  
  if ((retval = sendmsg(sk, &msg, 0)) == -1)
  {
    fprintf( stderr, "Failed to send packet via %s\n",
             inet_ntop( AF_INET, &(path.sin_addr), ipstr, INET6_ADDRSTRLEN ));
  }
  
  return retval;
}

/*
 * Discoveres usable IP addresses on the host machine.  This
 * uses malloc, so call free_interface_list sometime.
 * 
 * Return value:
 *    Pointer to address list created
 * 
 */
struct path_ll* discover_interfaces()
{
  struct path_ll addr_list; // List uses sentinel node.
  
  struct ifaddrs *ifaddr, *p;
  int status;
  char ipstr[INET6_ADDRSTRLEN];
  struct path_ll *curr = NULL;
  struct path_ll *next = NULL;

  addr_list.active = IFUP;
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
              next = (struct path_ll *)malloc(sizeof(struct path_ll));
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
void free_interface_list( struct path_ll *addr_list )
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
