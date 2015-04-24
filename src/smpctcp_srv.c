/*
 * mpctcp_srv.c
 * 
 * Functions to listen for incoming connections, send packets, and
 * handle ACKs.
 */

#include "default_config.h"
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
 * Input is a pointer to a path_ll struct.  When the thread 
 * is started, all the variables in arg should be initialized.
 * 
 * 
 * Does not return anything.
 */
void *sender_thread( void* arg )
{
  path_ll* data = (path_ll*)arg;
  int bytes;
  
  if( NULL != data )
  {
    while( 1 )
    {
      /*
      * Lock queue mutex and wait for signal.  Note that the
      * pthread_cond_wait routine will automatically and atomically 
      * unlock mutex while it waits.
      * This is in a while loop in case data->pkt becomes NULL after
      * we have been signaled, which should theoretically never happen.
      * We are, however, dealing with the black magic of multithreaded 
      * programming, so it doesn't hurt to be cautious.
      */
      pthread_mutex_lock( &(data->qlock) );
      while( data->pkt == NULL )
      {
        // While we don't have a packet to send, just wait here
        pthread_cond_wait( &(data->qcond), &(data->qlock) );
      }
      
      // Set the busy flag and increment the in_flight and sent counters
      pthread_mutex_lock( &(data->data_lock) );
      data->busy = 1;
      (data->sent)++;
      (data->in_flight)++;
      pthread_mutex_unlock( &(data->data_lock) );
      
      
      // Send the packet
      bytes = sendpkt( data->pkt, data->sk, data->remote, data->addr );
      
      // Set data->pkt to NULL to indicate we can receive another packet
      data->pkt = NULL;
      
      // We're not sending a packet anymore
      pthread_mutex_unlock( &(data->qlock) );
      
      // Clear busy flag
      pthread_mutex_lock( &(data->data_lock) );
      data->busy = 0;
      pthread_mutex_unlock( &(data->data_lock) );
    }
  }
  
  pthread_exit( NULL );
}

/*
 * Initialize the packet and thread management data of a 
 * path_ll element.  The Interface data should already be set.
 * 
 * Initializes mutexes and condition variables, creates the 
 * sockets, and initializes all the path statistics.
 * 
 * Binds listen_sk to next available port.
 * 
 * NOTE: Call destroy_path to destroy mutexes and condition 
 * variables.
 * 
 * Inputs:
 *    p - Pointer to path_ll struct.
 *    remote_ip - remote IP address string
 * 
 * Return value:
 *  Pointer to initialized path_ll struct.
 */
path_ll* init_path( path_ll* p, char* remote_ip, int remote_port )
{
  static int listen_port = 0;
  
  if( listen_port == 0 )
  {
    listen_port = config.port_start;
  }
  
  if( NULL != p )
  {
    // Initialize mutexes and condition variables
    pthread_mutex_init( &(p->qlock), NULL );
    pthread_mutex_init( &(p->data_lock), NULL );
    pthread_cond_init( &(p->qcond), NULL );
    
    // Initialize data
    p->pkt = NULL;
    p->busy = 0;
    p->in_flight = 0;
    p->sent = 0;
    p->lost = 0;
    p->rtt = 0.0;
    p->loss_prob = 0.0;
    
    // Initialize sockets, bind listener
    create_udp_socket( &(p->sk), remote_ip, remote_port, 0 );
    create_udp_socket( &(p->listen_sk), remote_ip, remote_port, 1 );
  }
  
  return p;
}

/*
 * Destroys mutexes and closes sockets associated with a path.
 * 
 * Input:
 *    p - path_ll to destroy
 */
void destroy_path( path_ll* p )
{
  if( NULL != p )
  {
    pthread_mutex_destroy( &(p->qlock) );
    pthread_mutex_destroy( &(p->data_lock) );
    pthread_cond_destroy( &(p->qcond) );
    
    close( p->sk );
    close( p->listen_sk );
  }
}

/*
 * Read from a file and place packet in the given queue.  This is not 
 * thread safe, but is intended to be called from the main thread
 * only.
 * 
 * NOTE that fd must refer to an open file.
 * NOTE Please call rm_pslist_elem to free the malloc'ed memory.
 * 
 * Inputs:
 *    fd - File to read from
 *    q - Pointer to pointer to the beginning of the queue
 *    n - Maximum number of bytes to read from file
 *    
 * Return Value:
 *    Pointer to newly created element.  NULL if error.
 */
pslist_elem* queue_next_packet( int fd, pslist_elem** q, int n )
{
  pslist_elem* elem = NULL;
  
  if( NULL != q )
  {
    // Create a new packet - we'll allocate n bytes in the buffer,
    // even if we don't need them.  We'll change it later.
    elem = create_pslist_elem( n );
    
    if( NULL != elem )
    {
      init_normal_data_pckt( &(elem->pkt), fd, n );
      
      if( 0 != (elem->pkt).payload_len )
      {
        // Add the element to the queue.
        ins_pslist_elem( q, elem );
      }
    }
  }
  
  return elem;
}

/*
 * Allocates memory for and initializes a new normal Data_Pckt.  Reads from
 * the file referred to by fd and copies at most n bytes into the packet's
 * data buffer
 * 
 * Inputs:
 *    fd - File to read from
 *    n  - Maximum number of bytes to read.
 * 
 * Returns pointer to packet that was created.
 */
Data_Pckt* make_next_pkt( int fd, int n )
{
  Data_Pckt* pkt;
  
  // Allocate memory for the packet
  pkt = create_pkt( n );
  
  // Initialize the packet and return it.
  return init_normal_data_pckt( pkt, fd, n );
}

/*
 * Initializes a NORMAL Data_Pckt.  Reads at most n bytes from 
 * file given by fd.
 * 
 * NOTE: This function keeps track of sequence numbers, so only
 * use this to set the seqno of a packet.
 * 
 * This does not allocate a buffer for the data, THAT MUST HAVE BEEN
 * DONE PREVIOUSLY!
 * 
 * Inputs:
 *    pkt - Pointer to Data_Pckt
 *    fd  - File to read from
 *    n   - Maximum number of bytes to read from file
 * 
 * Returns Data_Pckt passed in.
 * 
 */
Data_Pckt* init_normal_data_pckt( Data_Pckt *pkt, int fd, int n )
{
  static int seqno = 0;
  int retval;
  
  if( NULL != pkt )
  {
    // Read up to n bytes from fd, put in packet buffer
    retval = read( fd, pkt->buf, n );
    
    if( retval > 0 )
    {
      // We have read some bytes, go ahead and make the packet
      pkt->tstamp = getTime();
      pkt->flag = NORMAL;
      pkt->seqno = seqno++;    // Increments after
      pkt->payload_len = retval;
      pkt->num_packets = 0;
      pkt->coeff_seed = 0;
    }
    
  }
  
  return pkt;
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
    if( NULL != addr_list->next )
    {
      free_interface_list( addr_list->next );
    }
    
    // Destroy mutexes and then free memory associated with addr_list
    destroy_path( addr_list );
    free( addr_list );
  }
}
