//
//  multi_if.c
//  MultiIFTest
//
//  This code serves as a test to determine if packets can be
//  transmitted out of a specific interface on the local 
//  computer.  There are two options that seem to work:
//
//    1. Option 1: Setup a UDP socket and bind to a specific
//                 IP address. Messages are then sent using
//                 'sendto'.

//                 When executing the program,
//                 you need to make sure that the routing 
//                 tables are setup appropriately.  This 
//                 should be done similarly to what was 
//                 done at: 
//         
//	multipath-tcp.org/pmwiki.php/Users/ConfigureRouting
//
//    2. Option 2: Setup a UDP socket and bind to a specific
//                 IP address. To send a message, you will
//                 to use 'sendmsg' with the appropriate 
//                 command headers.  The code below provides
//                 an example.  It appears that there is no 
//                 need to modify routing tables or anything
//                 else on the system to run this program.
//  
//  Created by Jason Cloud on 9/24/13.
//  Copyright (c) 2013 Jason Cloud. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <string.h>
#include "multi_if.h"

#define REMOTE_ADDR "18.62.16.244"
#define REMOTE_PORT 8888

int main(int argc, const char * argv[])
{
  
  struct ifaddrs *ifaddr, *p;
  struct addr_ll *addr_list = NULL;
  int status;
  char ipstr[INET6_ADDRSTRLEN];
  struct addr_ll *curr = NULL;
  struct addr_ll *next = NULL;
  ssize_t bytes;
  int i;
  
  
  addr_list = (struct addr_ll *)malloc(sizeof(struct addr_ll));
  addr_list->active = IFDOWN;
  addr_list->next = NULL;
  
  if ((status = getifaddrs(&ifaddr)) != 0) {
    fprintf(stderr, "getifaddrs: %s\n",gai_strerror(status));
    return 2;
  }
  
  // Go through each of the available IP addresses and store them.
  for (p = ifaddr; p!=NULL; p=p->ifa_next) {
    
    if (p->ifa_addr != NULL) {
      
      if (p->ifa_addr->sa_family == AF_INET) {
        
        
        //Determine if address is the loopback or is already registered as active
        int ignore = 0;
        
        struct sockaddr_in *ipv4 = (struct sockaddr_in *)p->ifa_addr;
        inet_ntop(AF_INET, &ipv4->sin_addr, ipstr, sizeof(ipstr));
        
        if (strcmp(ipstr, "127.0.0.1")) {
          for (curr = addr_list; curr->next != NULL; curr = curr->next) {
            if (curr->active == IFUP && &(curr->addr.sin_addr) == &(ipv4->sin_addr)) {
              ignore = 1;
              break;
            }
          }
        } else {
          ignore = 1;
        }
        
        //If it is not already registered
        if (ignore == 0) {
          
          if (curr->active == IFUP) {
            next = (struct addr_ll *)malloc(sizeof(struct addr_ll));
            curr->next = next;
          } else {
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
  
  
  // Print out each available interface and its IPv4 address
  printf("My IP addresses are:\n\n");
  
  curr = addr_list;
  
  while (1) {
    if (curr->active == IFUP) {
      inet_ntop(curr->addr.sin_family,&(curr->addr.sin_addr),ipstr,sizeof(ipstr));
      printf("%s: %s\n",curr->if_name,ipstr);
    }
    
    curr = curr->next;
    if (curr == NULL)
      break;
  }
  
  
  freeifaddrs(ifaddr);
  
  // Now setup a socket on each interface and transmit a message
  char buf[13] = "Hello World!";
  struct sockaddr_in remote;
  
  inet_pton(AF_INET, REMOTE_ADDR, &(remote.sin_addr));
  remote.sin_port = htons(REMOTE_PORT);
  remote.sin_family = AF_INET;
  
  // Code for generating messages using sendmsg: //
  struct iovec iov;
  struct msghdr msg = {0};
  struct cmsghdr *cmsg;
  struct in_pktinfo *pktinfo;
  char control[sizeof(struct cmsghdr)+12];
  const int on=1, off=0;
  // End sendmsg code
  
  curr = addr_list;   
  
  while (1) { 
    if (curr->active == IFUP) {
      
      if ((curr->sk = socket(curr->addr.sin_family, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        continue;
      }
      
      
      // Code used when sendmsg is used 
      setsockopt(curr->sk, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
      setsockopt(curr->sk, IPPROTO_IP, IP_PKTINFO, &on, sizeof(on));
      // End code for sendmsg
      
      
      curr->addr.sin_port = htons(7777);
      
      if (bind(curr->sk, (struct sockaddr *)&curr->addr, sizeof(curr->addr)) == -1) {
        close(curr->sk);
        perror("bind");
        continue;
      }
      
      for (i = 0; i < 10; i++) {
        
        
        // Code for generating message for sendmsg:
        iov.iov_base = buf;
        iov.iov_len = sizeof(buf);
        
        msg.msg_name = &remote;
        msg.msg_namelen = sizeof(remote);
        
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        
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
        pktinfo->ipi_spec_dst = curr->addr.sin_addr;
        pktinfo->ipi_addr = remote.sin_addr;
        
        if ((bytes = sendmsg(curr->sk, &msg, 0)) == -1) {
          perror("send");
          exit(1);
        }
        // End sendmsg code
        
        /*
         *			// Code used for sendto
         *			if ((bytes = sendto(curr->sk,buf,sizeof(buf),0,(struct sockaddr *)&remote,sizeof(remote))) == -1) {
         *				perror("send");
         *				exit(1);
      }
      // End sendto code
      */
      }
    }
    
    close(curr->sk);
    
    curr = curr->next;
    if (curr == NULL)
      break;
    
  }
  
  return 0;
}

