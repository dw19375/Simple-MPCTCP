//
//  multi_if.h
//  MultiIFTest
//
//  Created by Jason Cloud on 9/24/13.
//  Copyright (c) 2013 Jason Cloud. All rights reserved.
//

#ifndef MultiIFTest_main_h
#define MultiIFTest_main_h

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>

#define IFDOWN 0
#define IFUP 1

struct addr_ll {
    int active;
    char if_name[10];
    struct sockaddr_in addr;
    struct addr_ll * next;
    int sk;
};



#endif
