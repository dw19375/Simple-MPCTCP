//
//  default_config.h
//  Coded-TCP-Multipath-Xcode
//
//  Created by Jason Cloud on 1/20/15.
//  Copyright (c) 2015 MIT. All rights reserved.
//

#ifndef DEFAULT_CONFIG_H
#define DEFAULT_CONFIG_H


//#include <stdio.h>
//#include <string.h>
//#include <unistd.h>

#define MAX_CFG_VARS 30

typedef struct proxy_config {
    char program_name[128];
    char version[128];
    char date[128];
    char config_file[256];
    char logdir[265];
    char logfile[256];
    char pidfile[256];
    char local_ip[256];
    int socks_port;
    int port_start;
    int port_end;
    int max_connections;
    int socks_neg_rto;
    int max_backlog;
    int max_pktbuf_size;
    char remote_addr[256];
    int remote_port;
    double slr_scaling;
    char cong_control[16];
    int debug;
    int mpctcp_probe;
    int fork;
    int num_vars;
    void* var_name[MAX_CFG_VARS];
    void* var_ptr[MAX_CFG_VARS];
    char var_type[MAX_CFG_VARS];
} proxy_config;

extern proxy_config config;

#endif /* DEFAULT_CONFIG_H */
