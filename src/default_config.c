//
//  default_config.c
//  Coded-TCP-Multipath-Xcode
//
//  Created by Jason Cloud on 1/20/15.
//  Copyright (c) 2015 MIT. All rights reserved.
//

#include "default_config.h"

proxy_config config = {
    // Program Information
    .program_name="mpctcp",
    .version="0.0",
    .date=__DATE__,
    
    // File Names/Locations
#ifdef __MACH__
    .config_file="~./mpctcp/mpctcp.config",
    .logdir="~/.mpctcp",
    .logfile="~/.mpctcp/mpctcp.log",
    .pidfile="~/.mpctcp/mpctcp.pid",
#else
    .config_file="/etc/mpctcp/mpctcp.conf",
    .logdir="/var/log",
    .logfile="/var/log/mpctcp",
    .pidfile="/var/run/mpctcp.pid",
#endif
    // SOCKS Information
    .local_ip="0.0.0.0",
    .socks_port=1080,
    .port_start=8001,
    .port_end=9999,
    .max_connections=25,
    .socks_neg_rto=5,
    .max_backlog = 10,
    .max_pktbuf_size = 2000,
    
    // Proxy Information
    .remote_addr="",
    .remote_port=8888,
    
    // MPCTCP Settings
    .slr_scaling=1.0,
    .cong_control="aimd",
    
    // Debug Settings
    .debug=1,
    .mpctcp_probe=0,
    .fork=1,
    
    // Configuration File Variables
    .num_vars = 16,
    .var_name = {
        "LOG_DIR", "LOG_FILE", "PID_FILE",
        "BIND_ADDR", "SOCKS_PORT", "START_PORT",
        "END_PORT", "MAX_CONNECTIONS", "SOCKS_NEGOTIATION_RTO",
        "REMOTE_ADDR", "REMOTE_PORT", "SLR_SCALING",
        "CONGESTION_CONTROL","DEBUG","MPCTCP_PROBE",
        "FORK"
    },
    
    .var_ptr = {
        &(config.logdir), &(config.logfile), &(config.pidfile),
        &(config.local_ip), &(config.socks_port), &(config.port_start),
        &(config.port_end), &(config.max_connections), &(config.socks_neg_rto),
        &(config.remote_addr), &(config.remote_port), &(config.slr_scaling),
        &(config.cong_control), &(config.debug), &(config.mpctcp_probe),
        &(config.fork)
    },
    
    .var_type = {
        's', 's', 's',
        's', 'n', 'n',
        'n', 'n', 'n',
        's', 'n', 'f',
        's', 'n', 'n',
        'n'
    }
 
};
