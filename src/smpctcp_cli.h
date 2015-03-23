#ifndef MPCTCP_CLI_H
#define MPCTCP_CLI_H 

#include "net_util.h"

/*
 * Function Declarations
 */
void handle_packet( struct iovec *iov );
void handle_data_pkt( Data_Pckt* pkt );
void handle_syn( Data_Pckt* pkt );
void handle_syn_ack( Data_Pckt* pkt );
void handle_fin( Data_Pckt* pkt );
void handle_fin_ack( Data_Pckt* pkt );
void handle_coded_pkt( Data_Pckt* pkt );
void handle_ack( Data_Pckt* pkt );

#endif
