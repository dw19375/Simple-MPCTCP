/*
 * pslist.h
 * 
 * Implementation of a sorted list for Data_Pckt's.  Packets are
 * kept in a circularly linked list and sorted according to their 
 * sequence numbers (seqno).  The head of the list has the highest
 * seqno, head->next contains a lower seqno, etc.
 * 
 * External code only needs to keep a pointer to the head of the 
 * list, since the tail is head->prev.
 * 
 * At the moment, none of this is thread safe.
 */

#ifndef P_QUEUE_H
#define P_QUEUE_H

#include "net_util.h"

typedef struct pslist_elem
{
  struct pslist_elem* next;
  struct pslist_elem* prev;
  Data_Pckt pkt;
} pslist_elem;


/*
 * Function Declarations
 */
pslist_elem* create_pslist_elem( uint32_t buflen );
void delete_list_elem( pslist_elem* elem );
pslist_elem* ins_pslist_elem( pslist_elem** head, pslist_elem* elem );
Data_Pckt* peek_pslist( pslist_elem* head );
int rm_pslist_elem( pslist_elem** head, pslist_elem* elem );
int chop_pslist( pslist_elem** head, uint32_t seqno );

#endif /* P_QUEUE_H */
