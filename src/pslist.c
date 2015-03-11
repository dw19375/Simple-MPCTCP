/*
 * pslist.c
 * 
 * Implementation of a sorted list for Data_Pckt's.  Packets are
 * kept in a linked list and sorted according to their sequence
 * numbers (seqno).
 * 
 * External code only needs to keep a pointer to the head of the 
 * list, since the tail is head->prev.
 * 
 * At the moment, none of this is thread safe.
 */
#include <stdlib.h>
#include <stdio.h>
#include "pslist.h"

/*
 * Allocates memory for a pslist_elem, which contains a Data_Pckt
 * plus its payload buffer (pkt.buf).  Everything is malloc'ed 
 * together with pkt.buf pointing to the memory address right 
 * after the pslist_elem struct.  pkt.buf is initialized here.
 * 
 * NOTE: This uses malloc, so please call delete_pslist_elem
 * somewhere.  Currently, it is called by rm_pslist_elem.
 * 
 * Inputs:
 *  buflen - Length of payload buffer to allocate.  If buflen == 0,
 *           then pkt.buf is NULL.
 * 
 * Returns a pointer to the pslist_elem created.
 */
pslist_elem* create_pslist_elem( uint32_t buflen )
{
  pslist_elem* ret = NULL;
  
  ret = (pslist_elem*)malloc(sizeof(pslist_elem) + buflen);
  
  if( NULL != ret )
  {
    // Add payload length to Data_Pckt struct.
    ret->pkt.payload_len = buflen;
    
    if( buflen > 0 )
    {
      ret->pkt.buf = ((char*)ret) + sizeof(pslist_elem);
    }
    else
    {
      ret->pkt.buf = NULL;
    }
  }
  
  return ret;
}

/*
 * Frees memory associated with the pslist_elem elem.
 * 
 * NOTE: Do not call this unless elem is not in a list anywhere.
 * 
 * Input:
 *  elem - Pointer to pslist_elem to free.
 */
void delete_list_elem( pslist_elem* elem )
{
  if( NULL != elem )
  {
    free( elem );
  }
}

/*
 * Inserts a new Data_Pckt into the list pointed to by head.  The
 * packet is inserted just after (i.e., closer to the head) a packet
 * with a smaller seqno.  If head is NULL, a new list is created.
 * 
 * Inputs:
 *    head - pointer to pointer to the head of the list.  If head 
 *           points to a NULL pointer, a new list is created.
 *    elem - Pointer to the pslist_elem to insert.
 * 
 * Returns pointer to pslist_elem that was inserted/created. 
 */
pslist_elem* ins_pslist_elem( pslist_elem** head, pslist_elem* elem )
{
  pslist_elem* cur;
  pslist_elem* ret = NULL;
  
  if( NULL != elem && NULL != head )
  {
    ret = elem;
    
    if( NULL == *head )
    {
      // No list exists yet, so we'll make a new one.
      elem->next = elem;
      elem->prev = elem;
    }
    else
    {
      // Add the element to the appropriate location
      cur = (*head);
      do
      {
        if( elem->pkt.seqno > cur->pkt.seqno )
        {
          // Insert new element right before cur
          elem->prev = cur->prev;
          elem->next = cur;
          (cur->prev)->next = elem;
          cur->prev = elem;
          
          if( cur == (*head) )
          {
            // We just inserted at the beginning of the list, update
            // head pointer.
            (*head) = elem;
          }
          break;
        }
        else if( cur->next == (*head) )
        {
          // We're at the end of the list, insert after cur
          elem->next = cur->next;
          elem->prev = cur;
          (cur->next)->prev = elem;
          cur->next = elem;
          break;
        }
          
        // Traverse list
        cur = cur->next;
      }
      while( (NULL != cur) && (cur->next != (*head)) );
    }
  }
  
  return ret;
}

/*
 * Returns a pointer to the oldest Data_Pckt in the list.
 * 
 * Input:
 *  head - pointer to the head of the list
 * 
 * Returns pointer to the oldest element in the list.
 */
Data_Pckt* peek_pslist( pslist_elem* head )
{
  Data_Pckt* ret = NULL;
  
  if( NULL != head && NULL != head->prev )
  {
    ret = &(head->prev->pkt);
  }
  
  return ret;
}

/*
 * Removes element pointed to by elem.  All memory associated
 * with the packet is free'd.
 * 
 * Input:
 *  elem - pointer to pslist element to remove.
 * 
 * Return Value:
 *  0   - Last element of list removed.
 *  1   - There exist more elements in the list
 *  -1  - An error occured.
 */
int rm_pslist_elem( pslist_elem** head, pslist_elem* elem )
{
  int retval = -1;
  
  if( NULL != elem )
  {
    if( (elem == elem->prev) && 
        (elem == elem->next) )
    {
      // This is the last element, set *head to NULL
      *head = NULL;
      retval = 0;
    }
    else
    {
      // There are more elements
      retval = 1;
      
      if( NULL != elem->prev )
      {
        (elem->prev)->next = elem->next;
      }
      else
      {
        retval = -1;
        fprintf( stderr, "NULL pointer in non-empty list!\n" );
      }
      
      if( NULL != elem->next )
      {
        (elem->next)->prev = elem->prev;
      }
      else
      {
        retval = -1;
        fprintf( stderr, "NULL pointer in non-empty list!\n" );
      }
    }
    
    // Now that the pointers are taken care of, free the memory.
    delete_list_elem( elem );
  }
  
  return retval;
}

/*
 * Removes all elements from the pslist that have a seqno less than
 * the seqno passed in.  If an element with a NULL pkt is encountered,
 * the function returns immediately, and does not attempt to remove
 * more packets.  This should not happen, as ins_pslist_elem should
 * ensure that elements don't have NULL pkts.
 * 
 * Inputs:
 *   head -  pointer to pointer to head of list.
 *   seqno - Elements with packets that have a seqno less than or 
 *           equal to this will be deleted
 * 
 * Returns number of packets removed.
 */
int chop_pslist( pslist_elem** head, uint32_t seqno )
{
  int retval = 0;
  pslist_elem* cur;
  
  if( NULL != head && NULL != (*head) )
  {
    cur = (*head)->prev;
    
    for( cur = (*head)->prev; 
         (NULL != cur) && (cur->pkt.seqno <= seqno);
         cur = cur->prev, retval++ )
    {
      int ret;
      
      ret = rm_pslist_elem( head, cur );
      
      if( ret <= 0 )
      {
        // Either no more elements or an error occured.
        break;
      }
    }
  }
  
  return retval;
}


