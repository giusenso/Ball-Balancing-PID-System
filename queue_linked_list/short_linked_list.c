#include "short_linked_list.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

void List_print(ListHead* head){
  ListItem* aux = head->first;
  printf("[");
  while(aux){
    ListItem* element = (ListItem*)aux;
    printf("%d ", element->value);
    aux = aux->next;
  }
  printf("]   size = %d   sum = %d\n", head->size, head->sum);
}

void List_init(ListHead* head, short n, short val){
    ListItem* adamo = (ListItem*)malloc(sizeof(ListItem));
    head->size = 0;
    head->first = adamo;
    head->last = adamo;
    adamo->value = val;
    adamo->next = 0;
    adamo->prev = 0;
    head->size++;
    head->sum = adamo->value;

    while(head->size < n){
        List_insert_new(head, val);
    }
}

void List_insert_new(ListHead* head, short new){
    ListItem* new_elem = (ListItem*)malloc(sizeof(ListItem));
    new_elem->next = head->first;
    new_elem->prev = 0;
    head->first->prev = new_elem;
    head->first = new_elem;
    head->size++;
    head->sum += new;
    new_elem->value = new;
}

void List_detach_last(ListHead* head){
    head->sum -= head->last->value;
    head->last = head->last->prev;
    free(head->last->next);
    head->last->next = 0;
    head->size--;
}

//update and return average value
short List_update(ListHead* head, short new){
    List_insert_new(head, new);
    List_detach_last(head);
    return (head->sum / head->size);
}




























//////////////////////////////
