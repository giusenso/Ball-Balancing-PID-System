#pragma once

typedef struct ListItem {
  short value;
  struct ListItem* next;
  struct ListItem* prev;
} ListItem;

typedef struct ListHead {
  ListItem* first;
  ListItem* last;
  short size;
  short sum;
} ListHead;

void List_print(ListHead* head);
void List_init(ListHead* head, short n, short val);
void List_insert_new(ListHead* head, short new);
void List_detach_last(ListHead* head);
short List_update(ListHead* head, short new);
