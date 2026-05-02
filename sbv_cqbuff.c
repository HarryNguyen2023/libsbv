#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "sbv_rtos.h"
#include "sbv_cqbuff.h"

sbv_cqbuff *sbv_cqbuff_create (int capacity, int element_size)
{
  sbv_cqbuff *new_buf;

  if (!capacity || !element_size)
    return NULL;

  new_buf = (sbv_cqbuff *)sbv_rtos_malloc(sizeof (sbv_cqbuff) + capacity);
  if (! new_buf)
  {
    /* LOG */
    return NULL;
  }

  memset (new_buf->buff, 0, capacity);
  new_buf->capacity     = capacity;
  new_buf->element_size = element_size;
  new_buf->head         = -1;
  new_buf->rear         = -1;
  return new_buf;
}

void sbv_cqbuff_delete (sbv_cqbuff* buff)
{
  if (! buff)
    return;
  
  sbv_rtos_free (buff);
  return;
}

int sbv_cqbuff_is_empty (sbv_cqbuff* buff)
{
  if (! buff)
    return 0;

  return (buff->head == -1);
}

int sbv_cqbuff_is_full (sbv_cqbuff* buff)
{
  if (! buff)
    return 0;

  return (buff->head == buff->rear
          && buff->head != -1);
}

int sbv_cqbuff_avail_size (sbv_cqbuff* buff)
{
  if (!buff)
    return 0;

  if (buff->head > buff->rear)
    return (buff->capacity - (buff->head - buff->rear));
  else if (buff->head < buff->rear)
    return (buff->rear - buff->head);
  else if (sbv_cqbuff_is_empty (buff))
    return buff->capacity;

  return 0;
}

int sbv_cqbuff_get_size (sbv_cqbuff* buff)
{
  if (! buff)
    return -1;

  return buff->capacity - sbv_cqbuff_avail_size(buff);
}

int sbv_cqbuff_write (sbv_cqbuff* buff, unsigned char* data, int element_num)
{
  int avail_size = 0, write_size = 0, tail_size = 0;

  if (!buff || !data || !element_num)
    return 0;

  if (sbv_cqbuff_is_full (buff))
    return 0;

  avail_size = sbv_cqbuff_avail_size (buff);
  if (avail_size < buff->element_size)
    return 0;

  if (sbv_cqbuff_is_empty (buff))
  {
    buff->head = 0;
    buff->rear = 0;
  }
  write_size = element_num * buff->element_size;
  write_size = (write_size > avail_size) ? \
                avail_size - (avail_size % buff->element_size) : write_size;
  tail_size  = buff->capacity - buff->head;
  if (tail_size > write_size)
    memcpy ((void *)(buff->buff + buff->head), (void *)data, write_size);
  else
  {
    memcpy ((void *)(buff->buff + buff->head), (void *)data, tail_size);
    memcpy ((void *)(buff->buff), (void *)(data + tail_size), write_size - tail_size);
  }

  buff->head = (buff->head + write_size) % buff->capacity;

  return (write_size / buff->element_size);
}

int sbv_cqbuff_read (sbv_cqbuff* buff, unsigned char* data, int element_num)
{
  int avail_size = 0, read_size = 0, tail_size = 0;

  if (!buff || !data || !element_num)
    return 0;

  if (sbv_cqbuff_is_empty (buff))
    return 0;

  avail_size = buff->capacity - sbv_cqbuff_avail_size (buff);
  if (avail_size < buff->element_size)
    return 0;

  read_size = element_num * buff->element_size;
  read_size = (read_size > avail_size) ? \
                (avail_size - (avail_size % buff->element_size)) : read_size;
  tail_size  = buff->capacity - buff->rear;
  if (tail_size > read_size)
  {
    memcpy ((void *)data, (void *)(buff->buff + buff->rear), read_size);
  }
  else
  {
    memcpy ((void *)data, (void *)(buff->buff + buff->rear), tail_size);
    memcpy ((void *)(data + tail_size), (void *)(buff->buff), read_size - tail_size);
  }

  buff->rear = (buff->rear + read_size) % buff->capacity;

  return (read_size / buff->element_size);
}

void sbv_cqbuff_dump (sbv_cqbuff *buff, void (*sbv_cqbuff_element_print)(void *))
{
  int i, count = 0;

  if (!buff)
    return;

  if (sbv_cqbuff_is_empty (buff))
  {
    return;
  }
  printf ("Circular buffer: ");

  i = buff->rear;
  do {
    if ((sbv_cqbuff_element_print))
      (*sbv_cqbuff_element_print)(buff->buff + i);
    i = (i + buff->element_size) % buff->capacity;
    count += buff->element_size;
  } while (i != buff->head && count < buff->capacity);

  printf ("\n");
}