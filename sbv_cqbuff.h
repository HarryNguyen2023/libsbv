#ifndef __SBV_CIRCULAR_BUFFER_H__
#define __SBV_CIRCULAR_BUFFER_H__

typedef struct sbv_cqbuff
{
  int       capacity;
  int       head;
  int       rear;
  int       element_size;
  uint8_t   buff[];
} sbv_cqbuff;

sbv_cqbuff *sbv_cqbuff_create (int capacity, int element_size);
void sbv_cqbuff_delete (sbv_cqbuff* buff);
int sbv_cqbuff_is_empty (sbv_cqbuff* buff);
int sbv_cqbuff_is_full (sbv_cqbuff* buff);
int sbv_cqbuff_write (sbv_cqbuff* buff, unsigned char* data, int element_num);
int sbv_cqbuff_read (sbv_cqbuff* buff, unsigned char* data, int element_num);
void sbv_cqbuff_dump (sbv_cqbuff *buff, void (*sbv_cqbuff_element_print)(void *));
int sbv_cqbuff_avail_size (sbv_cqbuff* buff);
int sbv_cqbuff_get_size (sbv_cqbuff* buff);

#endif /* __SBV_CIRCULAR_BUFFER_H__ */