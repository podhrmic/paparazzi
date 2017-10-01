/* This file was auto-generated by KreMLin! */
#include "kremlib.h"
#ifndef __Chacha20_H
#define __Chacha20_H

typedef uint32_t Hacl_Impl_Xor_Lemmas_u32;

typedef uint8_t Hacl_Impl_Xor_Lemmas_u8;

typedef uint8_t *Hacl_Lib_LoadStore32_uint8_p;

typedef uint32_t Hacl_Impl_Chacha20_u32;

typedef uint32_t Hacl_Impl_Chacha20_h32;

typedef uint8_t *Hacl_Impl_Chacha20_uint8_p;

typedef uint32_t *Hacl_Impl_Chacha20_state;

typedef uint32_t Hacl_Impl_Chacha20_idx;

typedef struct 
{
  void *k;
  void *n;
}
Hacl_Impl_Chacha20_log_t_;

typedef void *Hacl_Impl_Chacha20_log_t;

typedef uint32_t Hacl_Lib_Create_h32;

typedef uint8_t *Chacha20_uint8_p;

typedef uint32_t Chacha20_uint32_t;

void *Chacha20_op_String_Access(FStar_Monotonic_HyperStack_mem h, uint8_t *m);

void Chacha20_chacha20_key_block(uint8_t *block, uint8_t *k, uint8_t *n1, uint32_t ctr);

void
Chacha20_chacha20(
  uint8_t *output,
  uint8_t *plain,
  uint32_t len,
  uint8_t *k,
  uint8_t *n1,
  uint32_t ctr
);
#endif
