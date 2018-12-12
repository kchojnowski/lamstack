/**********************************************************\
|                                                          |
| xxtea.h                                                  |
|                                                          |
| XXTEA encryption algorithm library for C.                |
|                                                          |
| Encryption Algorithm Authors:                            |
|      David J. Wheeler                                    |
|      Roger M. Needham                                    |
|                                                          |
| Code Authors: Chen fei <cf850118@163.com>                |
|               Ma Bingyao <mabingyao@gmail.com>           |
| LastModified: Mar 3, 2015                                |
|                                                          |
\**********************************************************/

#ifndef XXTEA_INCLUDED
#define XXTEA_INCLUDED

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool xxtea_encrypt(const void * data, size_t len, const void * key, void* out, size_t * out_len);

bool xxtea_decrypt(const void * data, size_t len, const void * key, void* out, size_t * out_len);

void xxtea_init(uint32_t* dataBuffer, size_t dataBufferLen, uint32_t* keyBuffer, size_t keyBufferLen);

#ifdef __cplusplus
}
#endif

#endif

