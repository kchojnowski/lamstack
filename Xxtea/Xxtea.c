/**********************************************************\
|                                                          |
| xxtea.c                                                  |
|                                                          |
| XXTEA encryption algorithm library for C.                |
|                                                          |
| Encryption Algorithm Authors:                            |
|      David J. Wheeler                                    |
|      Roger M. Needham                                    |
|                                                          |
| Code Authors: Chen fei <cf850118@163.com>                |
|               Ma Bingyao <mabingyao@gmail.com>           |
| LastModified: Feb 7, 2016                                |
|                                                          |
\**********************************************************/

#include "Xxtea.h"
#include <string.h>

#define MX (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z))
#define DELTA 0x9e3779b9

#define FIXED_KEY \
    size_t i;\
    uint8_t fixed_key[16];\
    memcpy(fixed_key, key, 16);\
    for (i = 0; (i < 16) && (fixed_key[i] != 0); ++i);\
    for (++i; i < 16; ++i) fixed_key[i] = 0;\

static uint8_t* internalOutputBuffer = NULL;
static uint32_t* internalDataBuffer = NULL;
static uint32_t* internalKeyBuffer = NULL;

static size_t internalDataBufferLen = 0;
static size_t internalKeyBufferLen = 0;

static bool xxtea_to_uint_array(const uint8_t * data, size_t len, int inc_len, size_t * out_len, uint32_t* out, size_t out_max_len) {
    size_t n;
    n = (((len & 3) == 0) ? (len >> 2) : ((len >> 2) + 1));

    if (!out)
        return false;

    if (inc_len) {
        *out_len = n + 1;
        if (*out_len > out_max_len)
            return false;
        out[n] = (uint32_t)len;
    }
    else {
        *out_len = n;
        if (*out_len > out_max_len)
            return false;
    }
    memcpy(out, data, len);
    return true;
}

static bool xxtea_to_ubyte_array(const uint32_t * data, size_t len, int inc_len, size_t * out_len) {
    size_t m, n;

    n = len << 2;

    if (inc_len) {
        m = data[len - 1];
        n -= 4;
        if ((m < n - 3) || (m > n)) 
            return false;
        n = m;
    }

    memcpy(internalOutputBuffer, data, n);
    *out_len = n;

    return true;
}

static uint32_t * xxtea_uint_encrypt(uint32_t * data, size_t len, uint32_t * key) {
    uint32_t n = (uint32_t)len - 1;
    uint32_t z = data[n], y, p, q = 6 + 52 / (n + 1), sum = 0, e;

    if (n < 1) return data;

    while (0 < q--) {
        sum += DELTA;
        e = sum >> 2 & 3;

        for (p = 0; p < n; p++) {
            y = data[p + 1];
            z = data[p] += MX;
        }

        y = data[0];
        z = data[n] += MX;
    }

    return data;
}

static uint32_t * xxtea_uint_decrypt(uint32_t * data, size_t len, uint32_t * key) {
    uint32_t n = (uint32_t)len - 1;
    uint32_t z, y = data[0], p, q = 6 + 52 / (n + 1), sum = q * DELTA, e;

    if (n < 1) return data;

    while (sum != 0) {
        e = sum >> 2 & 3;

        for (p = n; p > 0; p--) {
            z = data[p - 1];
            y = data[p] -= MX;
        }

        z = data[n];
        y = data[0] -= MX;
        sum -= DELTA;
    }

    return data;
}

static bool xxtea_ubyte_encrypt(const uint8_t * data, size_t len, const uint8_t * key, size_t * out_len) {
    size_t data_len, key_len;

    if (!len)
        return NULL;

    if (!xxtea_to_uint_array(data, len, 1, &data_len, internalDataBuffer, internalDataBufferLen))
        return NULL;

    if (!xxtea_to_uint_array(key, 16, 0, &key_len, internalKeyBuffer, internalKeyBufferLen))
        return NULL;

    xxtea_uint_encrypt(internalDataBuffer, data_len, internalKeyBuffer);

    return xxtea_to_ubyte_array(internalDataBuffer, data_len, 0, out_len);
}

static bool xxtea_ubyte_decrypt(const uint8_t * data, size_t len, const uint8_t * key, size_t * out_len) {
    size_t data_len, key_len;

    if (!len) 
      return NULL;

    if (!xxtea_to_uint_array(data, len, 0, &data_len, internalDataBuffer, internalDataBufferLen))
        return NULL;

    if (!xxtea_to_uint_array(key, 16, 0, &key_len, internalKeyBuffer, internalKeyBufferLen))
        return NULL;

    xxtea_uint_decrypt(internalDataBuffer, data_len, internalKeyBuffer);

    return xxtea_to_ubyte_array(internalDataBuffer, data_len, 1, out_len);
}

// public functions

bool xxtea_encrypt(const void * data, size_t len, const void * key, void* out, size_t * out_len) {
    FIXED_KEY
    internalOutputBuffer = (uint8_t*)out;
    return xxtea_ubyte_encrypt((const uint8_t *)data, len, fixed_key, out_len);
}

bool xxtea_decrypt(const void * data, size_t len, const void * key, void* out, size_t * out_len) {
    FIXED_KEY
    internalOutputBuffer = (uint8_t*)out;
    return xxtea_ubyte_decrypt((const uint8_t *)data, len, fixed_key, out_len);
}

void xxtea_init(uint32_t* dataBuffer, size_t dataBufferLen, uint32_t* keyBuffer, size_t keyBufferLen) {
    internalDataBufferLen = dataBufferLen;
    internalKeyBufferLen = keyBufferLen;

    internalDataBuffer = dataBuffer;
    internalKeyBuffer = keyBuffer;
}

