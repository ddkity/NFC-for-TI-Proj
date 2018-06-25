
#ifndef __TAG_READ_H__
#define __TAG_READ_H__
#include <stdio.h>
#include <stdint.h>

// use PWD_AUTH command
#define TAG_USE_PWD_AUTH

#define TAG_UID_START_BLOCK    (0x00)

#define TAG_INFO_START_BLOCK    (0x10)
#define TAG_AUTH_START_BLOCK    (0x14)


#define ENC_KEY_LEN      (16)

#define TAG_UID_LEN      (8)
#define FILTER_UID_LEN   (4)

#define TOTAL_TAG_NUM   (4)

#define PWD0   (0x56)
#define PWD1   (0x9D)
#define PWD2   (0xF5)
#define PWD3   (0xC9)

#define PACK0       (0x66)
#define PACK1       (0x88)

typedef struct _TAG_INFO_T {
    uint8_t tag_uid[TAG_UID_LEN];
    uint8_t filter_uid[FILTER_UID_LEN];
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t manufacturer;
    uint8_t filter_type;
    uint8_t is_auth;
}TAG_INFO_T;


void set_cur_tag_id(uint8_t id);

uint8_t parse_tag_info(void);

const TAG_INFO_T * get_tag_info(uint8_t id);

int8_t clear_tag_info(uint8_t id);

#endif

