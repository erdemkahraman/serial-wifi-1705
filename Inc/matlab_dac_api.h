/*
  Copyright 2017 Antsis Elektronik Ltd.
 */

#ifndef DOC_API_DAC_V1_H_
#define DOC_API_DAC_V1_H_

#include <stdint.h>



#define DAC_API_VERSIN_1 1

#define DAC_API_SOP0 0xBE
#define DAC_API_SOP1 0xAF

#define DAC_API_GW_ID_LEN 6
#define DAC_API_NODE_ID_LEN 6


#pragma pack(1)

typedef enum {
  DAC_API_PING = 0,       // Payload: -. Gw to server.
  DAC_API_PING_RES,       // Payload: -. Server to gw.
  DAC_API_CONF,           // Payload: dac_api_gw_conf_s.
  DAC_API_CONF_RES,       // Payload: dac_api_gw_conf_res_s.
  DAC_API_POST,           // Payload: dac_api_post_s.
  DAC_API_POST_RES,       // Payload: dac_api_post_res_s.
  DAC_API_AGRO_CONF_RES,  // Payload: dac_api_gw_conf_res_agro_s.
  DAC_API_AGRO_POST,      // Payload: dac_api_post_agro_s. Response: DAC_API_POST_RES.

  DAC_API_MAX
} dac_api_e;

typedef enum {
  DAC_API_GW_TYPE_STATION = 0,            // Cold chain
  DAC_API_GW_TYPE_MOBILE,                 // Cold chain
  DAC_API_GW_TYPE_AGRO,                   // Precision farming

  DAC_API_GW_TYPE_MAX
} dac_api_gw_type_e;

/*
  CRC
  ===
  Applied between SOP and CRC bytes.

  [ BE AF 01 XX XX XX XX AA BB ]
         |--------------|

  Algorithm
  ---------
  CK_A = 0, CK_B = 0
  For(I=0;I<N;I++)
  {
    CK_A = CK_A + Buffer[I]
    CK_B = CK_B + CK_A
  }
*/

typedef struct dac_api_pack_s {
  uint8_t   sop0;                    // Start of package. Constant = 0xBE.
  uint8_t   sop1;                    // Start of package. Constant = 0xAF.
  uint8_t   ver;                     // Version of protocol. 1 for v1.
  uint8_t   dac;                     // One of dac_api_e.
  uint16_t  len;                     // Length of data
  uint8_t * data;                    // One of the structs below.
  uint8_t   ck_a;                    // CRC
  uint8_t   ck_b;                    // CRC
} dac_api_pack_t;


typedef struct dac_api_gw_conf_s {
  uint8_t   id[DAC_API_GW_ID_LEN];
  uint8_t   gw_type;                 // One of dac_api_gw_type_e
  uint8_t   has_loc;                 // 0: false, >0: true
  double    lat;
  double    lng;
} dac_api_gw_conf_t;


typedef struct dac_api_thresholds_s {
  int8_t   RF1_min;                // INT16_MIN to disable
  int8_t   RF1_max;                // INT16_MAX to disabe
  int8_t    RF2_min;            // CHAR_MIN to disable
  int8_t    RF2_max;            // CHAR_MAX to disable
  int8_t    RF3_min;             // CHAR_MIN to disable
  int8_t    RF3_max;             // CHAR_MAX to disable
} dac_api_thresholds_t;

typedef struct dac_api_gw_conf_res_s {

  dac_api_thresholds_t thresholds;   // If passes any threshold, starts
                                     // posting with the intervals above.
} dac_api_gw_conf_res_t;


typedef struct dac_api_post_s {
  uint8_t   id[DAC_API_GW_ID_LEN];
  uint8_t   lat[4];
  uint8_t   lng[4];
} dac_api_post_t;


typedef struct dac_api_post_res_s {
  uint8_t   is_saved;               // 0: false, >0: true
} dac_api_post_res_t;


#pragma pack()

#endif  // DOC_API_DAC_V1_H_
