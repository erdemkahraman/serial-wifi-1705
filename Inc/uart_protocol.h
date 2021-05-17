#ifndef UART_PROTOCOL_H_
#define UART_PROTOCOL_H_

#include <stdint.h>

#define ANT_UART_PROTOCOL_VERSION   1

/* UART Connection */
#define ANT_UART_PARITY_NONE        0
#define ANT_UART_PARITY_EVEN        2
#define ANT_UART_PARITY_ODD         3
#define ANT_UART_PARITY_SPACE       4
#define ANT_UART_PARITY_MARK        5
#define ANT_UART_STOP_ONE           1
#define ANT_UART_STOP_ONEANDHALF    3
#define ANT_UART_STOP_TWO           2
#define ANT_UART_FLOW_NONE          0
#define ANT_UART_FLOW_HW            1
#define ANT_UART_FLOW_SW            2
#define ANT_UART_BAUD           38400
#define ANT_UART_BITS               8
#define ANT_UART_PARITY             (ANT_UART_PARITY_NONE)
#define ANT_UART_STOP               (ANT_UART_STOP_ONE)
#define ANT_UART_FLOW               (ANT_UART_FLOW_NONE)

/* Protocol */
#define ANT_UART_SOP            0xBA
#define ANT_UART_HEARTBEAT_TO   2000
#define UART_PACKET_SIZE          11
#define ANT_DEBUG_PACK_STR_LEN    26
#define ANT_MD5_LEN               16
#define ANT_MAX_FW_SIZE    (1024*80)
#define ANT_FW_DATA_SIZE          23

/* Others */
#define SAMPLE_RATE              500
#define CHAIN_VOLTAGE_THRESHOLD   10
#define CHAIN_VOLTAGE_ON           1
#define CHAIN_VOLTAGE_OFF          0
#define CHAIN_MAX_LETP_COUNT     455


/*
 * Rules :
 *   * Each request key must have identical response key. Same with structs.
 *   * Each structure must have UartReqPackHeader_s fields at the head.
 *   * Each structure must have UartReqPackFooter_s fields at the tail.
 *   * Each structore must defined between ""#pragma pack" directives.
 *   * Enumeration fields must indicate proper enumeration at comments
 *   * Each enumeration must have GUI string implementation macros.
 *   * Each structure must little or equal UART_PACKET_SIZE.
 */


/*
 * Usage :
 *   * Device sends Heartbeat after port open
 *   * Each side must return ACK for each pack, except ACK and Heartbeat.
 *   * Device must response for each request pack.
 *   * Device must response if any changes at individual structure.
 */


/*
 * WARNING : Careful while using non-aligned members:
 * http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.100748_0606_00_en/xxq1474359912082.html
 * http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka3934.html
 * */


/* Packets */

typedef enum UartReqPackKey_e {
    UartReqPack_Conf = 1,            // PC -> Ct, UartReqPackAck_t
    UartReqPack__Max_
} UartReqPackKey_t;

typedef enum UartResPackKey_e {
	uartResPack_Conf = 1,            // Ct -> PC, UartResPackAck_t
    UartResPack__Max_
} UartResPackKey_t;


/* Payloads */

typedef enum ChainState_e {
    ChainState_NotInitted = 0,
    ChainState_Initting,
    ChainState_Initted,
    ChainState__Max_
} ChainState_t;

typedef enum ChainItemState_e {
    ChainItemState_PoweredOff  = 0,
    ChainItemState_Ready,
    ChainItemState_Sleeping,
    ChainItemState_Armed,
    ChainItemState_Fireable,
    ChainItemState_Fired,
    ChainItemState_Skipped,
    ChainItemState_NotFired,
    ChainItemState__Max_
} ChainItemState_t;

typedef enum SetStateError_e {
    SetStateError_None = 0,
    SetStateError_InvalidIndex,
    SetStateError_InvalidId,
    SetStateError_InvalidState,
    SetStateError__Max_
} SetStateError_t;

#define DEVICE_TYPE_BOOTLOADER 0
#define DEVICE_TYPE_APPLICATION 1
typedef enum DeviceType_e {
    DeviceType_Bootloader = DEVICE_TYPE_BOOTLOADER,
    DeviceType_Application = DEVICE_TYPE_APPLICATION,
    DeviceType__Max_
} DeviceType_t;


/* fixme: All requests and response packets are XX bytes for now */
/*        Remove dummy sections, fix dynamic uart read           */


#pragma pack(push, 1)

/* Common (Req + Resp) */


/* Request */

typedef struct UartReqPackHeader_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // One of UartReqPackKey_t
    uint8_t  seq;                   // Incremental sequence number
} UartReqPackHeader_t;

typedef struct UartReqPackFooter_s {
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackFooter_t;

typedef struct UartReqPackConf_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_Ack
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  ADRF_id;               // chip id number
    uint16_t  TB334_SET;             // INT16_MIN to disable
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackConf_t;

typedef struct UartReqPackHeartbeat_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_Heartbeat
    uint8_t  seq;                   // Incremental sequence number
    uint32_t session_id;            // unique for each app start
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackHeartbeat_t;

typedef struct UartReqPackDebug_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_Debug
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  allowed_levels;        // alloved less than given here
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackDebug_t;

typedef struct UartReqPackBootloader_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_Bootloader
    uint8_t  seq;                   // Incremental sequence number
    uint32_t session_id;            // unique for each app start
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackBootloader_t;

typedef struct UartReqPackFirmware_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_Firmware
    uint8_t  seq;                   // Incremental sequence number
    uint16_t tot_pck;               // Total FW packet number
    uint16_t cur_pck;               // Current FW packet number
    uint8_t  data[ANT_FW_DATA_SIZE]; // FW Data
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackFirmware_t;

typedef struct UartReqPackMeasurement_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_Measurement
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackMeasurement_t;

typedef struct UartReqPackChainInit_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_ChainInit
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  havePlug;              // 0: false, else true
    uint8_t  thirtySec;             // 0: false, else true
    uint16_t scanCount;             // item count to scan, user input
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackChainInit_t;

typedef struct UartReqPackChainStatus_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_ChainStatus
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackChainStatus_t;

typedef struct UartReqPackChainItem_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_ChainItem
    uint8_t  seq;                   // Incremental sequence number
    uint16_t index;                 // index of item at chain to request, 0 >= index > length
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackChainItem_t;

typedef struct UartReqPackChainItemSetState_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_ChainItemSetState
    uint16_t index;                 // index of item at chain, 0 >= index > length
    uint32_t id;                    // for double check
    uint8_t  state;                 // ChainItemState_e, state to set, see letp-state-diagram.pdf for valid transitions !
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartReqPackChainItemSetState_t;




/* Response */

typedef struct UartResPackHeader_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // Any UartResPackKey_t
    uint8_t  seq;                   // Incremental sequence number
} UartResPackHeader_t;

typedef struct UartResPackFooter_s {
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackFooter_t;

typedef struct UartResPackAck_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_Ack
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  key_for_ack;           // One of UartReqPackKey_t
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackAck_t;

typedef struct UartResPackConf_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartReqPack_Ack
    uint8_t  id[4];                 // stm id number.
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackConf_t;

typedef struct UartResPackHeartbeat_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_Heartbeat
    uint8_t  seq;                   // Incremental sequence number
    uint32_t session_id;            // unique for each control start
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackHeartbeat_t;

typedef struct UartResPackDebug_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_Debug
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  lvl;                   // 0: most important, ..., 9: least important
    uint8_t  str[ANT_DEBUG_PACK_STR_LEN];
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackDebug_t;

typedef struct UartResPackBootloader_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_Bootloader
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  sw_version;            // Control software version
    uint8_t  bootloader_version;    // Bootloader version
    uint8_t  md5[ANT_MD5_LEN];      // Firmware MD5 sum
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackBootloader_t;

typedef struct UartResPackFirmware_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_Firmware
    uint8_t  seq;                   // Incremental sequence number
    uint16_t last_acked_pck_no;     // Last acknowledged packet no
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackFirmware_t;

typedef struct UartResPackMeasurement_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_Measurement
    uint8_t  seq;                   // Incremental sequence number
    uint32_t v[3];                 // 3 voltage measurement
    uint32_t i[3];                 // 3 current measurement
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackMeasurement_t;

typedef struct UartResPackChainInit_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_ChainInit
    uint8_t  seq;                   // Incremental sequence number
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackChainInit_t;

typedef struct UartResPackChainStatus_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_ChainStatus
    uint8_t  seq;                   // Incremental sequence number
    // chain
    uint8_t  state;                 // ChainState_e
    uint8_t  havePlug;              // 0: false, else true, requested value
    uint8_t  thirtySec;             // 0: false, else true, requested value
    uint16_t scanCount;             // item count to scan, user input, requested value
    // item
    uint16_t currentIndex;          // current ChainItem index which will take given set state commands
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackChainStatus_t;

typedef struct UartResPackChainItem_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_ChainItem
    uint8_t  seq;                   // Incremental sequence number
    uint16_t index;                 // Index of item
    uint32_t id;                    // Id of item
    uint16_t address;               // Address of item
    uint8_t  state;                 // ChainItemState_e
    uint16_t firmwareVersion;       // Firmware version
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackChainItem_t;

typedef struct UartResPackChainItemSetState_s {
    uint8_t  sop;                   // ANT_UART_SOP
    uint8_t  key;                   // UartResPack_ChainItemSetState
    uint16_t index;                 // index of current item at chain
    uint32_t id;                    // for double check current
    uint8_t  error;                 // SetStateError_e
    uint8_t  cka;                   // CRC field
    uint8_t  ckb;                   // CRC field
} UartResPackChainItemSetState_t;


#pragma pack(pop)




/*
  CK_A = 0, CK_B = 0
  For(I=0;I<N;I++)
  {
  CK_A = CK_A + Buffer[I]
  CK_B = CK_B + CK_A
}
*/

int  crc_is_valid(uint8_t * buff, uint32_t len);
#define CRC_IS_VALID_IMP() \
  int crc_is_valid(uint8_t * buff, uint32_t len) { \
    if (len < 4) return 0; \
    uint8_t ck_a = 0, ck_b = 0; \
    uint32_t i_a = len-2, i_b = len-1, i = 0; \
    for (i = 1; i < len-2; ++i) { \
      ck_a = (ck_a + buff[i]) & 0xFF; \
      ck_b = (ck_a + ck_b) & 0xFF; \
    } \
    return (buff[i_a] == ck_a && buff[i_b] == ck_b) ? 1 : 0; \
  }

void crc_fill(uint8_t * buff, uint32_t len);
#define CRC_FILL_IMP() \
  void crc_fill(uint8_t * buff, uint32_t len) { \
    uint8_t ck_a = 0, ck_b = 0; \
    uint32_t i_a = len-2, i_b = len-1, i = 0; \
    for (i = 1; i < len-2; ++i) { \
      ck_a = (ck_a + buff[i]) & 0xFF; \
      ck_b = (ck_a + ck_b) & 0xFF; \
    } \
    buff[i_a] = ck_a; \
    buff[i_b] = ck_b; \
  }

#define SIZE_OF_REQ_PACK_IMP() \
    int sizeOfReqPack(UartReqPackKey_t key) { \
      switch (key) { \
      case UartReqPack_Ack:               return sizeof(UartReqPackAck_t); break; \
      case UartReqPack_Heartbeat:         return sizeof(UartReqPackHeartbeat_t); break; \
      case UartReqPack_Debug:             return sizeof(UartReqPackDebug_t); break; \
      case UartReqPack_Bootloader:        return sizeof(UartReqPackBootloader_t); break; \
      case UartReqPack_Firmware:          return sizeof(UartReqPackFirmware_t); break; \
      case UartReqPack_Measurement:       return sizeof(UartReqPackMeasurement_t); break; \
      case UartReqPack_ChainInit:         return sizeof(UartReqPackChainInit_t); break; \
      case UartReqPack_ChainStatus:       return sizeof(UartReqPackChainStatus_t); break; \
      case UartReqPack_ChainItem:         return sizeof(UartReqPackChainItem_t); break; \
      case UartReqPack_ChainItemSetState: return sizeof(UartReqPackChainItemSetState_t); break; \
      default: \
        return 0; break; \
      } \
    }

#define SIZE_OF_RES_PACK_IMP() \
    int sizeOfResPack(UartResPackKey_t key) { \
      switch (key) { \
        case UartResPack_Ack:               return sizeof(UartResPackAck_t); break; \
        case UartResPack_Heartbeat:         return sizeof(UartResPackHeartbeat_t); break; \
        case UartResPack_Debug:             return sizeof(UartResPackDebug_t); break; \
        case UartResPack_Bootloader:        return sizeof(UartResPackBootloader_t); break; \
        case UartResPack_Firmware:          return sizeof(UartResPackFirmware_t); break; \
        case UartResPack_Measurement:       return sizeof(UartResPackMeasurement_t); break; \
        case UartResPack_ChainInit:         return sizeof(UartResPackChainInit_t); break; \
        case UartResPack_ChainStatus:       return sizeof(UartResPackChainStatus_t); break; \
        case UartResPack_ChainItem:         return sizeof(UartResPackChainItem_t); break; \
        case UartResPack_ChainItemSetState: return sizeof(UartResPackChainItemSetState_t); break; \
        default: \
          return 0; break; \
      } \
    }

#define IS_STATE_TRANSITION_VALID_IMP() \
    bool isStateTransitionValid(ChainItemState_t current, ChainItemState_t next) { \
        switch (next) { \
        case ChainItemState_Ready:      return ( \
                        current == ChainItemState_PoweredOff || \
                        current == ChainItemState_Armed || \
                        current == ChainItemState_Fireable); \
        case ChainItemState_Armed:      return current == ChainItemState_Ready; \
        case ChainItemState_Fireable:   return current == ChainItemState_Armed; \
        case ChainItemState_Fired:      return current == ChainItemState_Fireable; \
        case ChainItemState_Skipped:    return current != ChainItemState_Fired; \
        case ChainItemState_NotFired:   return current == ChainItemState_Fireable; \
        default: break; \
        } \
        return false; \
    }


#ifdef DEFINITIONS_FOR_GUI

#define INIT_REQ_PACK_STRING_LIST() \
    QStringList reqKeyStringList = QStringList() \
    << QObject::tr("Ack") \
    << QObject::tr("Heartbeat") \
    << QObject::tr("Debug") \
    << QObject::tr("Bootloader") \
    << QObject::tr("Firmware") \
    << QObject::tr("Measurement") \
    << QObject::tr("ChainInit") \
    << QObject::tr("ChainStatus") \
    << QObject::tr("ChainItem") \
    << QObject::tr("ChainItemSetState") \
    ;
#define INIT_RES_PACK_STRING_LIST() \
    QStringList resKeyStringList = QStringList() \
    << QObject::tr("Ack") \
    << QObject::tr("Heartbeat") \
    << QObject::tr("Debug") \
    << QObject::tr("Bootloader") \
    << QObject::tr("Firmware") \
    << QObject::tr("Measurement") \
    << QObject::tr("ChainInit") \
    << QObject::tr("ChainStatus") \
    << QObject::tr("ChainItem") \
    << QObject::tr("ChainItemSetState") \
    ;

#define INIT_CHAIN_STATE_STRING_LIST() \
    QStringList chainStateStringList = QStringList() \
    << QObject::tr("Not Initted") \
    << QObject::tr("Initting") \
    << QObject::tr("Initted") \
    ;

#define INIT_CHAIN_ITEM_STATE_STRING_LIST() \
    QStringList chainItemStateStringList = QStringList() \
    << QObject::tr("Powered Off") \
    << QObject::tr("Ready") \
    << QObject::tr("Sleeping") \
    << QObject::tr("Armed") \
    << QObject::tr("Fireable") \
    << QObject::tr("Fired") \
    << QObject::tr("Skipped") \
    << QObject::tr("Not Fired") \
    ;

#define INIT_SET_STATE_ERROR_STRING_LIST() \
    QStringList setStateErrorStringList = QStringList() \
    << QObject::tr("None") \
    << QObject::tr("InvalidIndex") \
    << QObject::tr("InvalidId") \
    << QObject::tr("InvalidState") \
    ;

#define INIT_DEVICE_TYPE_STRING_LIST() \
    QStringList deviceTypeStringList = QStringList() \
    << QObject::tr("Bootloader") \
    << QObject::tr("Application") \
    ;

///! depends INIT_REQ_PACK_STRING_LIST
#define REQ_KEY_TO_STRING_IMP() \
    QString reqKeyToString(int key) { \
        if (key < 0 || key >= reqKeyStringList.count()) return QString("%1").arg(key); \
        return reqKeyStringList.at(key); \
    }

///! depends INIT_RES_PACK_STRING_LIST
#define RES_KEY_TO_STRING_IMP() \
    QString resKeyToString(int key) { \
        if (key < 0 || key >= resKeyStringList.count()) return QString("%1").arg(key); \
        return resKeyStringList.at(key); \
    }

///! depends INIT_CHAIN_STATE_STRING_LIST
#define CHAIN_STATE_TO_STRING_IMP() \
    QString chainStateToString(int state) { \
        if (state < 0 || state >= chainStateStringList.count()) QString("%1").arg(state); \
        return chainStateStringList.at(state); \
    }

///! depends INIT_CHAIN_ITEM_STATE_STRING_LIST
#define CHAIN_ITEM_STATE_TO_STRING_IMP() \
    QString chainItemStateToString(int state) { \
        if (state < 0 || state >= chainItemStateStringList.count()) QString("%1").arg(state); \
        return chainItemStateStringList.at(state); \
    }

///! depends INIT_SET_STATE_ERROR_STRING_LIST
#define SET_STATE_ERROR_TO_STRING_IMP() \
    QString setStateErrorToString(int error) { \
        if (error < 0 || error >= setStateErrorStringList.count()) QString("%1").arg(error); \
        return setStateErrorStringList.at(error); \
    }




///! depends REQ_KEY_TO_STRING_IMP
#define REQ_ACK_TO_STRING(req) \
    QString("key_for_ack:%2(%1)").arg(req.key_for_ack).arg(resKeyToString(req.key_for_ack))

#define REQ_HEARTBEAT_TO_STRING(req) \
    QString("session_id:%1").arg(req.session_id)

#define REQ_DEBUG_TO_STRING(req) \
    QString("allowed_levels:%1").arg(req.allowed_levels)

#define REQ_FIRMWARE_TO_STRING(req) \
    QString("cur_pck:%1/tot_pck:%2,data:[%3])") \
                .arg(req.cur_pck).arg(req.tot_pck) \
                .arg(QString::fromUtf8(QByteArray((const char *)req.data, sizeof(req.data)).toHex().toUpper()))

#define REQ_CHAIN_INIT_TO_STRING(req) \
    QString("havePlug:%1,thirtySec:%2,scanCount:%3") \
                .arg(req.havePlug).arg(req.thirtySec).arg(req.scanCount)

#define REQ_CHAIN_ITEM_TO_STRING(req) \
    QString("index:%1").arg(req.index)

///! depends CHAIN_ITEM_STATE_TO_STRING_IMP
#define REQ_CHAIN_ITEM_SET_STATE_TO_STRING(req) \
    QString("index:%1,id:%2,state:%4(%3)") \
                .arg(req.index).arg(QString("%1").arg(req.id, 6, 16, QLatin1Char('0')).toUpper()) \
                .arg(req.state).arg(chainItemStateToString(req.state))



///! depends RES_KEY_TO_STRING_IMP
#define RES_ACK_TO_STRING(res) \
    QString("key_for_ack:%2(%1)").arg(res.key_for_ack).arg(reqKeyToString(res.key_for_ack))

#define RES_HEARTBEAT_TO_STRING(res) \
    QString("session_id:%1").arg(res.session_id)

#define RES_DEBUG_TO_STRING(lvl,str) \
    QString("lvl:%1,str:%2").arg(lvl).arg(str)

#define RES_BOOTLOADER_TO_STRING(res) \
    QString("sw:%1,bl:%2,md5:%3") \
                .arg(res.sw_version) \
                .arg(res.bootloader_version) \
                .arg(QString::fromUtf8(QByteArray((const char *)res.md5, sizeof(res.md5)).toHex().toUpper()))

#define RES_FIRMWARE_TO_STRING(res) \
    QString("last_acked_pck_no:%1").arg(res.last_acked_pck_no)

#define RES_MEASUREMENT_TO_STRING(res) \
    QString("v0:%1,v1:%2,v2:%3,i0:%4,i1:%5,i2:%6") \
                .arg(res.v[0]).arg(res.v[1]).arg(res.v[2]) \
                .arg(res.i[0]).arg(res.i[1]).arg(res.i[2])

#define RES_CHAIN_STATUS_TO_STRING(res) \
    QString("state:%2(%1),havePlug:%3,thirtySec:%4,scanCount:%5,currentIndex:%6") \
                .arg(res.state).arg(chainStateToString(res.state)) \
                .arg(res.havePlug).arg(res.thirtySec) \
                .arg(res.scanCount).arg(res.currentIndex)

#define RES_CHAIN_ITEM_TO_STRING(res) \
    QString("index:%1,id:%2,address:%3,state:%5(%4),firmwareVersion:%6") \
                .arg(res.index) \
                .arg(QString("%1").arg(res.id, 6, 16, QLatin1Char('0')).toUpper()) \
                .arg(QString("%1").arg(res.address, 4, 16, QLatin1Char('0')).toUpper()) \
                .arg(res.state).arg(chainItemStateToString(res.state)) \
                .arg(res.firmwareVersion)

///! depends CHAIN_ITEM_STATE_TO_STRING_IMP
#define RES_CHAIN_ITEM_SET_STATE_TO_STRING(res) \
    QString("index:%1,id:%2,state:%4(%3)") \
                .arg(res.index) \
                .arg(QString("%1").arg(res.id, 6, 16, QLatin1Char('0')).toUpper()) \
                .arg(res.error).arg(setStateErrorToString(res.error))



#endif  // DEFINITIONS_FOR_GUI




#endif  // UART_PROTOCOL_H_
