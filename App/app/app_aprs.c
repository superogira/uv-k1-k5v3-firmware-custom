#include "app_aprs.h"
#include "gps_app.h"
#include <string.h>
#include <stdio.h> // ใส่ไว้แต่ยังไม่ใช้
//#include <stdlib.h>
#include "py32f0xx.h"
#include "driver/bk4819.h"
#include "driver/system.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "ui/helper.h"
//#include "audio.h"
// #include "settings.h" // ปิดไว้ก่อน ตัวปัญหา
#include "radio.h"
#include "app.h"
#include "functions.h"
//#include "driver/systick.h"
//#include "driver/bk4819.h"

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
   BK4819 / Radio specific (ต้องมีใน firmware เดิมคุณอยู่แล้ว)
   ========================================================= */
// ดึงฟังก์ชันแปลง string เป็น float แบบเบาๆ มาจาก gps_app.c
extern float my_atof(const char* s);

extern void RADIO_PrepareTX(void);
extern void APP_EndTransmission(void);

extern void SYSTEM_DelayMs(uint32_t ms);
extern void __disable_irq(void);
extern void __enable_irq(void);

extern VFO_Info_t *gTxVfo;
extern void RADIO_ConfigureSquelchAndOutputPower(VFO_Info_t *pInfo);
extern void BK4819_SetupPowerAmplifier(uint8_t Bias, uint32_t Frequency);

/* =========================================================
   AFSK / APRS constants
   ========================================================= */
#define TONE_MARK   12389   // 1200 Hz
#define TONE_SPACE  22706   // 2200 Hz
#define BIT_US      250     // 1200 baud (1 / 1200)

/* =========================================================
   Encode buffer
   ========================================================= */
//static uint8_t encoded_buffer[512];
static uint8_t encoded_buffer[256];
static int encoded_bit_idx;
static int ones_count;

/* =========================================================
   Simple busy-wait delay (ปรับ factor ตาม clock MCU)
   ========================================================= */
static inline void delay_bit_time(void)
{
    /* ค่า 8 เหมาะกับ MCU ~48 MHz
       ถ้าเร็ว/ช้าไป ปรับเลขนี้เล็กน้อย */
    for (volatile int i = 0; i < (BIT_US * 8); i++)
        __asm volatile ("nop");
}

/* =========================================================
   CRC-16 CCITT (AX.25, LSB first)
   ========================================================= */
uint16_t calc_crc(uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFF;
}

/* =========================================================
   Callsign formatting (AX.25 address field)
   ========================================================= */
void format_callsign(char *out, char *callsign, int ssid, bool is_last)
{
    char tmp[6];
    memset(tmp, ' ', 6);

    int len = strlen(callsign);
    if (len > 6) len = 6;
    memcpy(tmp, callsign, len);

    for (int i = 0; i < 6; i++)
        out[i] = tmp[i] << 1;

    out[6] = 0x60 | ((ssid & 0x0F) << 1);
    if (is_last)
        out[6] |= 0x01;
}


// =========================================================
// Helper: แปลง Decimal Degrees -> APRS Format (DDMM.mmN)
// =========================================================
// ตัวอย่าง: 13.74147 -> 1344.49N
/* void convert_decimal_to_aprs(char *decimal_str, char *aprs_out, bool is_lat) {
    if (strlen(decimal_str) == 0) return;
    
    double val = atof(decimal_str);
    double abs_val = (val < 0) ? -val : val;
    
    int deg = (int)abs_val;
    double min = (abs_val - deg) * 60.0;
    
    char hem;
    if (is_lat) {
        hem = (val >= 0) ? 'N' : 'S';
        // Format: DDMM.mmH (8 chars)
        sprintf(aprs_out, "%02d%05.2f%c", deg, min, hem);
    } else {
        hem = (val >= 0) ? 'E' : 'W';
        // Format: DDDMM.mmH (9 chars)
        sprintf(aprs_out, "%03d%05.2f%c", deg, min, hem);
    }
} */

/* =========================================================
   Bit buffer helpers (RAW bits only!)
   ========================================================= */
static inline void push_bit(uint8_t bit)
{
    int byte = encoded_bit_idx >> 3;
    int pos  = encoded_bit_idx & 7;

    if (pos == 0)
        encoded_buffer[byte] = 0;

    if (bit)
        encoded_buffer[byte] |= (1 << pos);

    encoded_bit_idx++;
}

/* AX.25 Flag 0x7E = 01111110 (LSB first) */
static void send_flag(void)
{
    push_bit(0);
    push_bit(1); push_bit(1); push_bit(1);
    push_bit(1); push_bit(1); push_bit(1);
    push_bit(0);
}

/* =========================================================
   Encode AX.25 -> raw bit stream (with bit stuffing)
   ========================================================= */
void APRS_EncodePacket(uint8_t *packet, int len)
{
    encoded_bit_idx = 0;
    ones_count = 0;

    /* Preamble: ใช้ค่าจาก Config แทนเลข 150 */
    // for (int i = 0; i < 150; i++)  <-- ของเดิม
    for (int i = 0; i < aprs_config.preamble; i++) // <-- ของใหม่
        send_flag();

    /* Data + FCS */
    for (int i = 0; i < len; i++) {
        uint8_t b = packet[i];
        for (int k = 0; k < 8; k++) {
            uint8_t bit = (b >> k) & 1;
            push_bit(bit);

            if (bit) {
                ones_count++;
                if (ones_count == 5) {
                    push_bit(0);     // bit stuffing
                    ones_count = 0;
                }
            } else {
                ones_count = 0;
            }
        }
    }

    /* Postamble */
    for (int i = 0; i < 5; i++)
        send_flag();
}

/* =========================================================
   AFSK + NRZI transmitter (หัวใจ APRS)
   ========================================================= */
void APRS_Send_Audio(uint8_t *bits, int total_bits)
{
    BK4819_WriteRegister(BK4819_REG_70, (1u << 15) | (10u << 8)); // Tone enable
    BK4819_WriteRegister(BK4819_REG_58, 0);                      // Disable FSK
    SYSTEM_DelayMs(10);

    __disable_irq();

    int tone = 1;        // NRZI start = MARK (1200 Hz)
    int last_tone = -1;

    for (int i = 0; i < total_bits; i++) {
        int byte = i >> 3;
        int pos  = i & 7;
        int bit  = (bits[byte] >> pos) & 1;

        if (bit == 0)
            tone ^= 1;   // NRZI toggle on zero

        if (tone != last_tone) {
            BK4819_WriteRegister(
                BK4819_REG_71,
                tone ? TONE_MARK : TONE_SPACE
            );
            last_tone = tone;
        }

        delay_bit_time();
    }

    __enable_irq();

    BK4819_WriteRegister(BK4819_REG_70, 0);
    BK4819_WriteRegister(BK4819_REG_71, 0);
}

// =========================================================
// Helper: แปลง Decimal Degrees -> APRS Format (DDMM.mmN)
// แบบประหยัดเมม (ไม่ใช้ sprintf %f)
// =========================================================
void convert_decimal_to_aprs(char *decimal_str, char *aprs_out, bool is_lat) {
    if (decimal_str[0] == 0) return;
    
    // 1. ใช้ฟังก์ชันเบาๆ แปลง string เป็น float (ยืมจาก gps_app.c)
    float val = my_atof(decimal_str);
    float abs_val = (val < 0) ? -val : val;
    
    // 2. คำนวณมือ (Manual Calculation)
    int deg = (int)abs_val;
    float min_full = (abs_val - deg) * 60.0f;
    int min_int = (int)min_full;
    int min_dec = (int)((min_full - min_int) * 100); // เอาทศนิยม 2 ตำแหน่ง

    // 3. สร้าง String ด้วย %d (เลขจำนวนเต็ม) ซึ่งกินพื้นที่น้อยกว่า %f มาก
    char hem;
    if (is_lat) {
        hem = (val >= 0) ? 'N' : 'S';
        // Format: DDMM.mmH
        sprintf(aprs_out, "%02d%02d.%02d%c", deg, min_int, min_dec, hem);
    } else {
        hem = (val >= 0) ? 'E' : 'W';
        // Format: DDDMM.mmH
        sprintf(aprs_out, "%03d%02d.%02d%c", deg, min_int, min_dec, hem);
    }
}

/* =========================================================
   APRS Beacon
   ========================================================= */
   
// เพิ่มฟังก์ชันอ่านความถี่จาก Hardware โดยตรง (วางไว้ก่อน APP_RunAPRS)
uint32_t APRS_GetCurrentFrequency(void) {
    // อ่านค่าจาก Register 38 และ 39 ของ BK4819
    // Reg 39 = High 16 bits
    // Reg 38 = Low 16 bits
    uint16_t f_high = BK4819_ReadRegister(BK4819_REG_39);
    uint16_t f_low  = BK4819_ReadRegister(BK4819_REG_38);
    
    uint32_t freq = ((uint32_t)f_high << 16) | f_low;
    
    // ปกติ BK4819 เก็บความถี่ในหน่วย 10Hz (เช่น 145.525 MHz = 14552500)
    // แต่บาง Firmware อาจเก็บเป็นหน่วยอื่น ถ้าค่าออกมาแปลกๆ ต้องมาดูตัวคูณอีกที
    return freq;
}

// ฟังก์ชันตั้งความถี่ (เขียนลง Register โดยตรง)
void APRS_SetFrequency(uint32_t freq_10hz) {
    // แยก 32-bit เป็น 16-bit 2 ก้อน
    uint16_t f_high = (freq_10hz >> 16) & 0xFFFF;
    uint16_t f_low  = freq_10hz & 0xFFFF;
    
    // เขียนลง Register 39 (High) และ 38 (Low)
    BK4819_WriteRegister(BK4819_REG_39, f_high);
    BK4819_WriteRegister(BK4819_REG_38, f_low);
}
   
// ฟังก์ชันแปลง String เป็น Int แบบเบาๆ (ใช้แทน atoi)
int custom_atoi(char *s) {
    int res = 0;
    while (*s >= '0' && *s <= '9') {
        res = res * 10 + (*s - '0');
        s++;
    }
    return res;
}

static uint8_t packet[128];

// แก้ไขฟังก์ชันส่ง ให้ดึงข้อมูลจาก aprs_config
void APRS_SendBeacon_Now(void) {
    // เช็คก่อนว่าเปิด APRS หรือไม่ (ถ้าเรียก manual อาจจะข้ามเช็คนี้ได้)
    // if (!aprs_config.aprs_on) return; 
	
	// 1. จำความถี่เดิมไว้ก่อน
    //uint32_t original_freq = BK4819_GetFrequency();
	uint32_t original_freq = APRS_GetCurrentFrequency();
	
	// จำค่ากำลังส่งเดิม และความถี่เดิมในระบบไว้
    uint8_t original_power = gTxVfo->OUTPUT_POWER;
    uint32_t original_vfo_freq = gTxVfo->pTX->Frequency;
	
	// แปลงความถี่ APRS จาก Config (Hz) เป็นหน่วย 10Hz
    // ตัวอย่าง: 144390000 Hz -> 14439000 (10Hz units)
    uint32_t aprs_freq_10hz = aprs_config.tx_freq / 10;

    //strcpy(status_msg, "TX..."); // ถ้ามีตัวแปร status_msg
    UI_DisplayClear();
    UI_PrintStringSmallBold("TX APRS...", 0, 0, 3);
	char buf[32];
	
/*     sprintf(buf, "%lu.%05lu", aprs_freq_10hz/100000, aprs_freq_10hz%100000);
    UI_PrintStringSmallNormal(buf, 0, 0, 4);
    ST7565_BlitFullScreen(); */
	
	// แสดงความถี่และกำลังส่ง
    //char *pwr_str = (aprs_config.tx_power==0) ? "L" : (aprs_config.tx_power==1 ? "M" : "H");
	char pwr_str[4];
    if (aprs_config.tx_power < 5) sprintf(pwr_str, "L%d", aprs_config.tx_power + 1);
    else if (aprs_config.tx_power == 5) strcpy(pwr_str, "M");
    else strcpy(pwr_str, "H");
	
    sprintf(buf, "%lu.%05lu %s", aprs_freq_10hz/100000, aprs_freq_10hz%100000, pwr_str);
    UI_PrintStringSmallNormal(buf, 0, 0, 4);
    ST7565_BlitFullScreen();

	
	// 2. ตั้งค่าความถี่และกำลังส่ง APRS ลงใน VFO ชั่วคราว
    gTxVfo->pTX->Frequency = aprs_freq_10hz;  // ต้องใส่ความถี่นี้เพื่อให้คำนวณ Calibration ถูกย่าน
    //gTxVfo->OUTPUT_POWER = aprs_config.tx_power; // 0=Low, 1=Mid, 2=High
	// ********** แก้ไขการ Map ค่ากำลังส่ง **********
    if (aprs_config.tx_power < 5) {
        // กรณี L1 - L5 (aprs_config 0-4) -> Map ไปเป็น 1-5 ของ Firmware
        gTxVfo->OUTPUT_POWER = aprs_config.tx_power + 1; 
    } 
    else if (aprs_config.tx_power == 5) {
        // กรณี MID -> Map ไปเป็น 6
        gTxVfo->OUTPUT_POWER = 6;
    } 
    else {
        // กรณี HIGH (หรืออื่นๆ) -> Map ไปเป็น 7
        gTxVfo->OUTPUT_POWER = 7;
    }
    // *****************************************
	
	// คำนวณค่า Bias ของ PA ใหม่ (ตามความถี่และกำลังส่ง APRS)
    RADIO_ConfigureSquelchAndOutputPower(gTxVfo);
	
	// สั่ง Hardware ให้ใช้ความถี่และกำลังส่งที่คำนวณได้
    APRS_SetFrequency(aprs_freq_10hz);
    BK4819_SetupPowerAmplifier(gTxVfo->TXP_CalculatedSetting, gTxVfo->pTX->Frequency);
	
	// 2. ตั้งความถี่สำหรับ APRS (จาก config)
    // ต้องแปลง 144.39000 เป็นหน่วยที่ BK4819 เข้าใจ (ปกติต้องดู radio.c ว่า set ยังไง)
    // สมมติฟังก์ชัน BK4819_SetFrequency รับหน่วย 10Hz
	// 2. สลับไปความถี่ APRS
    //BK4819_SetFrequency(aprs_freq_10hz);
	SYSTEM_DelayMs(50); // รอให้ PLL Lock นิดนึง

    RADIO_PrepareTX();
    SYSTEM_DelayMs(100);

    if (gCurrentFunction == FUNCTION_TRANSMIT) {
        
        // --- 1. เตรียม Packet ---
        int idx = 0;
        memset(packet, 0, sizeof(packet));

        // Header: ใช้ค่าจาก Config
		// 1. Destination (To Call)
        format_callsign((char*)&packet[idx], aprs_config.dest_call, 0, false); idx += 7;
		
		// 2. Source Call (My Call)
        // เช็คว่ามี Path ไหม? ถ้าไม่มี Path เลย ให้ Source เป็นตัวสุดท้าย (is_last = true)
		//format_callsign((char*)&packet[idx], aprs_config.callsign, aprs_config.ssid, false); idx += 7;
        bool has_path = (strlen(aprs_config.digipath) > 0);
        format_callsign((char*)&packet[idx], aprs_config.callsign, aprs_config.ssid, !has_path); idx += 7;
		
        //format_callsign((char*)&packet[idx], "WIDE1", 1, true); idx += 7; // Path อาจต้องเขียน logic เพิ่มถ้าจะแก้ได้
		// 3. Path (ถ้ามี)
        if (has_path) {
            // แยกชื่อ Path กับ SSID (เช่น WIDE1-1 -> Name=WIDE1, SSID=1)
            char path_name[10];
            int path_ssid = 0;
            
            // หาเครื่องหมายขีด '-'
            char *dash = strchr(aprs_config.digipath, '-');
            if (dash) {
                // กรณีมี SSID (เช่น WIDE1-1)
                int len = dash - aprs_config.digipath;
                if(len > 6) len = 6;
                memcpy(path_name, aprs_config.digipath, len);
                path_name[len] = 0;
                //path_ssid = atoi(dash + 1); // แปลงเลขหลังขีดเป็น int
				path_ssid = custom_atoi(dash + 1);
            } else {
                // กรณีไม่มี SSID (เช่น WIDE1)
                strncpy(path_name, aprs_config.digipath, 6);
                path_name[6] = 0;
                path_ssid = 0;
            }
            
            // ใส่ Path ลงไป (กำหนดให้เป็นตัวสุดท้ายเสมอ is_last=true)
            format_callsign((char*)&packet[idx], path_name, path_ssid, true); idx += 7;
        }

        packet[idx++] = 0x03; 
        packet[idx++] = 0xF0; 

        // Payload: สร้างตามรูปแบบ APRS Mic-E หรือ Text ธรรมดา
        // เพื่อความง่าย ใช้รูปแบบ Text: =Lat/Lon>SymbolComment
        // ตัวอย่าง: =1344.48N/10030.00E>Test
        
        char payload[100];
		//char lat_aprs[10]; // เก็บพิกัดที่แปลงแล้ว
        //char lon_aprs[11];
		// กำหนดค่าเริ่มต้นเป็น 0 หรือค่าที่ปลอดภัย เพื่อไม่ให้ส่งขยะออกไป
		char lat_aprs[10] = "0000.00N"; 
		char lon_aprs[11] = "00000.00E";
        //char lat_str[15], lon_str[15];
		
		// **********************************************************
        // [แก้ไข 1] เพิ่มการแปลงพิกัดตรงนี้ครับ ไม่งั้นพิกัดจะเป็น 0 ตลอด
        // **********************************************************
        if (aprs_config.use_gps) {
            // แปลงพิกัดจาก GPS (val_lat/val_lon) เป็นรูปแบบ APRS
            convert_decimal_to_aprs(val_lat, lat_aprs, true);
            convert_decimal_to_aprs(val_lon, lon_aprs, false);
        } else {
            // แปลงพิกัดจากค่า Fixed ที่ตั้งไว้
            convert_decimal_to_aprs(aprs_config.fixed_lat, lat_aprs, true);
            convert_decimal_to_aprs(aprs_config.fixed_lon, lon_aprs, false);
        }
        // **********************************************************

        // ถ้าใช้ GPS ให้ส่ง Speed/Course/Altitude ด้วย
        if (aprs_config.use_gps) {
            // 1. แปลง Speed (km/h -> Knots)
            float spd_kmh = my_atof(val_speed);
            int spd_knots = (int)(spd_kmh / 1.852);
            
            // 2. แปลง Altitude (m -> Feet)
            float alt_m = my_atof(val_alt);
            int alt_ft = (int)(alt_m * 3.28084);
            
            // 3. แปลง Course (String -> Int)
            int course = custom_atoi(val_course);
            
            // 4. สร้าง Packet ตามมาตรฐาน: =LAT/LON>CSE/SPD/A=ALTComment
            // %03d/%03d คือ ทิศทาง 3 หลัก / ความเร็ว 3 หลัก
            // /A=%06d   คือ ความสูง 6 หลัก (หน่วยฟุต)
            sprintf(payload, "=%s%c%s%c%03d/%03d/A=%06d%s", 
                    lat_aprs, 
                    aprs_config.icon_table, 
                    lon_aprs, 
                    aprs_config.icon_symbol,
                    course, spd_knots,      // เพิ่ม CSE/SPD
                    alt_ft,                 // เพิ่ม ALT
                    aprs_config.comment);
        } 
        else {
            // กรณี Fixed Position ส่งแบบเดิม (ไม่มี Speed/Course)
            sprintf(payload, "=%s%c%s%c%s", 
                    lat_aprs, 
                    aprs_config.icon_table, 
                    lon_aprs, 
                    aprs_config.icon_symbol,
                    aprs_config.comment);
        }
		
        int pay_len = strlen(payload);
        memcpy(&packet[idx], payload, pay_len);
        idx += pay_len;

        // --- 2. CRC ---
        uint16_t fcs = calc_crc(&packet[0], idx); 
        packet[idx++] = fcs & 0xFF;        
        packet[idx++] = (fcs >> 8) & 0xFF; 

        // --- 3. Send ---
        BK4819_DisableDTMF();
        //APRS_SendPacket_Encoded(packet, idx);
		// 1. แปลง Packet เป็น Bit Stream (ใส่ Buffer)
        APRS_EncodePacket(packet, idx); 
        
        // 2. ส่งเสียงออกไป (โดยใช้ Buffer ที่แปลงแล้ว)
        APRS_Send_Audio(encoded_buffer, encoded_bit_idx);
    }
    
    APP_EndTransmission();
	
	// 3. คืนค่าความถี่เดิม (สำคัญมาก!)
/*     APRS_SetFrequency(original_freq);
    SYSTEM_DelayMs(50); */
	
	// 3. คืนค่าเดิมกลับสู่ระบบ
    gTxVfo->OUTPUT_POWER = original_power;
    gTxVfo->pTX->Frequency = original_vfo_freq;
	
	// คำนวณและตั้งค่า Hardware กลับคืน
    RADIO_ConfigureSquelchAndOutputPower(gTxVfo);
    APRS_SetFrequency(original_freq);
    BK4819_SetupPowerAmplifier(gTxVfo->TXP_CalculatedSetting, gTxVfo->pTX->Frequency);
    
    SYSTEM_DelayMs(50);
	
    FUNCTION_Select(FUNCTION_FOREGROUND);
}

// ==========================================
// ฟังก์ชันส่งข้อความ APRS Message
// ==========================================
void APRS_Send_Message(char *target_call, char *message_text) {
    // 1. จำค่าเดิมและตั้งค่าวิทยุ (เหมือน Beacon)
    uint32_t original_freq = APRS_GetCurrentFrequency();
    uint8_t original_power = gTxVfo->OUTPUT_POWER;
    uint32_t original_vfo_freq = gTxVfo->pTX->Frequency;

    uint32_t aprs_freq_10hz = aprs_config.tx_freq / 10;

    UI_DisplayClear();
    UI_PrintStringSmallBold("TX MSG...", 0, 0, 3);
    char buf[32];
    sprintf(buf, "To: %s", target_call);
    UI_PrintStringSmallNormal(buf, 0, 0, 4);
    ST7565_BlitFullScreen();

    // ตั้งค่า Hardware
    gTxVfo->pTX->Frequency = aprs_freq_10hz;
    if (aprs_config.tx_power < 5) gTxVfo->OUTPUT_POWER = aprs_config.tx_power + 1;
    else if (aprs_config.tx_power == 5) gTxVfo->OUTPUT_POWER = 6;
    else gTxVfo->OUTPUT_POWER = 7;

    RADIO_ConfigureSquelchAndOutputPower(gTxVfo);
    APRS_SetFrequency(aprs_freq_10hz);
    BK4819_SetupPowerAmplifier(gTxVfo->TXP_CalculatedSetting, gTxVfo->pTX->Frequency);
    SYSTEM_DelayMs(50);
    RADIO_PrepareTX();
    SYSTEM_DelayMs(100);

    if (gCurrentFunction == FUNCTION_TRANSMIT) {
        uint8_t packet[200];
        int idx = 0;
        memset(packet, 0, sizeof(packet));

        // --- 1. AX.25 Header ---
        // Dest: APUVK1 (หรือ APRS software version) ไม่ใช่ปลายทางข้อความ
        format_callsign((char*)&packet[idx], "APUVK1", 0, false); idx += 7;
        
        // Source: MyCall
        bool has_path = (strlen(aprs_config.digipath) > 0);
        format_callsign((char*)&packet[idx], aprs_config.callsign, aprs_config.ssid, !has_path); idx += 7;

        // Path: WIDE1-1 etc.
        if (has_path) {
            char path_name[10];
            int path_ssid = 0;
            char *dash = strchr(aprs_config.digipath, '-');
            if (dash) {
                int len = dash - aprs_config.digipath;
                if(len > 6) len = 6;
                memcpy(path_name, aprs_config.digipath, len);
                path_name[len] = 0;
                path_ssid = custom_atoi(dash + 1);
            } else {
                strncpy(path_name, aprs_config.digipath, 6);
                path_name[6] = 0;
            }
            format_callsign((char*)&packet[idx], path_name, path_ssid, true); idx += 7;
        }

        packet[idx++] = 0x03; // Control
        packet[idx++] = 0xF0; // PID

        // --- 2. APRS Message Payload ---
        // Format: :BLNWCALL :Message Text{001
        // BLNWCALL ต้องยาว 9 ตัวอักษรเป๊ะ (Padded with spaces)
        
        char payload[150];
        char padded_dest[10];
        memset(padded_dest, ' ', 9); // เติม space รอไว้ 9 ตัว
        
        int dest_len = strlen(target_call);
        if(dest_len > 9) dest_len = 9;
        memcpy(padded_dest, target_call, dest_len); // ก๊อปชื่อทับลงไป
        padded_dest[9] = '\0'; // ปิดท้าย string

        // สร้าง Message ID (นับเพิ่มเรื่อยๆ)
        static int msg_seq = 1;
        
        // ประกอบร่าง Payload
        sprintf(payload, ":%s:%s{%03d", padded_dest, message_text, msg_seq);
        msg_seq++;
        if(msg_seq > 999) msg_seq = 1;

        int pay_len = strlen(payload);
        memcpy(&packet[idx], payload, pay_len);
        idx += pay_len;

        // --- 3. CRC & Send ---
        uint16_t fcs = calc_crc(&packet[0], idx);
        packet[idx++] = fcs & 0xFF;
        packet[idx++] = (fcs >> 8) & 0xFF;

        BK4819_DisableDTMF();
        APRS_EncodePacket(packet, idx);
        APRS_Send_Audio(encoded_buffer, encoded_bit_idx);
    }

    APP_EndTransmission();

    // คืนค่าเดิม
    gTxVfo->OUTPUT_POWER = original_power;
    gTxVfo->pTX->Frequency = original_vfo_freq;
    RADIO_ConfigureSquelchAndOutputPower(gTxVfo);
    APRS_SetFrequency(original_freq);
    BK4819_SetupPowerAmplifier(gTxVfo->TXP_CalculatedSetting, gTxVfo->pTX->Frequency);
    SYSTEM_DelayMs(50);
    FUNCTION_Select(FUNCTION_FOREGROUND);
}

/* 
void APP_RunAPRS(void) {
    UI_DisplayClear();
    
    // *** แก้ไข: อ่านความถี่จาก Hardware แทน gEeprom (ปลอดภัย 100%) ***
    uint32_t freq = APRS_GetCurrentFrequency();

    while (1) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("APRS MODEM", 0, 0, 0);
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);
        
        UI_PrintStringSmallNormal("To: APDR16", 0, 0, 2);
        UI_PrintStringSmallNormal("Fr: E25WOP-7", 0, 0, 3);

        // แสดงความถี่
        char freq_str[32];
        // ใช้การหารแบบบ้านๆ เพื่อเลี่ยง sprintf (ประหยัดเมม) หรือจะใช้ sprintf ก็ได้ถ้ามั่นใจว่า stack พอ
        // สมมติ freq = 14552500
        uint32_t mhz = freq / 100000;
        uint32_t khz = freq % 100000;
        sprintf(freq_str, "Freq: %lu.%05lu", mhz, khz);
        
        UI_PrintStringSmallNormal(freq_str, 0, 0, 5);
        
        //UI_PrintStringSmallBold("STATUS:", 0, 0, 6);
        //UI_PrintStringSmallNormal(status_msg, 45, 0, 6);
        
        UI_PrintStringSmallBold("[SIDE1] Send", 0, 0, 7); 
        
        ST7565_BlitFullScreen();

        KEY_Code_t key = KEYBOARD_Poll();

        if (key == KEY_EXIT) {
            return; 
        }
        else if (key == KEY_PTT || key == KEY_SIDE1 || key == KEY_SIDE2) {
            APRS_SendBeacon_Now();
            while(KEYBOARD_Poll() == key) SYSTEM_DelayMs(10); 
        }
        SYSTEM_DelayMs(20);
    }
} */
