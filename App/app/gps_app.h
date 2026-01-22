#ifndef APP_GPS_H
#define APP_GPS_H

#include <stdint.h>
#include <stdbool.h>

/* // 1. โครงสร้างข้อมูลสำหรับ Smart Beacon
typedef struct {
    int stationary_rate_s; // Stationary Interval (วินาที)
    int slow_rate_s;       // Slow Rate (วินาที)
    int fast_rate_s;       // Fast Rate (วินาที)
    int low_speed_kmh;     // Slow Speed (km/h)
    int high_speed_kmh;    // Fast Speed (km/h)
    int turn_angle;        // Turn Angle (องศา)
    int turn_time_s;       // Turn Time (วินาที) - เวลาขั้นต่ำระหว่างการเลี้ยว
    int min_tx_dist_m;     // Distance (เมตร) - ส่งเมื่อเคลื่อนที่เกินระยะนี้
} SmartBeacon_t; */

// 1. ปรับ SmartBeacon ใช้ตัวแปรขนาดเล็ก (ประหยัดพื้นที่ EEPROM)
typedef struct {
    uint16_t stationary_rate_s; // ใช้ uint16 เก็บค่าได้ถึง 65,535 (พอสำหรับวินาที)
	float  stationary_speed_kmh; // <--- [เพิ่มบรรทัดนี้] ความเร็วที่ถือว่าจอดนิ่ง (km/h)
    uint16_t slow_rate_s;
    uint16_t fast_rate_s;
    uint8_t  low_speed_kmh;     // ใช้ uint8 เก็บได้ถึง 255 km/h (รถคงไม่วิ่งเกินนี้)
    uint8_t  high_speed_kmh;
    uint8_t  turn_angle;        // 0-180 องศา ใช้ uint8 พอ
    uint8_t  turn_time_s;       // เวลาเลี้ยวคงไม่เกิน 255 วิ
    uint16_t min_tx_dist_m;     // ระยะทางเป็นเมตร (อาจเป็นพันเมตร) ใช้ uint16
} SmartBeacon_t;

/* // 2. โครงสร้างข้อมูลหลักสำหรับการตั้งค่า APRS
typedef struct {
    bool aprs_on;           // เปิด/ปิด
    char callsign[10];      // E25WOP
    int  ssid;              // 7
    char dest_call[10];     // APDR16
    char digipath[20];      // WIDE1-1
    char icon_table;        // /
    char icon_symbol;       // [
    char comment[40];       // Comment
    
    uint32_t tx_freq;       // 144.390
    uint8_t tx_power;       // 1=Mid
    
    bool use_gps;           // ใช้ GPS หรือ Fixed
    char fixed_lat[12];     
    char fixed_lon[12];     

    bool smart_beacon;      // เปิด Smart Beacon
    int  manual_interval;   // วินาที
	int preamble;
	
	int bl_time;            // *** เพิ่มตัวแปรนี้: เวลาดับไฟ (วินาที), 0=เปิดตลอด ***
	
    SmartBeacon_t sb_conf;  // ค่าตั้ง Smart Beacon
} APRS_Config_t; */

// 2. ปรับ Config หลัก
typedef struct {
    // Flag รวม (Bitfield) เพื่อประหยัดที่
    // เรียง bool ติดกัน Compiler จะพยายามบีบให้เองระดับนึง
    bool aprs_on;           
    bool use_gps;           
    bool smart_beacon;      
    
    char callsign[7];       // Callsign ยาวสุด 6 ตัว + Null = 7 (E25WOP)
    uint8_t  ssid;          // SSID 0-15 ใช้ uint8 เหลือเฟือ
    
    char dest_call[7];      // APUVK1 (6+1)
    char digipath[20];      // WIDE1-1,WIDE2-1 (ลดจาก 20)
    char icon_table;        
    char icon_symbol;       
    char comment[40];       // ลดจาก 40 เหลือ 25 (ประหยัดได้ 15 bytes)
    
    uint32_t tx_freq;       
    uint8_t tx_power;       
    
    // Fixed Location ไม่ต้องบันทึก string ยาวๆ 
    // เราบันทึกเป็น float หรือ double จะประหยัดกว่า แต่เพื่อความง่ายคง string ไว้ก่อนก็ได้
    char fixed_lat[11];     // "13.74147" (8-10 ตัว) ลดจาก 12
    char fixed_lon[11];     

    uint16_t manual_interval; // 60 วินาที ใช้ uint16
    uint16_t preamble;        // uint16
    
    uint8_t bl_time;          // uint8
	
	int timer_divider;
    
    SmartBeacon_t sb_conf;  // โครงสร้างย่อย
} APRS_Config_t;

// 3. ประกาศตัวแปร Global ให้ไฟล์อื่น (app_aprs.c) มองเห็น
extern APRS_Config_t aprs_config;

extern char val_lat[16];
extern char val_lon[16];

extern char val_alt[10];
extern char val_speed[10];
extern char val_course[10];

// 4. ฟังก์ชันหลัก
void APP_RunGPS(void);

// 5. ประกาศฟังก์ชันส่ง (เพื่อให้ gps_app.c เรียกใช้ได้)
void APRS_SendBeacon_Now(void); 

void APRS_Send_Message(char *target_call, char *message_text);

#endif /* APP_GPS_H */
