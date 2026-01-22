#include "py32f0xx.h"
#include "gps_app.h"
#include <string.h>
#include <stdio.h>
// #include <math.h> // ไม่ใช้ math.h เพื่อประหยัดเมมโมรี่
#include "ui/helper.h"
#include "driver/st7565.h"
#include "driver/keyboard.h"
#include "driver/system.h"
#include "driver/systick.h"
#include "driver/uart.h"
#include "driver/gpio.h" 
#include "functions.h"

//#include "radio.h"
//#include "functions.h"
// ==========================================
// 1. ส่วนประกาศตัวแปรและโครงสร้างข้อมูล
// ==========================================

#ifndef UART1
#define UART1 USART1
#endif

// ค่าคงที่ทางคณิตศาสตร์
#define PI 3.1415927f
#define HALF_PI 1.5707963f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define R_EARTH 6371.0f 

// โครงสร้างเก็บข้อมูล APRS ที่รับได้ (เพื่อนำไปโชว์)
typedef struct {
    char source_call[10];  // ชื่อผู้ส่ง
    char dest_call[10];    // ชื่อผู้รับ/อุปกรณ์
    char info[60];         // ข้อความ หรือ พิกัดดิบ
    char lat_str[10];      // พิกัดที่แกะแล้ว (ถ้ามี)
    char lon_str[11];
    char symbol_table;     // ไอคอน
    char symbol_code;
    bool valid;            // ข้อมูลถูกต้องไหม
} APRS_RX_Packet_t;

// 2. ประกาศตัวแปร Global (บรรทัดนี้แหละที่หายไปครับ)
APRS_RX_Packet_t last_rx_packet;

// ==========================================
// ส่วน AFSK MODEM (หูฟังเสียง)
// ==========================================

// สถานะการถอดรหัส
typedef struct {
    bool receiving;       // กำลังรับหรือไม่
    uint32_t sample_counter; 
    int16_t last_sample;  // ค่าเสียงครั้งที่แล้ว
    int32_t zero_crossing_period; // ระยะห่างระหว่างจุดตัดศูนย์
    
    // NRZI & HDLC decoding
    uint8_t current_byte;
    uint8_t bit_count;
    uint8_t ones_count;   // นับเลข 1 เรียงกัน (Bit stuffing)
    bool packet_started;
    
    // Buffer สำหรับเก็บข้อมูลที่แกะได้ (Bits -> Bytes)
    uint8_t buffer[200];
    int buffer_len;
	int8_t current_level; // เพิ่ม: จำสถานะปัจจุบัน (1=HIGH, -1=LOW)
} AFSK_Decoder_t;

APRS_RX_Packet_t current_packet; // ตัวแปรเก็บผลลัพธ์
AFSK_Decoder_t rx_decoder;       // ตัวแปรเก็บสถานะ Modem

// ค่าคงที่สำหรับการแยกเสียง (จูนตาม Sampling Rate)
// สมมติเราจะอ่าน ADC ที่ความเร็วประมาณ 9600-13200 Hz
// ค่ากลางๆ สำหรับแยก 1200Hz vs 2200Hz
#define THRESHOLD_1200_2200  8  // ต้องจูนทีหลัง

// กำหนดค่าเริ่มต้น (Default Settings)
APRS_Config_t aprs_config = {
    .aprs_on = false,
    .callsign = "E25WOP",
    .ssid = 7,
    .dest_call = "APUVK1",
    .digipath = "WIDE1-1",
    .icon_table = '/',
    .icon_symbol = '>',
    .comment = "APRS from QuanSheng UV-K1",
    .tx_freq = 144390000,
    .tx_power = 5,      // MID
    .use_gps = true,
    //.fixed_lat = "1344.48N", // Format APRS: DDMM.mmN
    //.fixed_lon = "10030.00E",
	//.fixed_lat = "13.74147", 
    //.fixed_lon = "100.48924",
	.fixed_lat = "", 
    .fixed_lon = "",
    .smart_beacon = false,
    .manual_interval = 60, // 60 วินาที
	// *** เพิ่มบรรทัดนี้ (ค่าเดิมคือ 150 flags = 1 วินาที) ***
    .preamble = 170,
	.bl_time = 10, // *** เพิ่มค่าเริ่มต้น: 0 (เปิดตลอด) หรือจะใส่ 10, 20 ก็ได้ ***
	.timer_divider = 28, // <--- เพิ่มค่าเริ่มต้น (28 รอบ = 1 วินาที)
    
    // Smart Beacon Defaults
    //.sb_conf = { 30, 600, 5, 80, 28 } 
	// ตั้งค่าเริ่มต้น Smart Beacon
    .sb_conf = {
        .stationary_rate_s = 300, // 30 นาที จอดนิ่ง
		.stationary_speed_kmh = 0.5f, // <--- [เพิ่มบรรทัดนี้] ค่า Default 3 km/h
        .slow_rate_s = 120,        // 5 นาที ความเร็วต่ำ
        .fast_rate_s = 60,         // 1 นาที ความเร็วสูง
        .low_speed_kmh = 15,        // ต่ำกว่า 5 km/h ถือว่าช้า
        .high_speed_kmh = 30,      // สูงกว่า 60 km/h ถือว่าเร็ว
        .turn_angle = 28,          // เลี้ยว 28 องศา
        .turn_time_s = 30,         // ส่งซ้ำเมื่อเลี้ยวไม่เกินทุก 30 วิ
        .min_tx_dist_m = 2000         // ระยะทางขั้นต่ำ (0=ปิด)
    }
};

// ตัวแปรนับเวลาไฟจอ (Global)
static uint32_t bl_timer_count = 0;

// ตัวแปรสำหรับ Smart Beacon Logic
static uint32_t sb_timer_tick = 0;      // ตัวนับเวลาหลัก (Monotonic)
static uint32_t last_tx_tick = 0;       // เวลาที่ส่งล่าสุด
static int32_t last_tx_gps_sec = 0; // จำเวลา GPS ที่ส่งครั้งล่าสุด
static float last_tx_lat = 0.0f;
static float last_tx_lon = 0.0f;
static int last_tx_course = 0;

static bool was_moving = false; // จำว่ารอบที่แล้วรถวิ่งอยู่ไหม

// เพิ่มตัวแปร Global หรือ Static ไว้ข้างนอก APRS_Task (ส่วนหัวไฟล์)
static bool first_beacon_sent = false;

// ตัวแปรสำหรับจับเวลาส่งอัตโนมัติ
//static uint32_t last_beacon_time = 0;

// ประกาศ Prototype ฟังก์ชันที่อยู่ด้านล่าง (เพื่อแก้ Error implicit declaration)
bool EditString(char *title, char *buffer, int max_len, bool numeric_only);
void SubApp_APRS_Settings(void);
//void SubApp_GridCalc(void);
void SubApp_SmartBeacon_Settings(void);
void SubApp_GPS_Info(void);
bool GPS_Fetch_Data(bool silent, int *page);

// ==========================================
// 2. ส่วนของ GPS Logic ตัวแปร
// ==========================================

//#define RX_BUF_SIZE 180
//#define RX_BUF_SIZE 120 
#define RX_BUF_SIZE 100 
static char rx_buffer[RX_BUF_SIZE] = ""; 
char val_lat[16]   = "13.74147";
char val_lon[16]   = "100.48924";

//extern char val_alt[10];
//extern char val_speed[10];
//extern char val_course[10];

extern void BACKLIGHT_TurnOn(void);
extern void BACKLIGHT_TurnOff(void);
extern uint16_t BK4819_GetRSSI(void);
extern uint16_t BK4819_ReadRegister(uint8_t Reg);

char val_alt[10]   = "0.0";
char val_sats[5]   = "0";
char val_speed[10] = "0";
char val_course[10]= "0";
static char val_time[10]  = "-";
static char val_date[16]  = "-"; 
static char val_grid[7] = "------";
static char val_lat_dst[16] = " 13.74147"; 
static char val_lon_dst[16] = " 100.48924";
static bool is_editing_nav = false;
static int nav_cursor = 0;

//static uint32_t aprs_timer_counter = 0;

/* bool Is_RX_Active(void) {
    // 1. อ่านค่าความแรงสัญญาณจากชิป BK4819 โดยตรง
    // (ค่าปกติ: ไม่มีสัญญาณ ~20-30, มีสัญญาณบางๆ ~50, สัญญาณเต็ม ~150-300)
    uint16_t rssi = BK4819_GetRSSI();
    
    // 2. เช็คว่าแรงกว่าเกณฑ์ที่กำหนดไหม (Threshold)
    // ตั้งไว้ที่ 50 ถือว่าปลอดภัยสำหรับทดสอบ (Squelch ระดับกลางๆ)
    // ถ้าอยากให้แม่นยำต้องไปดึงค่า Squelch Level จาก EEPROM มาคำนวณ แต่เอาแค่นี้ก่อนครับ
    if (rssi > 200) {
        return true; // มีสัญญาณเข้า
    }
    
    return false; // เงียบ
} */


// *** ฟังก์ชันจัดการไฟหน้าจอ (เอาไว้เรียกใน Loop ต่างๆ) ***
void HandleBacklight(KEY_Code_t key) {
    // ถ้าตั้งเป็น 0 = เปิดตลอด
    if (aprs_config.bl_time <= 0) {
        BACKLIGHT_TurnOn();
        return;
    }

    if (key != KEY_INVALID) {
        // ถ้ามีการกดปุ่ม ให้รีเซ็ตเวลาและเปิดไฟ
        bl_timer_count = 0;
        BACKLIGHT_TurnOn();
    } else {
        // ถ้าไม่มีการกดปุ่ม ให้นับเวลาเพิ่ม
        bl_timer_count++;
        
        // 1 วินาที loop วิ่งประมาณ 50 รอบ (เพราะเรา delay 20ms)
        uint32_t limit = aprs_config.bl_time * 50;
        
        if (bl_timer_count > limit) {
            BACKLIGHT_TurnOff();
        }
    }
}

// ==========================================
// 2. Custom Math Functions (Clean Format)
// ==========================================

// แปลง String เป็น Float (รองรับช่องว่างนำหน้า)
float my_atof(const char* s) {
    float rez = 0, fact = 1;
    while (*s == ' ') s++; // ข้ามช่องว่าง
    if (*s == '-') { s++; fact = -1; }
    
    int point_seen = 0;
    float d_point = 10.0f;
    for (; *s; s++) {
        if (*s == '.') { point_seen = 1; continue; }
        int d = *s - '0';
        if (d >= 0 && d <= 9) {
            if (point_seen) { 
                rez += (float)d / d_point;
                d_point *= 10.0f;
            } else {
                rez = rez * 10.0f + (float)d;
            }
        }
    }
    return rez * fact;
}

float my_abs(float x) { return (x < 0) ? -x : x; }

// Square Root
float my_sqrt(float n) {
    if (n <= 0) return 0;
    float x = n;
    if (x < 1.0f) x = 1.0f; 
    for (int i = 0; i < 20; i++) {
        x = 0.5f * (x + n / x);
    }
    return x;
}

// Cosine
float my_cos(float x) {
    while (x > PI) { x -= 2 * PI; }
    while (x < -PI) { x += 2 * PI; }
    float x2 = x * x;
    return 1.0f - (x2 / 2.0f) + (x2 * x2 / 24.0f) - (x2 * x2 * x2 / 720.0f);
}

// Sine
float my_sin(float x) {
    while (x > PI) { x -= 2 * PI; }
    while (x < -PI) { x += 2 * PI; }
    float x2 = x * x;
    float x3 = x * x2;
    float x5 = x3 * x2;
    float x7 = x5 * x2;
    return x - (x3 / 6.0f) + (x5 / 120.0f) - (x7 / 5040.0f);
}

// Arctan2
float my_atan2(float y, float x) {
    if (x == 0.0f) return (y > 0.0f) ? HALF_PI : ((y == 0.0f) ? 0.0f : -HALF_PI);
    float z = y / x;
    float az = my_abs(z);
    float a;
    if (az < 1.0f) a = z / (1.0f + 0.28f * z * z);
    else {
        float iz = 1.0f / z;
        a = (z > 0 ? HALF_PI : -HALF_PI) - (iz / (1.0f + 0.28f * iz * iz));
    }
    if (x < 0.0f && y >= 0.0f) a += PI;
    else if (x < 0.0f && y < 0.0f) a -= PI;
    return a;
}

// Haversine Formula
void CalculateDistanceBearing(float lat1, float lon1, float lat2, float lon2, float *dist, float *bearing) {
    float rLat1 = lat1 * DEG2RAD;
    float rLon1 = lon1 * DEG2RAD;
    float rLat2 = lat2 * DEG2RAD;
    float rLon2 = lon2 * DEG2RAD;
    
    float dLat = rLat2 - rLat1;
    float dLon = rLon2 - rLon1;

    float a = (my_sin(dLat / 2.0f) * my_sin(dLat / 2.0f)) +
              (my_cos(rLat1) * my_cos(rLat2) * my_sin(dLon / 2.0f) * my_sin(dLon / 2.0f));
    
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;
    
    float c = 2.0f * my_atan2(my_sqrt(a), my_sqrt(1.0f - a));
    *dist = R_EARTH * c;

    float y = my_sin(dLon) * my_cos(rLat2);
    float x = my_cos(rLat1) * my_sin(rLat2) - my_sin(rLat1) * my_cos(rLat2) * my_cos(dLon);
    *bearing = my_atan2(y, x) * RAD2DEG;
    if (*bearing < 0) *bearing += 360.0f;
}

// ==========================================
// 3. Helper Logic
// ==========================================

void CalculateGrid(void) {
    float lat = my_atof(val_lat);
    float lon = my_atof(val_lon);
    lon += 180; lat += 90;
    val_grid[0] = 'A' + (int)(lon / 20); val_grid[1] = 'A' + (int)(lat / 10);
    lon -= (int)(lon / 20) * 20; lat -= (int)(lat / 10) * 10;
    val_grid[2] = '0' + (int)(lon / 2); val_grid[3] = '0' + (int)(lat);
    lon -= (int)(lon / 2) * 2; lat -= (int)(lat);
    val_grid[4] = 'a' + (int)(lon * 12); val_grid[5] = 'a' + (int)(lat * 24); val_grid[6] = '\0';
}

void GridToLatLon(char *grid, float *lat, float *lon) {
    int len = strlen(grid);
    *lon = (grid[0] - 'A') * 20.0f - 180.0f;
    *lat = (grid[1] - 'A') * 10.0f - 90.0f;
    
    if (len >= 4) {
        *lon += (grid[2] - '0') * 2.0f;
        *lat += (grid[3] - '0') * 1.0f;
    }
    
    if (len >= 6) {
        float sub_lon = (float)(grid[4] - 'a');
        float sub_lat = (float)(grid[5] - 'a');
        *lon += sub_lon * (2.0f / 24.0f);
        *lat += sub_lat * (1.0f / 24.0f);
    }
    
    if (len == 2) { *lon += 10.0f; *lat += 5.0f; }
    else if (len == 4) { *lon += 1.0f; *lat += 0.5f; }
    else if (len == 6) { *lon += (1.0f / 24.0f); *lat += (0.5f / 24.0f); }
}

void WaitForRelease() { 
    while (KEYBOARD_Poll() != KEY_INVALID) {} 
    volatile uint32_t d = 50000; while (d--) {} 
}

void SetupUART() {
    UART1->CR1 = 0; 
    RCC->APBENR2 |= RCC_APBENR2_USART1EN; RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1); 
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12)); GPIOA->AFR[0] |= ((0x1 << 8) | (0x1 << 12)); 
    // 9600 = 0x1388 , 19200 = 0x9C4 , 38400 = 0x04E2
	UART1->BRR = 0x04E2; UART1->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
}

/* void UART_SendByte(uint8_t data) {
    while (!(UART1->SR & USART_SR_TXE)) {}
    UART1->DR = data;
    while (!(UART1->SR & USART_SR_TC)) {}
} */

// ตัวอย่างการแก้ให้ปลอดภัยขึ้น
void UART_SendByte(uint8_t data) {
    uint32_t timeout = 15000;
    while (!(UART1->SR & USART_SR_TXE) && timeout--) {} // รอแบบมี Timeout
    if (timeout == 0) return; // ถ้าหมดเวลาให้ข้ามไปเลย กันค้าง
    
    UART1->DR = data;
    
    timeout = 15000;
    while (!(UART1->SR & USART_SR_TC) && timeout--) {}
}

void DrawProgressBar(int percent) {
    if (percent < 0) percent = 0;
	if (percent > 100) percent = 100;
    int x_end = ((percent * 128) / 100) + 65;
    if (x_end > 0) UI_DrawRectangleBuffer(gFrameBuffer, 65, 54, x_end, 55, true);
}

// ==========================================
// 3. UI Helper Function (EditString)
// ==========================================
// ย้ายมาไว้ตรงนี้ก่อนถูกเรียกใช้
/* bool EditString(char *title, char *buffer, int max_len, bool numeric_only) {
    int cursor = 0; int len = strlen(buffer); WaitForRelease(); 
    while (1) {
        UI_DisplayClear(); UI_PrintStringSmallBold(title, 0, 0, 0);
        char disp_buff[20]; sprintf(disp_buff, "%s_", buffer); UI_PrintStringSmallBold(disp_buff, 10, 0, 3);
        UI_DrawRectangleBuffer(gFrameBuffer, 10 + (cursor * 7), 35, 10 + (cursor * 7) + 5, 36, true);
        UI_PrintStringSmallBold("UP/DN:Char MENU:Next", 0, 0, 6); ST7565_BlitFullScreen();
        KEY_Code_t key = KEYBOARD_Poll();
        if (key == KEY_EXIT) { WaitForRelease(); return false; }
        if (key == KEY_MENU) { cursor++; if (cursor >= max_len || cursor > len) { WaitForRelease(); return true; } WaitForRelease(); }
        if (key == KEY_DOWN) { char c = buffer[cursor]; if(c==0)c=' '; if(c==' ')c='A'; else if(c=='Z')c='0'; else if(c=='9')c=numeric_only?'0':'-'; else if(c=='-')c=numeric_only?'0':' '; else c++; buffer[cursor]=c; if(cursor==len){len++;buffer[len]=0;} volatile uint32_t d=600000; while (d--) {} }
        if (key == KEY_UP) { char c = buffer[cursor]; if(c==0)c=' '; if(c==' ')c=numeric_only?'9':'-'; else if(c=='-')c='9'; else if(c=='0')c='Z'; else if(c=='A')c=' '; else c--; buffer[cursor]=c; if(cursor==len){len++;buffer[len]=0;} volatile uint32_t d=600000; while (d--) {} }
    }
} */

// ==========================================
// 3. UI Helper Function (EditString) - Final Fixed (Safe Line 0-6)
// ==========================================
bool EditString(char *title, char *buffer, int max_len, bool numeric_only) {
    int cursor = 0; 
    int len = strlen(buffer); 
    
    // ตัวแปรจำว่ากด MENU ไปแล้วหรือยัง
    bool menu_pressed_once = false;
	
	// ตัวแปรเก็บสถานะภาษา (false=ENG, true=THAI)
    bool is_thai_mode = false;
    
    // ตั้งค่า cursor ไปที่ท้ายข้อความเสมอเมื่อเริ่ม
    cursor = len;

    WaitForRelease(); 
    
    while (1) {
        UI_DisplayClear(); 
		
		// แสดง Title พร้อมบอกโหมดภาษาปัจจุบัน
        char title_buff[32];
        sprintf(title_buff, "%s [%s]", title, is_thai_mode ? "TH" : "EN");
        UI_PrintStringSmallBold(title_buff, 0, 0, 0);
		
		
        UI_PrintStringSmallBold(title, 0, 0, 0); // Line 0
        
        // ขยาย Buffer เป็น 64 กันเหนียว
        char disp_buff[64]; 
        sprintf(disp_buff, "%s_", buffer); 
        UI_PrintStringSmallBold(disp_buff, 10, 0, 3); // Line 3
        
        // วาด Cursor (Y=35 คือช่วง Line 4)
        int cursor_x = 10 + (cursor * 7);
        if (cursor_x < 120) {
            UI_DrawRectangleBuffer(gFrameBuffer, cursor_x, 35, cursor_x + 5, 36, true);
        }
        
        // [จุดที่แก้] ย้ายคำแนะนำมาอยู่บรรทัด 5 และ 6 (ไม่ใช้ 7 แล้ว)
/*         if (menu_pressed_once) {
            UI_PrintStringSmallBold("MENU AGAIN TO SAVE", 20, 0, 6);
        } else {
            // ย้ายขึ้นมาบรรทัด 5
            UI_PrintStringSmallBold("UP/DN:Char *:Del", 0, 0, 5);
            // อยู่บรรทัด 6 (Max สุดที่ปลอดภัย)
            UI_PrintStringSmallBold("MENU:Next  EXIT:Esc", 0, 0, 6);
        } */
		
		// แสดงคำแนะนำปุ่มกด
        if (menu_pressed_once) {
            UI_PrintStringSmallBold("MENU AGAIN TO SAVE", 0, 0, 6);
        } else {
            // บรรทัด 5: บอกว่ากด # เพื่อเปลี่ยนภาษา
            if (numeric_only) 
                UI_PrintStringSmallBold("UP/DN:Num *:Del", 0, 0, 5);
            else 
                UI_PrintStringSmallBold("#:Lang *:Del", 0, 0, 5);
                
            UI_PrintStringSmallBold("MENU:Next EXIT:Esc", 0, 0, 6);
        }
        
        ST7565_BlitFullScreen();
        
        KEY_Code_t key = KEYBOARD_Poll();
        HandleBacklight(key); 
        
        if (key == KEY_EXIT) { WaitForRelease(); return false; }
        
        // --- ปุ่ม * (Backspace) ---
        if (key == KEY_STAR) {
            menu_pressed_once = false; 
            if (cursor > 0) {
                cursor--;           
                buffer[cursor] = 0; 
                len = cursor;       
                SYSTEM_DelayMs(150); 
            }
        }
		
		// --- ปุ่ม # (เปลี่ยนภาษา) ---
        // ปกติปุ่ม # บนเครื่อง UV-K5 คือ KEY_F
        if (!numeric_only && key == KEY_F) {
            is_thai_mode = !is_thai_mode; // สลับโหมด
            
            // เปลี่ยนตัวอักษรปัจจุบันให้เข้ากับโหมดใหม่ทันที
            unsigned char c = (unsigned char)buffer[cursor];
            
            if (is_thai_mode) {
                // ถ้าสลับมาไทย แล้วตัวเดิมไม่ใช่อักษรไทย -> เปลี่ยนเป็น ก ไก่ (0xA1)
                if (c < 0xA1 && c != ' ') buffer[cursor] = (char)0xA1;
            } else {
                // ถ้าสลับมาอังกฤษ แล้วตัวเดิมเป็นอักษรไทย -> เปลี่ยนเป็น A
                if (c >= 0xA1) buffer[cursor] = 'A';
            }
            
            SYSTEM_DelayMs(200);
        }

        // --- ปุ่ม MENU (Next / Save) ---
        if (key == KEY_MENU) { 
            // กด MENU เบิ้ล -> Save
            if (menu_pressed_once) {
                return true; 
            }

            if (cursor < max_len) {
                cursor++;
                if (cursor > len) { 
                    len = cursor; 
                    buffer[len] = 0; 
                }
                menu_pressed_once = true; // จำไว้ว่ากดแล้ว 1 ที
            } else {
                WaitForRelease(); 
                return true; 
            }
            
            SYSTEM_DelayMs(250); 
        }
        
        // --- ปุ่มเปลี่ยนตัวอักษร ---
        if (key == KEY_DOWN || key == KEY_UP) {
            menu_pressed_once = false; 
            
            
			// แปลงเป็น unsigned char เพื่อให้เปรียบเทียบค่าเกิน 127 (ภาษาไทย) ได้ถูกต้อง
            unsigned char c = (unsigned char)buffer[cursor];
			//char c = buffer[cursor];
            if (c == 0) c = ' '; 
            
            if (numeric_only) {
                if (key == KEY_DOWN) {
                    if (c == ' ') c = '-';
                    else if (c == '-') c = '.';
                    else if (c == '.') c = '0';
                    else if (c >= '0' && c < '9') c++;
                    else if (c == '9') c = ' ';
                    else c = ' '; 
                } else { 
                    if (c == ' ') c = '9';
                    else if (c > '0' && c <= '9') c--;
                    else if (c == '0') c = '.';
                    else if (c == '.') c = '-';
                    else if (c == '-') c = ' ';
                    else c = ' ';
                }
            } else {
				// === แก้ไขส่วน Text Mode ให้รองรับภาษาไทย ===
                
                // ลำดับการวน: Space -> A-Z -> 0-9 -> Symbols -> Thai -> Space
/*                 if (key == KEY_DOWN) {
                    if (c == ' ') c = 'A';
                    else if (c >= 'A' && c < 'Z') c++;
                    else if (c == 'Z') c = '0';
                    else if (c >= '0' && c < '9') c++;
                    else if (c == '9') c = '-';
                    else if (c == '-') c = '.';
                    else if (c == '.') c = '/';
                    else if (c == '/') c = ':';
                    //else if (c == ':') c = ' ';
					//else if (c == ':') c = 0xA1;    // *** กระโดดเข้าภาษาไทย (ก ไก่) ***
                   // else if (c >= 0xA1 && c < 0xF9) c++; // วนภาษาไทย (0xF9 คือตัวสุดท้ายใน font.c)
                   // else if (c >= 0xF9) c = ' ';    // จบภาษาไทย วนกลับมา Space
                    //else c = ' ';
					else if (c == ':') c = 0xA1;    // เข้าภาษาไทย (ก ไก่)
                    else if (c >= 0xA1 && c < 0xF9) { // วนภาษาไทย (0xF9 คือตัวสุดท้ายใน font.c)
                        c++;
                        // ** กระโดดข้ามช่องว่าง **
                        if (c == 0xDA) c = 0xDF; // ข้าม 57-61 ไปหา 62(฿)
                        if (c == 0xED) c = 0xF0; // ข้าม 76-78 ไปหา 79(๐)
                    }
                    else if (c >= 0xF9) c = ' ';    // จบไทยวนไป Space
                    else c = ' ';
                } else { 
                    //if (c == ' ') c = ':';
					//if (c == ' ') c = 0xF9;         // *** Space ถอยหลังไปตัวสุดท้ายของไทย ***
                    //else if (c > 0xA1 && c <= 0xF9) c--; // ถอยหลังในภาษาไทย
					if (c == ' ') c = 0xF9;         // ถอยจาก Space มาตัวสุดท้ายของไทย (๙)
                    else if (c > 0xA1 && c <= 0xF9) {
                        c--;
                        // ** กระโดดข้ามช่องว่าง (ถอยหลัง) **
                        if (c == 0xDE) c = 0xD9; // ถอยจาก 62(฿) ข้ามกลับไป 56(ู)
                        if (c == 0xEF) c = 0xEC; // ถอยจาก 79(๐) ข้ามกลับไป 75(์)
                    }
                    else if (c == 0xA1) c = ':';    // ถอยจาก ก ไก่ ไปสัญลักษณ์
                    else if (c == ':') c = '/';
                    else if (c == '/') c = '.';
                    else if (c == '.') c = '-';
                    else if (c == '-') c = '9';
                    else if (c > '0' && c <= '9') c--;
                    else if (c == '0') c = 'Z';
                    else if (c > 'A' && c <= 'Z') c--;
                    else if (c == 'A') c = ' ';
                    else c = ' ';
                } */

			
				// *** โหมดข้อความ (แยกตามภาษา) ***
                
                if (is_thai_mode) {
                    // === ลูปภาษาไทย (Space <-> ก-๙) ===
                    if (key == KEY_DOWN) {
                        if (c == ' ') c = 0xA1; // Space -> ก
                        else if (c >= 0xA1 && c < 0xF9) {
                            c++;
                            if (c == 0xDA) c = 0xDF; // ข้ามว่าง 1
                            if (c == 0xED) c = 0xF0; // ข้ามว่าง 2
                        }
                        else c = ' '; // จบไทย -> Space
                    } else { // KEY_UP
                        if (c == ' ') c = 0xF9; // Space -> ๙
                        else if (c > 0xA1 && c <= 0xF9) {
                            c--;
                            if (c == 0xDE) c = 0xD9; // ข้ามว่าง 1
                            if (c == 0xEF) c = 0xEC; // ข้ามว่าง 2
                        }
                        else c = ' '; // จบไทย -> Space
                    }
                } else {
                    // === ลูปภาษาอังกฤษ (Space <-> A-Z <-> 0-9 <-> Sym) ===
                    if (key == KEY_DOWN) {
                        if (c == ' ') c = 'A';
                        else if (c >= 'A' && c < 'Z') c++;
                        else if (c == 'Z') c = '0';
                        else if (c >= '0' && c < '9') c++;
                        else if (c == '9') c = '-';
                        else if (c == '-') c = '.';
                        else if (c == '.') c = '/';
                        else if (c == '/') c = ':';
                        else if (c == ':') c = ' '; // สัญลักษณ์สุดท้าย -> Space (ไม่เข้าไทย)
                        else c = ' '; // กันหลง
                    } else { // KEY_UP
                        if (c == ' ') c = ':'; // Space -> สัญลักษณ์สุดท้าย
                        else if (c == ':') c = '/';
                        else if (c == '/') c = '.';
                        else if (c == '.') c = '-';
                        else if (c == '-') c = '9';
                        else if (c > '0' && c <= '9') c--;
                        else if (c == '0') c = 'Z';
                        else if (c > 'A' && c <= 'Z') c--;
                        else if (c == 'A') c = ' ';
                        else c = ' ';
                    }
                }
            }
			
            //buffer[cursor] = c;
			buffer[cursor] = (char)c; // Cast กลับเป็น char เพื่อเก็บลง buffer
            if (cursor == len) { len++; buffer[len] = 0; }
            SYSTEM_DelayMs(150); 
        }
        
        SYSTEM_DelayMs(10);
    }
}

// แปลงเวลาเป็นวินาที (รองรับทั้งแบบ HHMMSS และ HH:MM:SS)
int32_t Get_GPS_Seconds(void) {
    if (val_time[0] == '-' || strlen(val_time) < 6) return -1;
    
    int h, m, s;

    // เช็คว่าตัวอักษรตำแหน่งที่ 2 เป็น ':' หรือไม่ (เช่น 12:30:45)
    if (val_time[2] == ':') {
        // รูปแบบ HH:MM:SS
        h = (val_time[0] - '0') * 10 + (val_time[1] - '0');
        m = (val_time[3] - '0') * 10 + (val_time[4] - '0'); // ข้าม : ตัวแรก
        s = (val_time[6] - '0') * 10 + (val_time[7] - '0'); // ข้าม : ตัวที่สอง
    } else {
        // รูปแบบเดิม HHMMSS (มาตรฐาน NMEA)
        h = (val_time[0] - '0') * 10 + (val_time[1] - '0');
        m = (val_time[2] - '0') * 10 + (val_time[3] - '0');
        s = (val_time[4] - '0') * 10 + (val_time[5] - '0');
    }
    
    return (h * 3600) + (m * 60) + s;
}

// ==========================================
// 4. APRS Menus
// ==========================================
void APRS_Task(void) {
    if (!aprs_config.aprs_on) return;
	
	// Smart Beacon ต้องใช้ GPS เท่านั้น
    if (aprs_config.smart_beacon && !aprs_config.use_gps) return;
	
	// ถ้าตั้ง Interval เป็น 0 ถือว่าปิด Auto Send
    //if (aprs_config.manual_interval <= 0) return;
	
	// เพิ่ม Tick (เรียกทุก 20ms โดยประมาณ)
    // 1 วินาที = 50 รอบ (Loop หลัก delay 20ms)
    sb_timer_tick++;

	// -------------------------------------------------------------
    // NEW: คำนวณเวลาที่ผ่านไป (Elapsed Time) แบบ Hybrid
    // -------------------------------------------------------------
	// *** ใช้ตัวหารจาก Config (กันค่าเป็น 0 ด้วย) ***
    uint32_t div = aprs_config.timer_divider;
    if (div < 1) div = 28; // ค่ากันตาย
	
	// แปลง Tick เป็นวินาที (โดยประมาณ)
	// คำนวณวินาทีจากตัวหารที่ตั้งไว้
/*     uint32_t current_time_s = sb_timer_tick / div; 
    uint32_t last_tx_time_s = last_tx_tick / div;
    uint32_t time_diff_s = current_time_s - last_tx_time_s; */
	
	int32_t current_gps_sec = Get_GPS_Seconds();
    uint32_t time_diff_s = 0;
    //bool using_gps_time = false;
	
	// เช็คว่าเวลา GPS ใช้ได้ไหม และ User เปิดใช้ GPS หรือเปล่า
    if (aprs_config.use_gps && current_gps_sec >= 0) {
        //using_gps_time = true;
        
        // คำนวณ Delta (รองรับกรณีข้ามวัน เที่ยงคืน 23:59 -> 00:00)
        if (current_gps_sec >= last_tx_gps_sec) {
            time_diff_s = current_gps_sec - last_tx_gps_sec;
        } else {
            // กรณีข้ามวัน (86400 คือจำนวนวินาทีใน 1 วัน)
            time_diff_s = (86400 - last_tx_gps_sec) + current_gps_sec;
        }
    } else {
        // Fallback: ถ้าไม่มี GPS ให้ใช้ Tick เหมือนเดิม
        uint32_t current_tick_s = sb_timer_tick / div;
        uint32_t last_tx_tick_s = last_tx_tick / div;
        if (current_tick_s >= last_tx_tick_s)
            time_diff_s = current_tick_s - last_tx_tick_s;
        else 
            time_diff_s = 0;
    }
    // -------------------------------------------------------------
	
	// --- โหมด Manual Interval ---
    if (!aprs_config.smart_beacon) {
		// ถ้าตั้ง Interval เป็น 0 ถือว่าปิด Auto Send
        if (aprs_config.manual_interval <= 0) return;
        
        if (time_diff_s >= aprs_config.manual_interval) {
            APRS_SendBeacon_Now();
            // *** อัปเดตตัวแปรเวลา ทั้ง 2 ระบบ (เพื่อความต่อเนื่อง) ***
			last_tx_tick = sb_timer_tick;      // อัปเดต Tick (Backup)
			last_tx_gps_sec = current_gps_sec; // อัปเดต GPS Time (Main)
        }
        return;
    }

    // TODO: คำนวณ Interval ตาม Smart Beacon (ถ้าเปิด)
    // ตอนนี้ใช้แบบ Manual ไปก่อน
    //uint32_t interval_ms = aprs_config.manual_interval * 1000;
    
    // ใช้ System Tick เช็คเวลา (ต้องมีฟังก์ชันดึง Tick เช่น HAL_GetTick หรือใช้ตัวแปรนับ loop)
    // สมมติใช้ตัวแปร loop counter ง่ายๆ หรือต้อง implement timer จริงจัง
    // เพื่อความง่าย: ถ้าถึงเวลา ให้ส่ง (โค้ดนี้เป็น Concept)
    
    // if (Time_Now() - last_beacon_time > interval_ms) {
    //     APRS_SendBeacon_Now();
    //     last_beacon_time = Time_Now();
    // }
	
	// สมมติว่า Loop หลักทำงานไวมาก เราจะใช้ตัวนับมาถ่วงเวลา
    // ค่านี้ต้องจูนตามความเร็ว CPU (สมมติ 1 วินาที ≈ 2000 รอบ - ต้องลองเทสดู)
    // หรือถ้ามีฟังก์ชัน Time_Now() ที่แม่นยำให้ใช้ตัวนั้นแทน
    
    //aprs_timer_counter++;
    
    // แปลงวินาทีเป็นรอบ (สมมติ rough estimate)
    //uint32_t target_count = aprs_config.manual_interval * 2000; 
	
	// จูนค่านี้ตามความเร็ว Loop เครื่อง (ลองที่ 4000 ก่อนครับ ถ้าช้าไปให้ลดเลขลง)
    // สมมติ Loop วิ่งที่ 2ms -> 1 วินาที = 500 รอบ 
    // แต่ถ้า Loop เร็วมาก อาจจะถึง 2000-4000 รอบต่อวิ
    //uint32_t loops_per_sec = 4000; 
	// จากการทดสอบจริง เครื่องวิ่งได้ประมาณ 85-100 รอบต่อวินาที (ในหน้า GPS Info)
	//uint32_t loops_per_sec = 85; 
    //uint32_t target_count = aprs_config.manual_interval * loops_per_sec;
	
	// --- โหมด Smart Beacon ---
    if (!aprs_config.use_gps) return; // Smart Beacon ต้องใช้ GPS เท่านั้น

    // 1. แปลงข้อมูล GPS
    //float current_speed_knots = my_atof(val_speed); 
    //float current_speed_kmh = current_speed_knots * 1.852f;
	float current_speed_kmh = my_atof(val_speed);
    int current_course = (int)my_atof(val_course);
    float current_lat = my_atof(val_lat);
    float current_lon = my_atof(val_lon);
	
	// *** เพิ่ม: เช็คว่า GPS จับได้จริงหรือยัง (ถ้า Lat/Lon เป็น 0 หรือ Sats=0 อย่าเพิ่งทำอะไร) ***
    // (สมมติเช็คจากค่า lat ไม่เป็น 0)
    if (my_abs(current_lat) < 0.001f && my_abs(current_lon) < 0.001f) {
        return; 
    }
	
	// *** แก้ไขจุดบอดที่ 1: First Beacon ***
    // ถ้ายังไม่เคยส่งเลย และ GPS จับได้แล้ว -> ส่งทันที!
    if (!first_beacon_sent) {
        APRS_SendBeacon_Now();
		
        // *** อัปเดตตัวแปรเวลา ทั้ง 2 ระบบ (เพื่อความต่อเนื่อง) ***
        last_tx_tick = sb_timer_tick;      // อัปเดต Tick (Backup)
        last_tx_gps_sec = current_gps_sec; // อัปเดต GPS Time (Main)
		
        last_tx_lat = current_lat;
        last_tx_lon = current_lon;
        last_tx_course = current_course;
        first_beacon_sent = true;
        return; // จบงานรอบนี้
    }

    int target_interval = aprs_config.sb_conf.slow_rate_s;

    // 2. คำนวณ Interval ตามความเร็ว (Speed-based Rate)
    if (current_speed_kmh < (float)aprs_config.sb_conf.stationary_speed_kmh) {
        // จอดนิ่ง
        target_interval = aprs_config.sb_conf.stationary_rate_s;
    } 
    else if (current_speed_kmh < aprs_config.sb_conf.low_speed_kmh) {
        // ขับช้า
        target_interval = aprs_config.sb_conf.slow_rate_s;
    } 
    else if (current_speed_kmh > aprs_config.sb_conf.high_speed_kmh) {
        // ขับเร็ว
        target_interval = aprs_config.sb_conf.fast_rate_s;
    } 
    else {
        // ความเร็วปานกลาง: Linear Interpolation หรือ Proportional
        // สูตร: FastRate * HighSpeed / CurrentSpeed
        target_interval = (int)((float)aprs_config.sb_conf.fast_rate_s * (float)aprs_config.sb_conf.high_speed_kmh / current_speed_kmh);
        
        // Clamp ค่าไม่ให้เกิน Slow Rate
        if (target_interval > aprs_config.sb_conf.slow_rate_s) 
            target_interval = aprs_config.sb_conf.slow_rate_s;
    }
	
	bool trigger_send = false;
	
	//bool is_moving = (current_speed_kmh >= 3.0f); // เช็คว่าตอนนี้รถวิ่งไหม
	bool is_moving = (current_speed_kmh >= (float)aprs_config.sb_conf.stationary_speed_kmh);
	
	// *** เพิ่ม: Logic ยิงทันทีเมื่อจอด (Parking Beacon) ***
    // ถ้าแต่ก่อนวิ่งอยู่ (was_moving) แล้วตอนนี้หยุด (!is_moving)
    // และเวลาผ่านไปเกิน 15 วินาทีแล้ว (กันยิงซ้ำรัวๆ ตอนรถติดไหลๆ)
    if (was_moving && !is_moving && time_diff_s > 60) {
        trigger_send = true;
    }
	
	// อัปเดตสถานะจำไว้ใช้รอบหน้า
    was_moving = is_moving;
	
	// 3. ตรวจสอบเงื่อนไขเวลา (Time Trigger)
    if (time_diff_s >= target_interval) {
        trigger_send = true;
    }
	

    // 4. ตรวจสอบการเลี้ยว (Corner Pegging)
    // ทำงานเฉพาะเมื่อมีความเร็วระดับหนึ่ง และพ้น Turn Time มาแล้ว
    //if (!trigger_send && current_speed_kmh > 3.0f && time_diff_s > aprs_config.sb_conf.turn_time_s) {
		if (!trigger_send && current_speed_kmh > aprs_config.sb_conf.stationary_speed_kmh && time_diff_s > aprs_config.sb_conf.turn_time_s) {
        int course_diff = my_abs(current_course - last_tx_course);
        if (course_diff > 180) course_diff = 360 - course_diff; // ปรับให้เป็นมุมเล็กสุด

        if (course_diff > aprs_config.sb_conf.turn_angle) {
            trigger_send = true;
        }
    }
	
    // ตรวจสอบเงื่อนไข Distance (ถ้ายังไม่มีการสั่งส่งจากเงื่อนไขอื่น)
    if (!trigger_send && aprs_config.sb_conf.min_tx_dist_m > 0) {
        float dist_km, bear;
        
        // คำนวณระยะห่าง
        CalculateDistanceBearing(last_tx_lat, last_tx_lon, current_lat, current_lon, &dist_km, &bear);
        
        // *** แก้ไข: แปลง km เป็น m ตรงนี้ ***
        float dist_m = dist_km * 1000.0f;
        
        // เทียบหน่วย เมตร กับ เมตร
        if (dist_m >= aprs_config.sb_conf.min_tx_dist_m) {
            trigger_send = true; // สั่งให้ส่ง (แต่ยังไม่ส่งทันที ไปรอส่งข้างล่างทีเดียว)
        }
    }

    // --- ส่วนประมวลผลกลาง (Centralized Execution) ---
    // ส่ง Beacon เมื่อมีเงื่อนไขใดเงื่อนไขหนึ่งเป็นจริง (Time OR Distance OR Corner)
    if (trigger_send) {
        APRS_SendBeacon_Now();
		
		// *** อัปเดตตัวแปรเวลา ทั้ง 2 ระบบ (เพื่อความต่อเนื่อง) ***
        last_tx_tick = sb_timer_tick;      // อัปเดต Tick (Backup)
        last_tx_gps_sec = current_gps_sec; // อัปเดต GPS Time (Main)

		// ถ้าตอนส่ง GPS ยังไม่มา ให้ตั้งค่า last_tx_gps_sec เป็น -1 หรือค่าปัจจุบันไปก่อน
        if (last_tx_gps_sec < 0) last_tx_gps_sec = 0;
		
        // อัปเดตตัวแปรสถานะทั้งหมดทีเดียว (ป้องกันการลืมอัปเดตบางตัว)
        last_tx_lat = current_lat;
        last_tx_lon = current_lon;
        last_tx_course = current_course; // **อย่าลืมตัวนี้** สำคัญสำหรับการคำนวณมุมเลี้ยวครั้งต่อไป
    }
}


// -------------------------------------------------------------
// เมนู APRS Settings (ปรับปรุง)
// -------------------------------------------------------------
void SubApp_APRS_Settings(void) {
    int cursor = 0;
    const int N_ITEMS = 15;
    int scroll_idx = 0;    // ตัวแปรสำหรับเลื่อนหน้าจอ
    
    WaitForRelease();
    while(1) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("APRS SETTINGS", 0, 0, 0);
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);

        // คำนวณการเลื่อนหน้าจอ (Show 5 lines: Y=2 to 6)
        if (cursor < scroll_idx) scroll_idx = cursor;
        if (cursor >= scroll_idx + 5) scroll_idx = cursor - 4;

        for (int i = 0; i < 5; i++) {
            int item_idx = scroll_idx + i;
            if (item_idx >= N_ITEMS) break;
            
            char buff[40];
            char marker = (cursor == item_idx) ? '>' : ' ';
            int y_pos = 2 + i; // เริ่มบรรทัดที่ 2
			
			//if (item_idx == 0) sprintf(buff, "%c APRS: %s", marker, aprs_config.aprs_on ? "ON" : "OFF");
			// --- 1. APRS Mode: OFF / ON / SMART ---
            if (item_idx == 0) {
                char *mode = "OFF";
                if (aprs_config.aprs_on) {
                    mode = aprs_config.smart_beacon ? "SMART" : "ON";
                }
                sprintf(buff, "%c APRS: %s", marker, mode);
            }
			
			// --- 2. Smart Beacon Settings (Menu ใหม่) ---
            else if (item_idx == 1) {
                sprintf(buff, "%c SMART BEACON >", marker);
            }
			
			// --- 3. Interval (ถ้า Smart เปิด ให้โชว์ AUTO) ---
            else if (item_idx == 2) {
                if (aprs_config.smart_beacon)
                    sprintf(buff, "%c INTERVAL: AUTO", marker);
                else if (aprs_config.manual_interval == 0)
                    sprintf(buff, "%c INTERVAL: OFF", marker);
                else
                    sprintf(buff, "%c INTERVAL: %d", marker, aprs_config.manual_interval);
            }
			// --- เมนูใหม่: TX FREQ (5 ทศนิยม) ---
            else if (item_idx == 3) {
                uint32_t mhz = aprs_config.tx_freq / 1000000;
                // เอาเศษ 6 หลัก แล้วหาร 10 เพื่อเหลือ 5 หลัก (หน่วย 10Hz)
                uint32_t decimals = (aprs_config.tx_freq % 1000000) / 10; 
                sprintf(buff, "%c FREQ: %lu.%05lu", marker, mhz, decimals);
            }
			
			// --- แก้ไขเมนู TX PWR ให้โชว์ L1-L5, MID, HIGH ---
            else if (item_idx == 4) {
                char pwr_str[10];
                if (aprs_config.tx_power < 5) sprintf(pwr_str, "L%d", aprs_config.tx_power + 1);
                else if (aprs_config.tx_power == 5) strcpy(pwr_str, "MID");
                else strcpy(pwr_str, "HIGH");
                
                sprintf(buff, "%c TX PWR: %s", marker, pwr_str);
            }
			// --- เมนูใหม่: PREAMBLE ---
            else if (item_idx == 5) {
                sprintf(buff, "%c PREAMBLE: %d", marker, aprs_config.preamble);
            }
            //else if (item_idx == 5) sprintf(buff, "%c MY: %s-%d", marker, aprs_config.callsign, aprs_config.ssid);
			// ของใหม่: เช็คก่อนว่ามี SSID ไหม
			else if (item_idx == 6) {
				if (aprs_config.ssid == 0)
					sprintf(buff, "%c MY: %s", marker, aprs_config.callsign);
				else
					sprintf(buff, "%c MY: %s-%d", marker, aprs_config.callsign, aprs_config.ssid);
			}
            else if (item_idx == 7) sprintf(buff, "%c TO: %s", marker, aprs_config.dest_call);
            // เพิ่มเมนู PATH ตรงนี้
            else if (item_idx == 8) sprintf(buff, "%c PATH: %s", marker, aprs_config.digipath); 
            else if (item_idx == 9) sprintf(buff, "%c SYM: %c%c", marker, aprs_config.icon_table, aprs_config.icon_symbol);
            else if (item_idx == 10) sprintf(buff, "%c POS: %s", marker, aprs_config.use_gps ? "GPS" : "FIXED");
            else if (item_idx == 11) sprintf(buff, "%c LAT: %s", marker, aprs_config.fixed_lat);
            else if (item_idx == 12) sprintf(buff, "%c LON: %s", marker, aprs_config.fixed_lon);

			/*}  else if (item_idx == 5) { // เมนูใหม่: แก้ Lat
                sprintf(buff, "%c LAT: %s", marker, aprs_config.fixed_lat);
            } else if (item_idx == 6) { // เมนูใหม่: แก้ Lon
                sprintf(buff, "%c LON: %s", marker, aprs_config.fixed_lon);
            } */
			// --- เมนูใหม่: COMMENT ---
			else if (item_idx == 13) {
                // ตัดข้อความให้สั้นลงถ้ามันยาวเกินจอ
                char short_cmt[8];
                strncpy(short_cmt, aprs_config.comment, 5);
                short_cmt[5] = '\0';
                if(strlen(aprs_config.comment) > 5) strcat(short_cmt, "..");
                
                sprintf(buff, "%c COMMENT: %s", marker, short_cmt);
            }
			// *** เพิ่มเมนูใหม่: BACKLIGHT ***
            else if (item_idx == 14) {
                if(aprs_config.bl_time == 0) sprintf(buff, "%c BL TIME: ON", marker);
                else sprintf(buff, "%c BL TIME: %ds", marker, aprs_config.bl_time);
            }
			// *** เมนูใหม่ (15): TIMER TUNE ***
            /* else if (item_idx == 15) {
                sprintf(buff, "%c DELAY TUNE: %d", marker, aprs_config.timer_divider);
            } */
			
            UI_PrintStringSmallBold(buff, 0, 0, y_pos);
        }

        ST7565_BlitFullScreen();
        
        KEY_Code_t key = KEYBOARD_Poll();
		HandleBacklight(key); // *** เพิ่มบรรทัดนี้: เลี้ยงไฟจอในหน้าเมนู ***
		
        if (key == KEY_EXIT) {
			WaitForRelease();
			return;
		}
        
        if (key == KEY_DOWN) { 
            cursor++; 
            if(cursor >= N_ITEMS) cursor = 0; 
            // หน่วงเวลานิดหน่อยเพื่อให้กดเลื่อนได้ลื่นๆ
            //volatile uint32_t d = 10000; while (d--) {} 
			SYSTEM_DelayMs(150); // หน่วง 150ms กำลังดี
        }
        if (key == KEY_UP) { 
            cursor--; 
            if(cursor < 0) cursor = N_ITEMS-1; 
            //volatile uint32_t d = 10000; while (d--) {} 
			SYSTEM_DelayMs(150); // หน่วง 150ms กำลังดี
        }
        
        if (key == KEY_MENU) {
            WaitForRelease();
			
            // 1. Logic APRS Mode: OFF -> ON -> SMART -> OFF
            if (cursor == 0) {
                if (!aprs_config.aprs_on) {
                    aprs_config.aprs_on = true;
                    aprs_config.smart_beacon = false;
                } else if (aprs_config.aprs_on && !aprs_config.smart_beacon) {
                    aprs_config.smart_beacon = true;
                } else {
                    aprs_config.aprs_on = false;
                    aprs_config.smart_beacon = false;
                }
            }
			// 2. เข้าเมนู Smart Beacon
            else if (cursor == 1) {
                SubApp_SmartBeacon_Settings();
            }
			// 3. Interval (แก้ไขเฉพาะตอน Manual)
            else if (cursor == 2) {
                if (!aprs_config.smart_beacon) {
					// แปลงค่า int เป็น string เพื่อแก้ไข
                    char tmp_buf[10];
					sprintf(tmp_buf, "%d", aprs_config.manual_interval);
					
					// เรียกใช้ EditString (numeric_only = true)
					//if(EditString("INTERVAL (Sec)", tmp_buf, 5, true))
                    if(EditString("INTERVAL", tmp_buf, 5, true)) {
						// แปลงกลับเป็น int (ใช้ my_atof แล้ว cast เป็น int)
                        aprs_config.manual_interval = (int)my_atof(tmp_buf);
                    }
                }
            }
			// --- เมนูใหม่: TX FREQ (5 ทศนิยม) ---
            else if (cursor == 3) {
                char tmp_buf[20];
                uint32_t mhz = aprs_config.tx_freq / 1000000;
                uint32_t decimals = (aprs_config.tx_freq % 1000000) / 10;
                
                sprintf(tmp_buf, "%lu.%05lu", mhz, decimals);
                
                // เพิ่ม max_len เป็น 10 เพื่อรองรับ "XXX.XXXXX"
                if(EditString("TX FREQ (MHz)", tmp_buf, 10, true)) {
                    // แปลงกลับเป็น Hz แบบ Manual (เพื่อเลี่ยง Float Precision Loss)
                    uint32_t new_mhz = 0;
                    uint32_t new_dec = 0;
                    
                    char *dot = strchr(tmp_buf, '.');
                    if(dot) {
                        *dot = '\0'; // ตัด String ตรงจุด
                        
                        // แปลงส่วน MHz
                        new_mhz = (uint32_t)my_atof(tmp_buf); 
                        
                        // แปลงส่วนทศนิยม
                        char *p = dot + 1;
                        // อ่านตัวเลข 5 ตัวแรก
                        int count = 0;
                        while(*p >= '0' && *p <= '9' && count < 5) { 
                            new_dec = new_dec*10 + (*p - '0'); 
                            p++; count++;
                        }
                        // ถ้ากรอกไม่ครบ 5 ตัว ให้เติม 0 ต่อท้าย
                        while(count < 5) { new_dec *= 10; count++; }
                    }
                    
                    // คำนวณกลับเป็น Hz (MHz * 1,000,000 + Decimals * 10)
                    aprs_config.tx_freq = (new_mhz * 1000000) + (new_dec * 10);
                }
            }
			
			// --- แก้ไขปุ่มกด ให้วนลูป 0-6 (L1-H) ---
            else if (cursor == 4) { 
                aprs_config.tx_power++; 
                if(aprs_config.tx_power > 6) aprs_config.tx_power = 0; // 0=L1 ... 6=HIGH
            }
			
			// --- เมนูใหม่: PREAMBLE ---
            else if (cursor == 5) {
                char tmp_buf[10]; sprintf(tmp_buf, "%d", aprs_config.preamble);
                // ให้แก้ค่า Preamble (หน่วยเป็น Flags)
                if(EditString("PREAMBLE (Flags)", tmp_buf, 4, true)) {
                    aprs_config.preamble = (int)my_atof(tmp_buf);
                    // กันค่ามั่ว (อย่างน้อย 10 flags, ไม่เกิน 500)
                    if(aprs_config.preamble < 10) aprs_config.preamble = 10;
                    if(aprs_config.preamble > 500) aprs_config.preamble = 500;
                }
            }
            //else if (cursor == 5) EditString("MY CALL", aprs_config.callsign, 6, false);
			else if (cursor == 6) {
				// เตรียมตัวแปรชั่วคราวสำหรับรวมร่าง (เช่น "E25WOP-7")
				char temp_full_call[12]; 
				
				if (aprs_config.ssid == 0)
					sprintf(temp_full_call, "%s", aprs_config.callsign);
				else
					sprintf(temp_full_call, "%s-%d", aprs_config.callsign, aprs_config.ssid);

				// เรียก EditString โดยให้พิมพ์ได้ยาวขึ้น (9 ตัวอักษร)
				// ผู้ใช้สามารถพิมพ์ E25WOP-9 หรือลบ -9 ออกก็ได้ตามใจ
				if (EditString("MY CALL-SSID", temp_full_call, 9, false)) {
					
					// --- ส่วนแยกร่างกลับคืน (Parse) ---
					char *dash = strchr(temp_full_call, '-'); // หาขีดกลาง
					
					if (dash) {
						// กรณีเจอขีด (เช่น E25WOP-9)
						*dash = 0; // ตัดข้อความตรงขีดให้ขาดออกจากกัน
						
						strcpy(aprs_config.callsign, temp_full_call); // เก็บส่วนหน้าเป็น Callsign
						aprs_config.ssid = (int)my_atof(dash + 1);    // แปลงส่วนหลังเป็นเลข SSID
					} else {
						// กรณีไม่เจอขีด (พิมพ์แค่ E25WOP)
						strcpy(aprs_config.callsign, temp_full_call);
						aprs_config.ssid = 0; // ถือว่า SSID เป็น 0
					}
				}
			}
            else if (cursor == 7) EditString("DEST CALL", aprs_config.dest_call, 6, false);
            // cursor 3 (ICON) ข้ามไปก่อน หรือจะทำเพิ่มก็ได้
			// แก้ไข PATH
            else if (cursor == 8) EditString("PATH", aprs_config.digipath, 9, false);
			
			// --- เมนูเลือก ICON (SYM) แบบ Presets + Custom ---
            else if (cursor == 9) {
                // เพิ่ม "Custom" ต่อท้ายรายการ
                const char* names[] = {"Car", "Runner",  "Motorcycle", "Bike", "House", "Van", "Truck",	"CUSTOM"};
                const char syms[]   = {'>',     '[',         '<',			'b',    '-',     'v',   'k',	'?'};
                const int n_icons = 8; // เพิ่มจำนวนเป็น 
                
                int icon_idx = 0;
                // หา Index ปัจจุบัน
                for(int i=0; i<n_icons-1; i++) if(aprs_config.icon_symbol == syms[i]) icon_idx = i;
                
                while(1) {
                    UI_DisplayClear();
                    UI_PrintStringSmallBold("SELECT SYMBOL", 0, 0, 0);
                    UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);
                    
                    char buf[32];
                    sprintf(buf, "< %s >", names[icon_idx]);
                    UI_PrintStringSmallBold(buf, 0, 0, 3);
                    
                    if (icon_idx < n_icons - 1) {
                        sprintf(buf, "Code: /%c", syms[icon_idx]);
                        UI_PrintStringSmallNormal(buf, 35, 0, 5);
                    } else {
                        UI_PrintStringSmallNormal("Type your own", 30, 0, 5);
                    }
                    
                    ST7565_BlitFullScreen();
                    
                    KEY_Code_t k = KEYBOARD_Poll();
                    if(k == KEY_EXIT) { WaitForRelease(); break; }
                    
                    if(k == KEY_MENU) {
                        WaitForRelease();
                        if (icon_idx < n_icons - 1) {
                            aprs_config.icon_symbol = syms[icon_idx];
                            aprs_config.icon_table = '/'; 
                        } else {
                            // --- Custom Symbol Logic ---
                            char tmp_table = aprs_config.icon_table;
                            char tmp_sym = aprs_config.icon_symbol;
                            int edit_step = 0; // 0=Table, 1=Symbol
                            
                            while(1) {
                                UI_DisplayClear();
                                UI_PrintStringSmallBold("CUSTOM SYMBOL", 0, 0, 0);
                                UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);
                                
                                char t_buf[20], s_buf[20];
                                sprintf(t_buf, "Table:  %c", tmp_table);
                                sprintf(s_buf, "Symbol: %c", tmp_sym);
                                
                                UI_PrintStringSmallBold(t_buf, 5, 0, 3);
                                UI_PrintStringSmallBold(s_buf, 5, 0, 5);
                                
                                // Cursor ชี้ (วางที่ x=95 ไม่บังตัวหนังสือ)
                                if(edit_step == 0) UI_PrintStringSmallBold("<<<", 80, 0, 3);
                                else UI_PrintStringSmallBold("<<<", 80, 0, 5);
                                
                                ST7565_BlitFullScreen();
                                
                                KEY_Code_t k2 = KEYBOARD_Poll();
                                if(k2 == KEY_EXIT) { WaitForRelease(); break; }
                                
                                if(k2 == KEY_MENU) {
                                    WaitForRelease();
                                    edit_step++;
                                    if(edit_step > 1) {
                                        aprs_config.icon_table = tmp_table;
                                        aprs_config.icon_symbol = tmp_sym;
                                        break;
                                    }
                                }
                                
                                if(k2 == KEY_UP || k2 == KEY_DOWN) {
                                    if (edit_step == 0) {
                                        // *** แก้ไขตรงนี้: สลับแค่ / กับ \ เท่านั้น ***
                                        if (tmp_table == '/') tmp_table = '\\';
                                        else tmp_table = '/';
                                        SYSTEM_DelayMs(200); // หน่วงเวลาปุ่มกดสักหน่อย
                                    } else {
                                        // ส่วน Symbol ยังหมุนได้อิสระเหมือนเดิม
                                        if(k2 == KEY_UP) { tmp_sym++; if(tmp_sym > 126) tmp_sym = 33; } 
                                        else { tmp_sym--; if(tmp_sym < 33) tmp_sym = 126; }
                                        //volatile uint32_t d = 80000; while(d--){} 
										// *** แก้ตรงนี้: ใช้ SYSTEM_DelayMs แทน Loop เดิม ***
                                        SYSTEM_DelayMs(170); // 100ms กำลังดีสำหรับการหมุนตัวอักษรต่อเนื่อง
                                    }
                                }
                            }
                        }
                        break;
                    }
                    
                    if(k == KEY_DOWN) { icon_idx++; if(icon_idx >= n_icons) icon_idx = 0; SYSTEM_DelayMs(150); }
                    if(k == KEY_UP) { icon_idx--; if(icon_idx < 0) icon_idx = n_icons - 1; SYSTEM_DelayMs(150); }
                }
            }
			
            else if (cursor == 10) aprs_config.use_gps = !aprs_config.use_gps;
            
            // เข้าเมนูแก้ไขพิกัด (ใช้ EditString ตัวเดิมที่มีอยู่)
            //if (cursor == 5) EditString("FIX LAT (DDMM.mmN)", aprs_config.fixed_lat, 9, false);
            //if (cursor == 6) EditString("FIX LON (DDDMM.mmE)", aprs_config.fixed_lon, 10, false);
			
			// อนุญาตให้กรอกแบบ Decimal (ใช้ตัวเลข, จุด, ลบ)
            else if (cursor == 11) EditString("LAT (Decimal)", aprs_config.fixed_lat, 10, true);
            else if (cursor == 12) EditString("LON (Decimal)", aprs_config.fixed_lon, 11, true);
            
			// --- เมนูใหม่: COMMENT ---
            else if (cursor == 13) {
                // ให้พิมพ์ได้สูงสุด 30 ตัวอักษร
                EditString("COMMENT", aprs_config.comment, 30, false);
            }
			// *** เพิ่มเงื่อนไขสำหรับเมนู 13 ***
            else if (cursor == 14) {
                char tmp_buf[10]; sprintf(tmp_buf, "%d", aprs_config.bl_time);
                if(EditString("BL TIME (Sec)", tmp_buf, 3, true)) {
                    aprs_config.bl_time = (int)my_atof(tmp_buf);
                }
            }
			// *** เมนูใหม่ (15): แก้ไขตัวหาร ***
            /* else if (cursor == 15) {
                char tmp_buf[10]; sprintf(tmp_buf, "%d", aprs_config.timer_divider);
                // ให้แก้ค่าได้ตั้งแต่ 10 ถึง 200
                if(EditString("DELAY TUNE", tmp_buf, 3, true)) {
                    int val = (int)my_atof(tmp_buf);
                    if(val < 10) val = 10;   // เร็วสุด
                    if(val > 200) val = 200; // ช้าสุด
                    aprs_config.timer_divider = val;
                }
            } */
			
            WaitForRelease();
        }
        // ลดการหน่วงเวลา Loop หลักลงเพื่อให้หน้าจอตอบสนองไวขึ้น
        // volatile uint32_t d = 10000; while (d--) {}
    }
}

// -------------------------------------------------------------
// เมนูตั้งค่า Smart Beacon (เพิ่มใหม่)
// -------------------------------------------------------------
void SubApp_SmartBeacon_Settings(void) {
    int cursor = 0;
    const int N_ITEMS = 9;
    char buf[16];
    
    WaitForRelease();
    while(1) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("SMART BEACON CFG", 0, 0, 0);
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);
        
        // แสดงเมนูแบบเลื่อน (เหมือน Settings หลัก)
        int start_idx = 0;
        if (cursor > 4) start_idx = cursor - 4;
        
        for (int i = 0; i < 5; i++) {
            int idx = start_idx + i;
            if (idx >= N_ITEMS) break;
            
            char marker = (cursor == idx) ? '>' : ' ';
            char text[40];
            int y = 2 + i;
            
			//if (idx == 0) sprintf(text, "%c STAT SPD:  %.1f km", marker, aprs_config.sb_conf.stationary_speed_kmh);
			if (idx == 0) {
				float val = aprs_config.sb_conf.stationary_speed_kmh;
				int int_part = (int)val;
				int dec_part = (int)((val - int_part) * 10); // เอาทศนิยม 1 ตำแหน่ง
				if (dec_part < 0) dec_part = -dec_part; // กันค่าติดลบ
				
				sprintf(text, "%c STAT SPD: %d.%d km", marker, int_part, dec_part);
			}
			else if (idx == 1) sprintf(text, "%c STATIONARY:%ds", marker, aprs_config.sb_conf.stationary_rate_s);
            else if (idx == 2) sprintf(text, "%c SLOW SPD: %d km", marker, aprs_config.sb_conf.low_speed_kmh);
            else if (idx == 3) sprintf(text, "%c SLOW RATE: %ds", marker, aprs_config.sb_conf.slow_rate_s);
            else if (idx == 4) sprintf(text, "%c FAST SPD: %d km", marker, aprs_config.sb_conf.high_speed_kmh);
            else if (idx == 5) sprintf(text, "%c FAST RATE: %ds", marker, aprs_config.sb_conf.fast_rate_s);
            else if (idx == 6) sprintf(text, "%c TURN ANG: %ddeg", marker, aprs_config.sb_conf.turn_angle);
            else if (idx == 7) sprintf(text, "%c TURN TIME: %ds", marker, aprs_config.sb_conf.turn_time_s);
            else if (idx == 8) {
                if(aprs_config.sb_conf.min_tx_dist_m == 0) sprintf(text, "%c DISTANCE: OFF", marker);
                else sprintf(text, "%c DISTANCE: %dm", marker, aprs_config.sb_conf.min_tx_dist_m);
            }
            
            UI_PrintStringSmallBold(text, 0, 0, y);
        }
        
        ST7565_BlitFullScreen();
        
        KEY_Code_t key = KEYBOARD_Poll();
        HandleBacklight(key);
        
        if (key == KEY_EXIT) {
			WaitForRelease();
			return;
		}
        if (key == KEY_DOWN) { cursor++; if(cursor >= N_ITEMS) cursor = 0; SYSTEM_DelayMs(150); }
        if (key == KEY_UP)   { cursor--; if(cursor < 0) cursor = N_ITEMS-1; SYSTEM_DelayMs(150); }
        
        if (key == KEY_MENU) {
            WaitForRelease();
			
/*             // ใช้ Logic แก้ไขตัวเลขแบบง่าย
            int *val_ptr = NULL;
            char *title = "";
            int max_chars = 4; */
            
            /* if (cursor == 0) { val_ptr = &aprs_config.sb_conf.stationary_rate_s; title = "STATIONARY (s)"; }
            else if (cursor == 1) { val_ptr = &aprs_config.sb_conf.low_speed_kmh; title = "SLOW SPEED (km)"; }
            else if (cursor == 2) { val_ptr = &aprs_config.sb_conf.slow_rate_s; title = "SLOW RATE (s)"; }
            else if (cursor == 3) { val_ptr = &aprs_config.sb_conf.high_speed_kmh; title = "FAST SPEED (km)"; }
            else if (cursor == 4) { val_ptr = &aprs_config.sb_conf.fast_rate_s; title = "FAST RATE (s)"; }
            else if (cursor == 5) { val_ptr = &aprs_config.sb_conf.turn_angle; title = "TURN ANGLE (deg)"; }
            else if (cursor == 6) { val_ptr = &aprs_config.sb_conf.turn_time_s; title = "TURN TIME (s)"; }
            else if (cursor == 7) { val_ptr = &aprs_config.sb_conf.min_tx_dist_m; title = "DISTANCE (m)"; }
            
            if (val_ptr) {
                sprintf(buf, "%d", *val_ptr);
                if (EditString(title, buf, max_chars, true)) {
                    *val_ptr = (int)my_atof(buf);
                }
            }
            WaitForRelease();
        } */

            // ---------------------------------------------------------
            // ในส่วน if (key == KEY_MENU) -> if (cursor == 1)
			// -----------------------------------------------------------
			if (cursor == 0) { 
				char buf[16];
				char *title = "STAT SPD (km)";
				
				// 1. แปลง Float เป็น String เอง (ไม่ง้อ %.1f)
				float val = aprs_config.sb_conf.stationary_speed_kmh;
				int int_part = (int)val;
				int dec_part = (int)((val - int_part) * 10); 
				if (dec_part < 0) dec_part = -dec_part;
				
				// ใส่ลง Buffer เช่น "0.5"
				sprintf(buf, "%d.%d", int_part, dec_part);

				// 2. เรียกหน้าจอแก้ไข (EditString รองรับจุดทศนิยมอยู่แล้ว ถ้า Numeric=true)
				if (EditString(title, buf, 6, true)) {
					// 3. แปลงกลับ (my_atof ทำงานได้ปกติ ไม่ต้องแก้)
					aprs_config.sb_conf.stationary_speed_kmh = my_atof(buf);
				}
			}
			// -----------------------------------------------------------
            // กรณีปกติ: แก้ไขค่าอื่น ๆ (ที่เป็น Integer / จำนวนเต็ม)
            // ---------------------------------------------------------
			else {
				// ตัวแปรชั่วคราวสำหรับแก้ไขค่า
				int temp_val = 0;
				char *title = "";
				int max_chars = 5;
				bool valid_selection = true;
				
				// 1. อ่านค่าปัจจุบันมาใส่ temp_val
				//else if (cursor == 0) { temp_val = aprs_config.sb_conf.stationary_speed_kmh; title = "STAT SPD (km)"; }
				if (cursor == 1) { temp_val = aprs_config.sb_conf.stationary_rate_s; title = "STATIONARY (s)"; }
				else if (cursor == 2) { temp_val = aprs_config.sb_conf.low_speed_kmh; title = "SLOW SPEED (km)"; }
				else if (cursor == 3) { temp_val = aprs_config.sb_conf.slow_rate_s; title = "SLOW RATE (s)"; }
				else if (cursor == 4) { temp_val = aprs_config.sb_conf.high_speed_kmh; title = "FAST SPEED (km)"; }
				else if (cursor == 5) { temp_val = aprs_config.sb_conf.fast_rate_s; title = "FAST RATE (s)"; }
				else if (cursor == 6) { temp_val = aprs_config.sb_conf.turn_angle; title = "TURN ANGLE (deg)"; }
				else if (cursor == 7) { temp_val = aprs_config.sb_conf.turn_time_s; title = "TURN TIME (s)"; }
				else if (cursor == 8) { temp_val = aprs_config.sb_conf.min_tx_dist_m; title = "DISTANCE (m)"; }
				else { valid_selection = false; }
            
				if (valid_selection) {
					sprintf(buf, "%d", temp_val);
					// เรียกหน้าจอแก้ไข
					if (EditString(title, buf, max_chars, true)) {
						// แปลงค่าที่แก้แล้วกลับเป็น int
						int new_val = (int)my_atof(buf);
						
						// 2. บันทึกกลับเข้า struct (Cast ให้ตรงชนิดตัวแปร)
						//else if (cursor == 0) aprs_config.sb_conf.stationary_speed_kmh = (uint16_t)new_val;
						if (cursor == 1) aprs_config.sb_conf.stationary_rate_s = (uint16_t)new_val;
						else if (cursor == 2) aprs_config.sb_conf.low_speed_kmh = (uint8_t)new_val;
						else if (cursor == 3) aprs_config.sb_conf.slow_rate_s = (uint16_t)new_val;
						else if (cursor == 4) aprs_config.sb_conf.high_speed_kmh = (uint8_t)new_val;
						else if (cursor == 5) aprs_config.sb_conf.fast_rate_s = (uint16_t)new_val;
						else if (cursor == 6) aprs_config.sb_conf.turn_angle = (uint8_t)new_val;
						else if (cursor == 7) aprs_config.sb_conf.turn_time_s = (uint8_t)new_val;
						else if (cursor == 8) aprs_config.sb_conf.min_tx_dist_m = (uint16_t)new_val;
					}
				}
			}
            WaitForRelease();
        }
        SYSTEM_DelayMs(20);
    }
}

// ฟังก์ชันสำหรับเคลียร์ข้อมูลขยะใน UART ทิ้งให้หมด
void Flush_GPS_UART(void) {
    // วนลูปอ่านจนกว่าจะไม่มีอะไรค้างในบัฟเฟอร์ (UART1 คือพอร์ต GPS ของบอร์ดส่วนใหญ่)
    while(UART1->SR & USART_SR_RXNE) {
        volatile uint8_t garbage = UART1->DR; // อ่านแล้วทิ้ง
        (void)garbage; // กัน Warning
    }
}

// =========================================================
// SMART BEACON STATUS DASHBOARD
// =========================================================
void SubApp_SmartBeacon_Status(void) {
    WaitForRelease();
    
    // ตัวแปรสำหรับ Refresh หน้าจอ
    uint32_t refresh_timer = 0;
	
    // [เพิ่ม 1] ตัวแปรจำเวลาส่งล่าสุด ไว้เช็คว่ามีการส่งเกิดขึ้นหรือไม่
    static uint32_t prev_tx_tick = 0;
    // อัปเดตค่าเริ่มต้นให้เท่ากับปัจจุบัน
    prev_tx_tick = last_tx_tick; 

    // [เพิ่ม 2] ตัวนับเวลาพัก (Cooldown Timer)
    int cooldown_timer = 0;
	
	//UI_DisplayClear();
    while(1) {
			// -----------------------------------------------------------------
        // ส่วนตรวจสอบการส่ง (TX Watchdog)
        // -----------------------------------------------------------------
        // ถ้าค่า last_tx_tick เปลี่ยนไป แสดงว่า APRS_Task เพิ่งจะยิงสัญญาณออกไป
        if (last_tx_tick != prev_tx_tick) {
            prev_tx_tick = last_tx_tick;     // จำค่าใหม่ไว้
            cooldown_timer = 60;            // สั่งพักยาวๆ 3 วินาที (100 * 33ms)
        }

        // ถ้าอยู่ในช่วงพักฟื้น (Cooldown Mode)
        if (cooldown_timer > 0) {
            cooldown_timer--;

            UI_DisplayClear();
            
            // 1. แจ้งสถานะบนหน้าจอ
            char buf[32];
            sprintf(buf, "TX DONE.. WAIT %d", cooldown_timer/20);
            UI_PrintStringSmallBold(buf, 10, 0, 3);
            ST7565_BlitFullScreen();

            // 2. [สำคัญมาก] ล้างขยะใน UART ทิ้งตลอดเวลาที่รอ
            // เพื่อไม่ให้ขยะ RFI ค้างคาอยู่ในท่อ
            Flush_GPS_UART(); 

            // 3. เช็คปุ่มออก (เผื่อ user อยากออก)
            KEY_Code_t key = KEYBOARD_Poll();
            if (key == KEY_EXIT) { WaitForRelease(); return; }

            // 4. หน่วงเวลาและ **ข้าม** การทำงานส่วนอื่นทั้งหมด (continue)
            // ทำให้ APRS_Task ไม่ถูกเรียกซ้ำ และ GPS_Fetch_Data ไม่ถูกเรียก
            SYSTEM_DelayMs(20);
            continue; 
        }
        // -----------------------------------------------------------------
		
		// วาดหน้าจอ (จุดเสี่ยงที่สุด ถ้าทำตอนส่งวิทยุ จอจะค้าง)
		UI_DisplayClear();
		
		// ตรวจสอบว่ากำลังส่งวิทยุอยู่หรือไม่? (สมมติชื่อตัวแปร g_TxState หรือเช็ค PTT)
        // ถ้ากำลังส่งอยู่ ห้ามไปยุ่งกับ GPS เดี๋ยวค่าเพี้ยนแล้ว Loop นรก
        //bool is_transmitting = (BK4819_GetGPIO(BK4819_GPIO0_PIN28) == 0); // ตัวอย่าง: เช็ค PTT หรือใช้ตัวแปร Global ของคุณ
		
		if(!aprs_config.smart_beacon && aprs_config.manual_interval > 0) {
			
/* 			if (refresh_timer % 15 == 0) {
				int dummy = 0;
				// อ่านแบบ silent=true (ไม่โชว์ Progress Bar, แอบอ่านเงียบๆ)
				GPS_Fetch_Data(true, &dummy); 
			} */
			//UI_DisplayClear();
			UI_PrintStringSmallBold("APRS OFF or Using", 3, 0, 2);
			UI_PrintStringSmallBold("Manual Interval", 11, 0, 4);
			//ST7565_BlitFullScreen();
			//SYSTEM_DelayMs(20);
		}
		else {
			// อัปเดตข้อมูลทุกๆ 200ms (ไม่ต้องถี่มาก เดี๋ยวลายตา)
			//if (refresh_timer % 10 == 0) {
				
				// -----------------------------------------------------------------
				// 2. [สำคัญ] สั่งให้รับข้อมูล GPS และรัน Smart Beacon Logic
				// -----------------------------------------------------------------
				// ถ้าไม่เรียก 2 บรรทัดนี้ ค่า speed และเวลาจะไม่ขยับเลย
				if (aprs_config.use_gps && (refresh_timer % 15 == 0)) {
					GPS_Fetch_Data(true, NULL); // อ่าน GPS แบบ Silent (ไม่อัปเดตหน้า GPS Info)
				}
				
				//UI_DisplayClear();
				UI_PrintStringSmallBold("SB STATUS MONITOR", 0, 0, 0);
				//UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);
		
				// 1. ดึงข้อมูลดิบมาคำนวณ
				//float spd_knots = my_atof(val_speed);
				//float spd_kmh = spd_knots * 1.852f;
				float spd_kmh = my_atof(val_speed);
				
				// คำนวณเวลาที่ผ่านไป (Time Diff)
				// *** ใช้ตัวหารจาก Config (กันค่าเป็น 0 ด้วย) ***
				uint32_t div = aprs_config.timer_divider; 
				if (div < 1) div = 28; // ค่ากันตาย
				
				// ป้องกันการหารด้วย 0 หรือค่าเพี้ยน
	/*             uint32_t current_time_s = sb_timer_tick / div;
				uint32_t last_tx_time_s = last_tx_tick / div;
				int elapsed_s = current_time_s - last_tx_time_s; // เวลาที่ผ่านไปแล้ว
				if (elapsed_s < 0) elapsed_s = 0; */
				
				// คำนวณ Elapsed Time แบบ Hybrid เพื่อโชว์บนหน้าจอ
				int32_t current_gps_sec = Get_GPS_Seconds();
				int elapsed_s = 0;

				if (aprs_config.use_gps && current_gps_sec >= 0) {
					 // ใช้สูตร GPS
					 if (current_gps_sec >= last_tx_gps_sec)
						elapsed_s = current_gps_sec - last_tx_gps_sec;
					 else
						elapsed_s = (86400 - last_tx_gps_sec) + current_gps_sec;
				} else {
					 // ใช้สูตร Tick (Backup)
					 uint32_t div = aprs_config.timer_divider; 
					 if (div < 1) div = 28;
					 elapsed_s = (sb_timer_tick / div) - (last_tx_tick / div);
				}
				
				if (elapsed_s < 0) elapsed_s = 0;
				
				// 2. คำนวณสถานะปัจจุบัน (State Machine)
				int state = 0; // 0=Stat, 1=Slow, 2=Fast, 3=Other
				int target_interval = aprs_config.sb_conf.stationary_rate_s;
				
				if (spd_kmh < (float)aprs_config.sb_conf.stationary_speed_kmh) {
					state = 0; // Stationary
					target_interval = aprs_config.sb_conf.stationary_rate_s;
				} else if (spd_kmh < aprs_config.sb_conf.low_speed_kmh ) {
					state = 1; // Slow
					target_interval = aprs_config.sb_conf.slow_rate_s;
				} else if (spd_kmh > aprs_config.sb_conf.high_speed_kmh) {
					state = 2; // Fast
					target_interval = aprs_config.sb_conf.fast_rate_s;
				} else {
					state = 3; // Other (Mid)
					// คำนวณ Interval แบบแปรผันตามความเร็ว
					if (spd_kmh > 0) {
						target_interval = (int)((float)aprs_config.sb_conf.fast_rate_s * (float)aprs_config.sb_conf.high_speed_kmh / spd_kmh);
					}
					if (target_interval > aprs_config.sb_conf.slow_rate_s) target_interval = aprs_config.sb_conf.slow_rate_s;
					if (target_interval < aprs_config.sb_conf.fast_rate_s) target_interval = aprs_config.sb_conf.fast_rate_s;
				}
				
				// 3. แสดงผล Time-based Rules
				char buf[32];
				char marker;
				int remain;
				
				// --- STATIONARY ---
				marker = (state == 0) ? '>' : ' ';
				remain = (state == 0) ? (target_interval - elapsed_s) : 0;
				if (state == 0 && remain < 0) remain = 0;
				sprintf(buf, "%cSTATIONARY:%ds", marker, (state == 0) ? remain : 0);
				UI_PrintStringSmallNormal(buf, 0, 0, 2);

				// --- SLOW ---
				marker = (state == 1) ? '>' : ' ';
				remain = (state == 1) ? (target_interval - elapsed_s) : 0;
				if (state == 1 && remain < 0) remain = 0;
				sprintf(buf, "%cSLOW : %ds", marker, (state == 1) ? remain : 0);
				UI_PrintStringSmallNormal(buf, 0, 0, 3);
				
				// --- FAST ---
				marker = (state == 2) ? '>' : ' ';
				remain = (state == 2) ? (target_interval - elapsed_s) : 0;
				if (state == 2 && remain < 0) remain = 0;
				sprintf(buf, "%cFAST : %ds", marker, (state == 2) ? remain : 0);
				UI_PrintStringSmallNormal(buf, 0, 0, 4);

				// --- TURN (Event) ---
				// แสดงสถานะ Cooldown
				int turn_cd = aprs_config.sb_conf.turn_time_s - elapsed_s;
				if (turn_cd <= 0) sprintf(buf, " TURN:READY (>%d)", aprs_config.sb_conf.turn_angle);
				else sprintf(buf, " TURN : WAIT %ds", turn_cd);
				
				// ถ้าเลี้ยวจริง (มุมเปลี่ยนเยอะ) ให้ใส่ลูกศรแวบๆ หรือ Highlight ก็ได้ 
				// แต่นี่เอาแค่สถานะ Cooldown ก่อน
				UI_PrintStringSmallNormal(buf, 0, 0, 5);

				// --- DISTANCE (Event) ---
				if (aprs_config.sb_conf.min_tx_dist_m > 0) {
					float lat = my_atof(val_lat);
					float lon = my_atof(val_lon);
					float dist_km, bear;
					CalculateDistanceBearing(lat, lon, last_tx_lat, last_tx_lon, &dist_km, &bear);
					int dist_m = (int)(dist_km * 1000.0f);
					int dist_remain = aprs_config.sb_conf.min_tx_dist_m - dist_m;
					
					if (dist_remain < 0) dist_remain = 0;
					sprintf(buf, " DIST:%dm left", dist_remain);
				} else {
					sprintf(buf, " DIST : OFF");
				}
				UI_PrintStringSmallNormal(buf, 0, 0, 6);
				
				// Info Bar ด้านล่าง (ความเร็วปัจจุบัน)
				sprintf(buf, "SPEED: %s km/h", val_speed);
				//UI_DrawRectangleBuffer(gFrameBuffer, 0, 56, 127, 63, true);
				// เขียนตัวหนังสือสีขาว (Invert) บนแถบดำไม่ได้ง่ายๆ ในไลบรารีนี้ 
				// งั้นเขียนทับแบบปกติไปก่อน หรือใช้เทคนิค XOR ถ้ามี
				UI_PrintStringSmallNormal(buf, 0, 0, 1); // อาจจะมองไม่เห็นบนพื้นดำ
				
				//ST7565_BlitFullScreen();
			//}
			
			//refresh_timer++;
		}
		
		refresh_timer++;
		
		// วาดหน้าจอ (จุดเสี่ยงที่สุด ถ้าทำตอนส่งวิทยุ จอจะค้าง)
		ST7565_BlitFullScreen();
		
        // อย่าลืมเรียก Task เพื่อให้ Beacon ทำงานจริง
        APRS_Task();

        KEY_Code_t key = KEYBOARD_Poll();
        HandleBacklight(key);
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        
        SYSTEM_DelayMs(20);
    }
}

// =========================================================
// APRS RX MONITOR (รับเป็นหลัก ดึง GPS เป็นรอง)
// =========================================================
/* void SubApp_APRS_RX(void) {
    
    // 1. ตั้งค่าเบื้องต้น
    UI_DisplayClear();
    UI_PrintStringSmallBold("APRS RX MONITOR", 0, 0, 0);
    UI_PrintStringSmallNormal("Waiting for signal...", 0, 0, 2);
    ST7565_BlitFullScreen();

    // ตัวแปรนับเวลาสำหรับดึง GPS
    uint32_t gps_timer = 0;
    
    // กำหนดรอบเวลาดึง GPS (Interval)
    // สมมติ Loop วิ่งไว เราจะดึงทุกๆ 5 วินาทีก็พอ (ไม่จำเป็นต้องดึงทุกวิ เพราะเปลือง CPU)
    // 5 วินาที x 50 loops/sec (ประมาณ) = 250
    const uint32_t GPS_POLL_INTERVAL = 250; 
    
    WaitForRelease();
    
    while(1) {
        // ====================================================
        // 1. PRIORITY สูงสุด: เช็คสัญญาณวิทยุ (RX)
        // ====================================================
        if (Is_RX_Active()) {
            
            // --- จังหวะนี้คือมีสัญญาณเข้า! ---
            
            // A. เปิดไฟจอเพื่อให้รู้ว่ามีของมา
            BACKLIGHT_TurnOn();
             */
            // B. โชว์สถานะรับ
/*             UI_DisplayClear();
            UI_PrintStringSmallBold("RECEIVING...", 0, 0, 0);
            // โชว์ความถี่ที่รับ
            char freq_buff[32];
            uint32_t rx_freq = gRxVfo->pRX->Frequency;
            sprintf(freq_buff, "%lu.%05lu", rx_freq/100000, rx_freq%100000);
            UI_PrintStringSmallBold(freq_buff, 0, 0, 2); 
			
            // C. วนลูปแสดงผลจนกว่าคลื่นจะหาย
            while(Is_RX_Active()) {
                // อัปเดต Bar Graph หรือ RSSI ตรงนี้ได้
                ST7565_BlitFullScreen();
                
                // เช็คปุ่ม Exit เผื่ออยากออกเลย
                if (KEYBOARD_Poll() == KEY_EXIT) { WaitForRelease(); return; }
                
                SYSTEM_DelayMs(20); // พัก CPU นิดหน่อย
            }
			*/
/* 			
			// วนลูปแสดงผลจนกว่าคลื่นจะหาย
            while(Is_RX_Active()) {
                UI_DisplayClear();
                UI_PrintStringSmallBold("RECEIVING...", 0, 0, 0);
                
                // 1. อ่านค่า RSSI ปัจจุบัน
                uint16_t current_rssi = BK4819_GetRSSI();
                
                // 2. แสดงความถี่
                char freq_buff[32];
                uint32_t rx_freq = gRxVfo->pRX->Frequency;
                sprintf(freq_buff, "%lu.%05lu", rx_freq/100000, rx_freq%100000);
                UI_PrintStringSmallBold(freq_buff, 0, 0, 2);
                
                // 3. แสดงค่า RSSI เป็นตัวเลข
                char rssi_buff[32];
                sprintf(rssi_buff, "RSSI: %d", current_rssi);
                UI_PrintStringSmallBold(rssi_buff, 0, 0, 4);
                
                // 4. (แถม) วาดกราฟแท่งความแรงสัญญาณ
                // RSSI ปกติอยู่ช่วง 0-300+ 
                // เราจะ Map เข้ากับความกว้างจอ (128 pixels)
                int bar_width = (current_rssi * 128) / 300; 
                if(bar_width > 128) bar_width = 128;
                UI_DrawRectangleBuffer(gFrameBuffer, 0, 50, bar_width, 55, true);
                
                ST7565_BlitFullScreen();
                
                if (KEYBOARD_Poll() == KEY_EXIT) { WaitForRelease(); return; }
                
                SYSTEM_DelayMs(50); 
            }
            
            // *** ตรงนี้คือจุดที่จะเรียก AFSK Decoder ในอนาคต ***
            // APRS_Process_Audio_Stream(); 
            
            // พอกลับสู่สภาวะปกติ ให้เคลียร์หน้าจอเตรียมรอรอบใหม่
            UI_DisplayClear();
            UI_PrintStringSmallBold("APRS RX MONITOR", 0, 0, 0);
            UI_PrintStringSmallNormal("Waiting...", 0, 0, 2);
            ST7565_BlitFullScreen();
            
            // รีเซ็ต Timer GPS เพื่อไม่ให้มันแย่งดึงข้อมูลทันทีหลังรับเสร็จ
            gps_timer = 0; 
        }

        // ====================================================
        // 2. PRIORITY รอง: ดึง GPS (ทำเมื่อว่างเท่านั้น)
        // ====================================================
        
        gps_timer++;
        
        if (gps_timer >= GPS_POLL_INTERVAL) {
            // ถึงเวลาดึง GPS แล้ว
            
            // แอบดึงแบบ Silent (แต่ฟังก์ชันนี้เราแก้ให้มันหยุดถ้ามี RX แล้ว)
            int dummy = 0;
            
            // โชว์จุดไข่ปลาให้รู้ว่าทำงานอยู่
            UI_PrintStringSmallNormal(".", 120, 0, 0); 
            ST7565_BlitFullScreen();
            
            // ดึงข้อมูล! (ถ้ามีสัญญาณเข้า มันจะดีดตัวออกมาเอง เพราะเราแก้ GPS_Fetch_Data ไว้แล้ว)
            GPS_Fetch_Data(true, &dummy); 
            
            // ลบจุดไข่ปลา
            UI_PrintStringSmallNormal(" ", 120, 0, 0);
            ST7565_BlitFullScreen();
            
            // รีเซ็ต Timer
            gps_timer = 0;
            
            // *** ถ้าจะให้ Tracker ทำงานด้วย (ยิง Beacon ออโต้) ให้เรียกตรงนี้ ***
             APRS_Task(); 
        }

        // ====================================================
        // 3. จัดการปุ่มกดและการแสดงผล
        // ====================================================
        
        KEY_Code_t key = KEYBOARD_Poll();
        HandleBacklight(key);
        
        if (key == KEY_EXIT) {
            WaitForRelease();
            return;
        }
        
        // ถ้ากด PTT หรือปุ่มข้าง ให้ยิง Beacon มือ
        if (key == KEY_PTT || key == KEY_SIDE1) {
             APRS_SendBeacon_Now();
             // พอยิงเสร็จ กลับมาหน้า RX ต่อ
             UI_DisplayClear();
             UI_PrintStringSmallBold("APRS RX MONITOR", 0, 0, 0);
             ST7565_BlitFullScreen();
        }

        SYSTEM_DelayMs(20);
    }
}
 */
 
 // ฟังก์ชันแกะ Packet (Parser)
void APRS_Parse_Packet(char *raw_text, APRS_RX_Packet_t *pkt) {
    // ล้างข้อมูลเก่า
    memset(pkt, 0, sizeof(APRS_RX_Packet_t));
    pkt->valid = false;
    
    // 1. หา Source Call (ผู้ส่ง) - อยู่หน้าสุด จนถึงเครื่องหมาย '>'
    char *p = raw_text;
    char *gr = strchr(p, '>');
    if (!gr) return; // ถ้าไม่มี > แสดงว่าไม่ใช่ APRS
    
    int len = gr - p;
    if (len > 9) len = 9;
    memcpy(pkt->source_call, p, len);
    pkt->source_call[len] = '\0';
    
    p = gr + 1; // ขยับไปหลัง >
    
    // 2. หา Dest Call / Path - อยู่จนถึงเครื่องหมาย ':'
    char *colon = strchr(p, ':');
    if (!colon) return; // ถ้าไม่มี : แสดงว่าไม่มีข้อมูล
    
    // เอาแค่ Dest Call ตัวแรก (ก่อนเครื่องหมาย ,)
    char *comma = strchr(p, ',');
    len = (comma && comma < colon) ? (comma - p) : (colon - p);
    if (len > 9) len = 9;
    memcpy(pkt->dest_call, p, len);
    pkt->dest_call[len] = '\0';
    
    p = colon + 1; // ขยับไปหลัง : (เข้าสู่ส่วนข้อมูล Data/Info)
    
    // 3. เก็บ Info ดิบไว้ก่อน
    strncpy(pkt->info, p, 59);
    
    // 4. ลองแกะพิกัด (ถ้ามี)
    // รูปแบบมาตรฐาน: ! หรือ = หรือ / ตามด้วย LAT(8) + SYM(1) + LON(9) + SYM(1)
    // ตัวอย่าง: !1344.48N/10030.00E>
    
    char *body = pkt->info;
    char data_type = body[0];
    
    // เช็คว่าเป็น Packet ระบุพิกัดหรือไม่ (!, =, /, @)
    if (data_type == '!' || data_type == '=' || data_type == '/' || data_type == '@') {
        // ตรวจสอบความยาวขั้นต่ำของพิกัด (ประมาณ 19-20 ตัวอักษร)
        if (strlen(body) >= 19) {
            // ขยับ Pointer ข้าม Data Type Identifier ไป
            char *pos = body + 1; 
            if (data_type == '@') pos += 7; // ถ้าเป็น @ จะมี Timestamp นำหน้า 7 ตัว
            
            // ตรวจสอบคร่าวๆ ว่าตัวอักษรเป็นตัวเลขไหม (กันพลาด)
            if (pos[0] >= '0' && pos[0] <= '9') {
                // LAT: 8 ตัว (1344.48N)
                memcpy(pkt->lat_str, pos, 8); 
                pkt->lat_str[8] = '\0';
                
                pkt->symbol_table = pos[8]; // Symbol Table Identifier (/)
                
                // LON: 9 ตัว (10030.00E)
                memcpy(pkt->lon_str, pos + 9, 9);
                pkt->lon_str[9] = '\0';
                
                pkt->symbol_code = pos[18]; // Symbol Code (>)
                
                pkt->valid = true;
            }
        }
    }
    
    // ถ้าแกะพิกัดไม่ได้ ให้ถือว่าเป็น Message หรือ Status อย่างเดียว
    if (!pkt->valid && strlen(pkt->source_call) > 0) {
        pkt->valid = true; // อย่างน้อยก็ได้ชื่อมา
    }
}
 
 // =========================================================
// APRS RX TEST LAB (ห้องทดลองรับสัญญาณ)
// =========================================================
/* void SubApp_APRS_RX_Test(void) {
    WaitForRelease();
    
    // สถานะการทำงาน: 0=Waiting, 1=Receiving, 2=Decoding, 3=ShowResult
    int state = 0;
    uint32_t rx_timer = 0;
    char status_msg[32] = "Ready";
    
    // จำลอง Buffer เก็บข้อมูล (เดี๋ยวอนาคตเราจะเอาเสียงใส่ตรงนี้)
    int sample_count = 0;

    while(1) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("APRS RX LAB", 0, 0, 0);
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);
        
        // แสดงสถานะปัจจุบัน
        if (state == 0) {
            UI_PrintStringSmallBold("STATUS: WAITING", 0, 0, 2);
            UI_PrintStringSmallNormal("Listening for signal...", 0, 0, 3);
            
            // แสดง RSSI แบบ Real-time เพื่อดูว่าหูตึงไหม
            uint16_t rssi = BK4819_GetRSSI();
            char rssi_buf[20];
            sprintf(rssi_buf, "RSSI: %d", rssi);
            UI_PrintStringSmallNormal(rssi_buf, 0, 0, 5);
            
            // วาดกราฟ RSSI ง่ายๆ
            int bar = (rssi * 100) / 300; if(bar>100) bar=100;
            UI_DrawRectangleBuffer(gFrameBuffer, 0, 50, bar, 54, true);

            // --- เงื่อนไขการเริ่มรับ (Trigger) ---
            // ถ้า RSSI สูงกว่า 100 (มีคนกดคีย์) -> เริ่มเข้าโหมดรับ
            if (rssi > 170) {
                state = 1;       // เปลี่ยนสถานะเป็น Receiving
                rx_timer = 0;    // เริ่มจับเวลา
                sample_count = 0;
            }
        }
        else if (state == 1) {
            UI_PrintStringSmallBold("STATUS: RECEIVING", 0, 0, 2);
            
            // แสดงเวลารับ
            char timer_buf[20];
            sprintf(timer_buf, "Time: %lu ms", rx_timer * 20);
            UI_PrintStringSmallNormal(timer_buf, 0, 0, 3);
            
            // จำลองการเก็บข้อมูล (อนาคตเราจะอัดเสียงตรงนี้)
            sample_count++;
            sprintf(status_msg, "Samples: %d", sample_count);
            UI_PrintStringSmallNormal(status_msg, 0, 0, 5);

            // เช็คว่าสัญญาณยังอยู่ไหม?
            uint16_t rssi = BK4819_GetRSSI();
            if (rssi < 150) {
                // สัญญาณหายแล้ว -> ไปถอดรหัสต่อ
                state = 2; 
            }
            
            // หรือถ้านานเกิน 2 วินาที (APRS packet ไม่ควรยาวกว่านี้) -> ตัดจบ
            if (rx_timer > 100) { 
                state = 2; 
            }
            
            rx_timer++;
        }
        else if (state == 2) {
            UI_PrintStringSmallBold("STATUS: DECODING...", 0, 0, 2);
            ST7565_BlitFullScreen();
            
            // --- พื้นที่สำหรับใส่โค้ดถอดรหัสในอนาคต ---
            // (ตอนนี้จำลองการทำงานไปก่อน)
            SYSTEM_DelayMs(500); // แกล้งทำเป็นคิดหนัก
            
            sprintf(status_msg, "Noise / No Data"); // สมมติว่าแกะไม่ออก
            state = 3; // ไปหน้าแสดงผล
        }
        else if (state == 3) {
            UI_PrintStringSmallBold("STATUS: FINISHED", 0, 0, 2);
            UI_PrintStringSmallNormal("Result:", 0, 0, 4);
            UI_PrintStringSmallBold(status_msg, 10, 0, 5);
            
            UI_PrintStringSmallNormal("[MENU] to Reset", 0, 0, 7);
            
            // กด MENU เพื่อเริ่มรอรับใหม่
            KEY_Code_t key = KEYBOARD_Poll();
            if (key == KEY_MENU) {
                state = 0;
                WaitForRelease();
            }
        }

        ST7565_BlitFullScreen();
        
        // ปุ่มออก
        KEY_Code_t key = KEYBOARD_Poll();
        HandleBacklight(key);
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        
        SYSTEM_DelayMs(20);
    }
} */

// เพิ่มตัวแปร Static เพื่อจำค่าเฉลี่ย (เก็บไว้นอกฟังก์ชัน หรือประกาศ static ข้างในก็ได้)
static int32_t signal_mean = 9500; // เริ่มต้นที่ค่าเฉลี่ยที่คุณบอกมา

// ฟังก์ชันรีเซ็ตระบบรับ
void RX_AFSK_Init(void) {
    memset(&rx_decoder, 0, sizeof(AFSK_Decoder_t));
    rx_decoder.receiving = false;
    rx_decoder.current_level = -1; // เริ่มต้นเป็น LOW
    signal_mean = 9500; // รีเซ็ตค่ากลางด้วย
}

// แก้ไขให้รับ Argument เป็นเบอร์ Register
uint16_t Hardware_Get_Audio_Sample(uint8_t reg_addr) {
    // อ่านค่าดิบจาก Register ที่ระบุ
    return BK4819_ReadRegister(reg_addr);
}

void RX_AFSK_Process_Sample(uint16_t adc_value) {
    // 1. หาค่ากลาง (ปรับสูตรให้ไวขึ้นนิดนึง จาก /32 เป็น /16)
    // เพื่อให้มันเกาะตาม DC Offset ได้ทัน
    signal_mean = ((signal_mean * 15) + adc_value) / 16;
    
    int32_t sample_32 = (int32_t)adc_value - signal_mean;
    int16_t sample = (int16_t)sample_32;

    rx_decoder.sample_counter++;

    // ---------------------------------------------
    // 2. SCHMITT TRIGGER (พระเอกของเรา)
    // ---------------------------------------------
    // ตั้งกำแพงไว้ที่ 4000 (ตามที่คุณบอกว่า noise วิ่งถึง 3500)
    // ถ้าสัญญาณจริงมาแรงกว่า Noise มันจะข้ามกำแพงนี้ได้
    const int16_t THRESHOLD_HIGH = 4000;
    const int16_t THRESHOLD_LOW  = -4000;
    
    bool crossed = false;

    if (rx_decoder.current_level == -1) {
        // ตอนนี้อยู่ข้างล่าง... ต้องสูงกว่ากำแพงบน ถึงจะยอมเปลี่ยน
        if (sample > THRESHOLD_HIGH) {
            rx_decoder.current_level = 1; // เปลี่ยนเป็น HIGH
            crossed = true;
        }
    } 
    else {
        // ตอนนี้อยู่ข้างบน... ต้องต่ำกว่ากำแพงล่าง ถึงจะยอมเปลี่ยน
        if (sample < THRESHOLD_LOW) {
            rx_decoder.current_level = -1; // เปลี่ยนเป็น LOW
            crossed = true;
        }
    }
    // ---------------------------------------------

    if (crossed) {
        // เจอจุดตัด (แบบผ่านกำแพงแล้ว)
        rx_decoder.zero_crossing_period = rx_decoder.sample_counter;
        rx_decoder.sample_counter = 0;
        
        // --- กรองความถี่ (Frequency Filter) ---
        // Noise มักจะมีความถี่สูงมาก (Period สั้นจู๋)
        // 1200Hz/2200Hz จะมีความกว้างระดับหนึ่ง
        // ถ้าคาบเวลามันสั้นเกินไป (เช่น น้อยกว่า 2 Sample) อาจจะเป็น Noise ที่หลุดรอดมา
        if (rx_decoder.zero_crossing_period < 2) {
             // สั้นไป! ไม่นับ
             return;
        }

        rx_decoder.ones_count++; 
        
        // ถ้าเจอต่อเนื่อง 4 ครั้งก็พอ (เพราะผ่าน Schmitt Trigger มาแล้ว ถือว่าชัวร์ระดับนึง)
        if (rx_decoder.ones_count > 4) { 
             rx_decoder.receiving = true;
             if(rx_decoder.ones_count > 10) rx_decoder.ones_count = 10;
        }
    }
}

/* void SubApp_APRS_RX_Test(void) {
    WaitForRelease();
    
    int state = 0; // 0=Wait, 1=RX, 2=Decode, 3=Show
    uint32_t rx_timer = 0;
    
    // ข้อความจำลอง (Mock Data)
    char *mock_packet = "E25WOP-9>APUVK1,WIDE1-1:!1344.48N/10030.00E>Test from Lab";

    while(1) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("APRS RX LAB", 0, 0, 0);
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);
        
        if (state == 0) {
            UI_PrintStringSmallBold("STATUS: WAITING", 0, 0, 2);
            UI_PrintStringSmallNormal("[PTT] to Simulate RX", 0, 0, 3); // <--- เพิ่มคำแนะนำ
            
            uint16_t rssi = BK4819_GetRSSI();
            char rssi_buf[20];
            sprintf(rssi_buf, "RSSI: %d", rssi);
            UI_PrintStringSmallNormal(rssi_buf, 0, 0, 5);
            
            // กด PTT เพื่อจำลองว่ารับสัญญาณได้ (Test Parser)
            if (KEYBOARD_Poll() == KEY_SIDE1) {
                state = 1;
                rx_timer = 0;
                // หน่วงเวลานิดนึงกันกดเบิ้ล
                SYSTEM_DelayMs(200); 
            }
        }
        else if (state == 1) {
            UI_PrintStringSmallBold("STATUS: RECEIVING", 0, 0, 2);
            char timer_buf[20];
            sprintf(timer_buf, "Time: %lu ms", rx_timer * 20); // แก้ %d เป็น %lu แล้ว
            UI_PrintStringSmallNormal(timer_buf, 0, 0, 3);
            
            // จำลองว่ารับเสร็จใน 1 วินาที
            if (rx_timer > 50) { 
                state = 2; 
            }
            rx_timer++;
        }
        else if (state == 2) {
            UI_PrintStringSmallBold("STATUS: DECODING...", 0, 0, 2);
            ST7565_BlitFullScreen();
            
            // *** เรียกใช้ Parser จริงๆ กับข้อมูลจำลอง ***
            APRS_Parse_Packet(mock_packet, &last_rx_packet);
            
            SYSTEM_DelayMs(500); // แกล้งคิด
            state = 3;
        }
        else if (state == 3) {
            // *** แสดงผลข้อมูลที่แกะได้จริง ***
            UI_PrintStringSmallBold("RX PACKET INFO:", 0, 0, 2);
            
            char buf[40];
            sprintf(buf, "FR: %s", last_rx_packet.source_call);
            UI_PrintStringSmallNormal(buf, 0, 0, 3);
            
            if (last_rx_packet.lat_str[0] != 0) {
                // ถ้ามีพิกัด ให้โชว์พิกัด
                sprintf(buf, "Pos: %s %c", last_rx_packet.lat_str, last_rx_packet.symbol_table);
                UI_PrintStringSmallNormal(buf, 0, 0, 4);
                sprintf(buf, "     %s %c", last_rx_packet.lon_str, last_rx_packet.symbol_code);
                UI_PrintStringSmallNormal(buf, 0, 0, 5);
            } else {
                // ถ้าไม่มีพิกัด โชว์ข้อความดิบ
                UI_PrintStringSmallNormal(last_rx_packet.info, 0, 0, 4);
            }

            UI_PrintStringSmallNormal("[MENU] Reset", 0, 0, 7);
            
            if (KEYBOARD_Poll() == KEY_MENU) {
                state = 0;
                WaitForRelease();
            }
        }

        ST7565_BlitFullScreen();
        
        KEY_Code_t key = KEYBOARD_Poll();
        HandleBacklight(key);
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        
        SYSTEM_DelayMs(20);
    }
} */

/* void SubApp_APRS_RX_Test(void) {
    WaitForRelease();
    
    // ตั้งค่าไปที่ 0xEE ตามที่คุณเจอ
    uint8_t target_reg = 0xEE; 
    
    uint16_t min_val = 65535;
    uint16_t max_val = 0;

    while(1) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("CALIBRATE 0xEE", 0, 0, 0);
        
        // อ่านค่าปัจจุบัน
        uint16_t val = BK4819_ReadRegister(target_reg);
        
        // หาค่าต่ำสุด/สูงสุด (เพื่อดูช่วงการแกว่ง)
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
        
        // กด MENU เพื่อรีเซ็ตค่า Min/Max (เผื่อค่าเก่าค้าง)
        if (KEYBOARD_Poll() == KEY_MENU) {
            min_val = 65535;
            max_val = 0;
        }

        char buf[32];
        // แสดงค่าปัจจุบันตัวใหญ่ๆ
        sprintf(buf, "NOW: %u", val);
        UI_PrintStringSmallBold(buf, 0, 0, 2);
        
        // แสดงค่า Min/Max ที่เคยเจอ
        sprintf(buf, "MIN: %u", min_val);
        UI_PrintStringSmallNormal(buf, 0, 0, 4);
        
        sprintf(buf, "MAX: %u", max_val);
        UI_PrintStringSmallNormal(buf, 0, 0, 5);
        
        // คำนวณความต่าง (Swing)
        sprintf(buf, "DIFF: %u", max_val - min_val);
        UI_PrintStringSmallNormal(buf, 0, 0, 6);

        ST7565_BlitFullScreen();
        if (KEYBOARD_Poll() == KEY_EXIT) { WaitForRelease(); return; }
        
        // อ่านไวๆ หน่อย
        SYSTEM_DelayMs(5); 
    }
} */
/* 
void SubApp_APRS_RX_Test(void) {
    WaitForRelease();
    
    // ตั้งค่า Register เป้าหมาย
    uint8_t target_reg = 0xEE; 
    
    // Thresholds (คำนวณจากข้อมูลที่คุณให้มา)
    // Min 3000, Max 21000 -> จุดกึ่งกลางประมาณ 12000
    // เราเผื่อระยะปลอดภัย (Hysteresis) ไว้หน่อย
    const uint16_t THRESHOLD_LOW_MAX = 8000;  // ต่ำกว่านี้คือ 1200Hz (Mark)
    const uint16_t THRESHOLD_HIGH_MIN = 16000; // สูงกว่านี้คือ 2200Hz (Space)
    
    // ตั้งค่า Squelch (RSSI ต้องมากกว่านี้ถึงจะทำงาน)
    const uint16_t RSSI_THRESHOLD = 80; 

    // Buffer สำหรับโชว์กราฟิก Bits วิ่งๆ
    char bit_stream[22];
    memset(bit_stream, ' ', 21);
    bit_stream[21] = '\0';

    while(1) {
        // 1. อ่าน RSSI ก่อน (Squelch Check)
        uint16_t rssi = BK4819_GetRSSI();
        
        UI_DisplayClear();
        UI_PrintStringSmallBold("DIGITAL DECODER", 0, 0, 0);
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);

        char buf[32];
        sprintf(buf, "RSSI: %d  (SQ:%d)", rssi, RSSI_THRESHOLD);
        UI_PrintStringSmallNormal(buf, 0, 0, 2);

        // 2. ถ้าสัญญาณแรงพอ ให้เริ่มอ่านบิต
        if (rssi > RSSI_THRESHOLD) {
            uint16_t val = BK4819_ReadRegister(target_reg);
            
            // แสดงค่า Raw
            sprintf(buf, "VAL: %u", val);
            UI_PrintStringSmallNormal(buf, 0, 0, 3);
            
            char current_bit = '?';
            
            // --- LOGIC แยกเสียง (Simple Slicer) ---
            if (val < THRESHOLD_LOW_MAX) {
                // เจอเสียงต่ำ (1200Hz)
                UI_PrintStringSmallBold("TONE: LOW (1200)", 0, 0, 5);
                current_bit = '_'; // สัญลักษณ์แทน Low
            } 
            else if (val > THRESHOLD_HIGH_MIN) {
                // เจอเสียงสูง (2200Hz)
                UI_PrintStringSmallBold("TONE: HIGH (2200)", 0, 0, 5);
                current_bit = '-'; // สัญลักษณ์แทน High
            } 
            else {
                // อยู่กึ่งกลาง (Transition)
                current_bit = '.';
            }
            
            // เลื่อน Bit stream (Shift Left) เพื่อทำภาพเคลื่อนไหว
            for(int i=0; i<20; i++) bit_stream[i] = bit_stream[i+1];
            bit_stream[20] = current_bit;
            
        } else {
            UI_PrintStringSmallNormal("Status: SQUELCHED", 0, 0, 3);
            UI_PrintStringSmallNormal("Waiting Signal...", 0, 0, 5);
        }
        
        // แสดง Bit Stream (เส้นกราฟิก _ - _ -)
        UI_PrintStringSmallBold(bit_stream, 5, 0, 6);

        ST7565_BlitFullScreen();
        if (KEYBOARD_Poll() == KEY_EXIT) { WaitForRelease(); return; }
        
        // Loop นี้ต้องไวหน่อย เพื่อให้ทันความเร็วบิต
        // แต่อย่าไวเกินจนจอ Refresh ไม่ทัน
        SYSTEM_DelayMs(2); 
    }
}
 */
void SubApp_RX_Scope(void) {
    WaitForRelease();
    
    uint8_t scope_buf[128]; 
    uint8_t current_reg = 0x6E; // เริ่มต้นที่ 0x6E เลย (ผู้ต้องสงสัยอันดับ 1)
    
    // ตัวแปรหน่วงเวลา (ยิ่งเยอะ ยิ่งอ่านช้า = เห็นคลื่นถี่ขึ้น)
    uint32_t scan_delay = 500; 
    
    while(1) {
        // 1. เก็บตัวอย่าง (Sampling)
        for(int i=0; i<128; i++) {
            // อ่านค่าจาก Register
            uint16_t val = Hardware_Get_Audio_Sample(current_reg);
            
            // Auto Scaling (อาจต้องจูนตัวหารตามหน้างาน)
            uint8_t y = (val / 1024); // ลองหาร 1024 ดูครับ ถ้ากราฟเตี้ยไปให้ลดลงเหลือ 512
            
            if(y > 55) y = 55;
            scope_buf[i] = y;
            
            // *** หน่วงเวลา (Timebase) ***
            // เพื่อให้เห็นรูปคลื่นชัดขึ้น
            volatile uint32_t d = scan_delay; 
            while(d > 0) d--; 
        }
        
        // 2. แสดงผล
        UI_DisplayClear();
        
        char head[40];
        // แสดง Register และค่า Delay ปัจจุบัน
        sprintf(head, "REG:0x%02X  DLY:%lu", current_reg, scan_delay);
        UI_PrintStringSmallBold(head, 0, 0, 0);
        
        UI_PrintStringSmallNormal("UP/DN:Reg S1/2:Zoom", 0, 0, 1);
        
        // วาดเส้น Baseline
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 55, 127, 55, true);
        
        for(int x=0; x<127; x++) {
            int y1 = 55 - scope_buf[x];
            int y2 = 55 - scope_buf[x+1]; // เอาบรรทัดนี้กลับมาใช้
            
            // หาจุดต่ำสุดและสูงสุด ระหว่างจุดปัจจุบันกับจุดถัดไป
            int y_min = (y1 < y2) ? y1 : y2;
            int y_max = (y1 > y2) ? y1 : y2;

            // วาดเส้นแนวตั้งเชื่อมระหว่าง 2 จุด (Connect the dots)
            // จะทำให้กราฟดูเป็นเส้นต่อเนื่อง ไม่ขาดตอน
            UI_DrawRectangleBuffer(gFrameBuffer, x, y_min, x, y_max, true);
        }
        
        ST7565_BlitFullScreen();
        
        // 3. ปุ่มควบคุม
        KEY_Code_t key = KEYBOARD_Poll();
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        
        // เปลี่ยน Register
        if (key == KEY_UP)   { current_reg--; SYSTEM_DelayMs(100); }
        if (key == KEY_DOWN) { current_reg++; SYSTEM_DelayMs(100); }
        
        // ปรับความเร็วการสแกน (Zoom In/Out)
        if (key == KEY_SIDE1) { // กดปุ่มข้างบน -> อ่านช้าลง (Zoom Out เห็นคลื่นเยอะขึ้น)
            scan_delay += 100; 
            if(scan_delay > 5000) scan_delay = 5000;
        }
        if (key == KEY_SIDE2) { // กดปุ่มข้างล่าง -> อ่านเร็วขึ้น (Zoom In ขยายคลื่น)
            if(scan_delay >= 100) scan_delay -= 100; 
            else scan_delay = 0;
        }
    }
}

// =========================================================
// APRS REAL DECODER v2 (Bit Packing Edition)
// =========================================================
//Pulse Width
/* void SubApp_APRS_RX_RealDecoder(void) {
    WaitForRelease();
    
    const uint8_t TARGET_REG = 0xEE; 
    
    // *** เร่งความเร็ว Loop ขึ้น 5 เท่า! ***
    const uint32_t LOOP_DELAY = 1500; 

    // ตัวแปรระบบ
    long moving_avg_center = 0;
    uint8_t current_level = 0;
    uint8_t last_level = 0;
    int sample_count = 0;
    
    // ตัวเก็บสถิติ
    int last_pulses[12]; // เก็บความกว้าง 6 ลูกล่าสุด
    int p_idx = 0;

    UI_DisplayClear();
    UI_PrintStringSmallBold("HIGH-RES CALIB", 0, 0, 0);
    UI_PrintStringSmallNormal("Init...", 0, 0, 2);
    ST7565_BlitFullScreen();
    
    // Init Center
    long sum = 0;
    for(int i=0; i<1000; i++) sum += BK4819_ReadRegister(TARGET_REG);
    moving_avg_center = sum / 1000;

    while(1) {
        if (KEYBOARD_Poll() == KEY_EXIT) { WaitForRelease(); return; }

        uint16_t val = BK4819_ReadRegister(TARGET_REG);
        
        // Auto-Center Tracking
        moving_avg_center = ((moving_avg_center * 63) + val) / 64;
        uint16_t CENTER = (uint16_t)moving_avg_center;
        
        // Schmitt Trigger (Gap 2000)
        // ถ้าค่าแกว่ง ให้ลองลด Gap เหลือ 1000
        if (val > (CENTER + 1000)) current_level = 1;
        else if (val < (CENTER - 1000)) current_level = 0;

        // วัดความกว้าง
        if (current_level == last_level) {
            sample_count++;
        } 
        else {
            // เจอขอบ (Edge) -> จบ 1 Pulse
            // บันทึกความกว้างลง Array (เฉพาะที่เป็นคลื่นจริงๆ ไม่ใช่ Noise สั้นๆ)
            if (sample_count > 3) { 
                last_pulses[p_idx] = sample_count;
                p_idx = (p_idx + 1) % 12; // วนลูปเก็บ 6 ตัวล่าสุด
                
                // แสดงผล Real-time
                UI_DisplayClear();
                UI_PrintStringSmallBold("PULSE WIDTHS:", 0, 0, 0);
                
                char buf1[32], buf2[32], buf3[32];
                // แสดง 3 ตัวแรก
                sprintf(buf1, "%d  %d  %d %d", last_pulses[0], last_pulses[1], last_pulses[2], last_pulses[3]);
                // แสดง 3 ตัวหลัง
                sprintf(buf2, "%d  %d  %d %d", last_pulses[4], last_pulses[5], last_pulses[6], last_pulses[7]);
				
				sprintf(buf3, "%d  %d  %d %d", last_pulses[8], last_pulses[9], last_pulses[10], last_pulses[11]);
                
                UI_PrintStringSmallNormal(buf1, 0, 0, 2);
                UI_PrintStringSmallNormal(buf2, 0, 0, 4);
				UI_PrintStringSmallNormal(buf3, 0, 0, 6);
                
                // คำแนะนำ
                //UI_PrintStringSmallNormal("Short ~10? Long ~20?", 0, 0, 6);
                
                ST7565_BlitFullScreen();
            }
            
            sample_count = 0;
            last_level = current_level;
        }

        for(volatile int d=0; d<LOOP_DELAY; d++); 
    }
} */

// ถ้าไม่มี header ให้ include ถึง SYSTICK_DelayUs ให้ประกาศไว้ก่อน
// void SYSTICK_DelayUs(uint32_t Delay);

// ต้อง include/prototype ให้มองเห็น
// void SYSTICK_DelayUs(uint32_t Delay);


void SubApp_APRS_RX_RealDecoder(void)
{
    WaitForRelease();

    // ------------------------------------------------------------
    // CONFIG
    // ------------------------------------------------------------
    const uint8_t  TARGET_REG      = 0xEE;

    // Sampling: 9600Hz => 8 samples/bit @ 1200 baud
    const uint16_t SAMPLE_RATE_HZ  = 9600;
    const uint8_t  SAMPLES_PER_BIT = 8;
    const uint16_t SAMPLE_US       = (uint16_t)(1000000UL / SAMPLE_RATE_HZ); // ~104us

    const int32_t  CENTER_IDLE_NUM   = 127;
    const int32_t  CENTER_IDLE_DEN   = 128;
    const int32_t  CENTER_ACTIVE_NUM = 1023;
    const int32_t  CENTER_ACTIVE_DEN = 1024;

    // Squelch (ต้อง OPEN > CLOSE)
    const int32_t  SQL_LEVEL_OPEN_DELTA  = 2000;
    const int32_t  SQL_LEVEL_CLOSE_DELTA = 1000;

    const uint16_t OPEN_HOLD_BITS  = 1200 / 5;   // ~0.2s
    const uint16_t CLOSE_HOLD_BITS = 1200 / 3;   // ~0.33s

    const int32_t  MINMAX_RELAX_DEN    = 32;
    const uint16_t MINMAX_WARMUP_BITS  = 1200 / 3; // ~0.33s (ลดลงจาก 0.5s ให้เริ่มหา flag เร็วขึ้น)

    const uint8_t  FLAGS_TO_LOCK = 3; // ถ้าจับยากลองลดเป็น 2

    const uint32_t PACKET_TIMEOUT_BITS = 1200U * 3U; // ~3s

    // UI rates
    const uint16_t UI_READY_DIV  = 960;  // ~10Hz
    const uint16_t UI_RX_DIVBITS = 240;  // ~5Hz

    // ------------------------------------------------------------
    // State
    // ------------------------------------------------------------
    int32_t moving_center = 0;
    int32_t idle_baseline = 0;

    bool squelch_open = false;
    uint16_t open_cnt = 0;
    uint16_t close_cnt = 0;

    // bit mean accumulator
    int32_t bit_sum = 0;
    uint8_t samples_in_bit = 0;
    int32_t bit_mean = 0;

    // leaky min/max
    int32_t bit_min = 0;
    int32_t bit_max = 0;
    bool    minmax_inited = false;
    uint32_t warmup_bits = 0;

    // tone
    bool last_tone_hi_A = false;
    bool last_tone_hi_B = false;

    // decoder A/B (ก่อน lock จะรันคู่)
    uint8_t shift_A = 0, shift_B = 0;
    int ones_A = 0, ones_B = 0;
    uint8_t flag_run_A = 0, flag_run_B = 0;

    bool packet_active = false;
    bool use_invert = false;          // false = A (เดิม), true = B (invert NRZI)
    uint32_t bit_counter_since_lock = 0;

    // Display/debug
    char raw_text[20];
    char shifted_text[20];
    int text_idx = 0;

    uint16_t ui_div = 0;
    uint16_t ui_tick = 0;
    char dbg1[40], dbg2[40], dbg3[40];
	int32_t last_thr = 0;
	int32_t last_m   = 0;


    memset(raw_text, ' ', 19); raw_text[19] = 0;
    memset(shifted_text, ' ', 19); shifted_text[19] = 0;

    // ------------------------------------------------------------
    // UI init
    // ------------------------------------------------------------
    UI_DisplayClear();
    UI_PrintStringSmallBold("APRS RX (AUTO)", 0, 0, 0);
    UI_PrintStringSmallNormal("Calibrating...", 0, 0, 2);
    ST7565_BlitFullScreen();

    // ------------------------------------------------------------
    // Calibrate idle baseline
    // ------------------------------------------------------------
    {
        int64_t sum = 0;
        for (int i = 0; i < 1200; i++) {
            sum += (int32_t)BK4819_ReadRegister(TARGET_REG);
            SYSTICK_DelayUs(SAMPLE_US);
        }
        idle_baseline = (int32_t)(sum / 1200);
        moving_center = idle_baseline;
    }

    UI_DisplayClear();
    UI_PrintStringSmallBold("READY", 0, 0, 0);
    UI_PrintStringSmallNormal("Waiting signal...", 0, 0, 2);
    ST7565_BlitFullScreen();

    while (1)
    {
        if (KEYBOARD_Poll() == KEY_EXIT) {
            WaitForRelease();
            return;
        }

        int32_t val = (int32_t)BK4819_ReadRegister(TARGET_REG);

        // center tracking
        if (!packet_active) {
            moving_center = (moving_center * CENTER_IDLE_NUM + val) / CENTER_IDLE_DEN;
        } else {
            moving_center = (moving_center * CENTER_ACTIVE_NUM + val) / CENTER_ACTIVE_DEN;
        }

        // ----------------- Squelch (with hold) -----------------
        const int32_t open_thr  = idle_baseline + SQL_LEVEL_OPEN_DELTA;
        const int32_t close_thr = idle_baseline + SQL_LEVEL_CLOSE_DELTA;

        if (!squelch_open) {
            if (moving_center > open_thr) {
                if (open_cnt < OPEN_HOLD_BITS) open_cnt++;
            } else {
                open_cnt = 0;
            }

            if (open_cnt >= OPEN_HOLD_BITS) {
                squelch_open = true;
                open_cnt = 0;
                close_cnt = 0;

                // reset
                packet_active = false;
                use_invert = false;
                bit_counter_since_lock = 0;

                bit_sum = 0;
                samples_in_bit = 0;

                minmax_inited = false;
                warmup_bits = 0;

                last_tone_hi_A = false;
                last_tone_hi_B = false;
                shift_A = shift_B = 0;
                ones_A = ones_B = 0;
                flag_run_A = flag_run_B = 0;

                memset(raw_text, ' ', 19); raw_text[19] = 0;
                memset(shifted_text, ' ', 19); shifted_text[19] = 0;
                text_idx = 0;

                UI_DisplayClear();
                UI_PrintStringSmallBold("SIGNAL", 0, 0, 0);
                UI_PrintStringSmallNormal("Learning...", 0, 0, 2);
                ST7565_BlitFullScreen();
            }
        } else {
            if (moving_center < close_thr) {
                if (close_cnt < CLOSE_HOLD_BITS) close_cnt++;
            } else {
                close_cnt = 0;
            }

            if (close_cnt >= CLOSE_HOLD_BITS) {
                squelch_open = false;
                close_cnt = 0;
                open_cnt = 0;

                packet_active = false;
                minmax_inited = false;
                warmup_bits = 0;

                UI_DisplayClear();
                UI_PrintStringSmallBold("READY", 0, 0, 0);
                UI_PrintStringSmallNormal("Squelch Closed", 0, 0, 2);
                ST7565_BlitFullScreen();
            }
        }

        // ----------------- READY debug -----------------
        if (!squelch_open)
        {
            ui_tick++;
            if (ui_tick >= UI_READY_DIV) {
                ui_tick = 0;
                int32_t d = val - moving_center;

                snprintf(dbg1, sizeof(dbg1), "i:%ld c:%ld", (long)idle_baseline, (long)moving_center);
                snprintf(dbg2, sizeof(dbg2), "v:%ld d:%ld", (long)val, (long)d);
                snprintf(dbg3, sizeof(dbg3), "thO:%ld thC:%ld", (long)open_thr, (long)close_thr);

                UI_DisplayClear();
                UI_PrintStringSmallBold("READY", 0, 0, 0);
                UI_PrintStringSmallNormal(dbg1, 0, 0, 2);
                UI_PrintStringSmallNormal(dbg2, 0, 0, 4);
                UI_PrintStringSmallNormal(dbg3, 0, 0, 6);
                ST7565_BlitFullScreen();
            }

            SYSTICK_DelayUs(SAMPLE_US);
            continue;
        }

        // ----------------- Accumulate for bit mean -----------------
        bit_sum += val;
        samples_in_bit++;

        if (samples_in_bit >= SAMPLES_PER_BIT)
        {
            samples_in_bit = 0;
            bit_mean = bit_sum / (int32_t)SAMPLES_PER_BIT;
            bit_sum = 0;

            // update leaky min/max
            if (!minmax_inited) {
                bit_min = bit_mean;
                bit_max = bit_mean;
                minmax_inited = true;
                warmup_bits = 0;
                goto bit_done;
            } else {
                if (bit_mean < bit_min) bit_min = bit_mean;
                else bit_min += (bit_mean - bit_min) / MINMAX_RELAX_DEN;

                if (bit_mean > bit_max) bit_max = bit_mean;
                else bit_max -= (bit_max - bit_mean) / MINMAX_RELAX_DEN;

                warmup_bits++;
            }

            if (warmup_bits < MINMAX_WARMUP_BITS) {
                goto bit_done;
            }

            int32_t thr = (bit_min + bit_max) / 2;
            bool tone_hi = (bit_mean > thr);
			last_thr = thr;
			last_m   = bit_mean;


            // --------------------------------------------------------
            // ก่อน lock: รัน 2 decoder คู่กัน
            //   A: transition => 0, same => 1   (เดิม)
            //   B: transition => 1, same => 0   (invert NRZI)
            // --------------------------------------------------------
            if (!packet_active)
            {
                // ---- Decoder A ----
                uint8_t bitA = (tone_hi != last_tone_hi_A) ? 0 : 1;
                last_tone_hi_A = tone_hi;

                if (ones_A == 5) {
                    if (bitA == 0) { ones_A = 0; goto decA_done; }
                    ones_A = 0; // abort-ish
                    shift_A = 0;
                    flag_run_A = 0;
                    goto decA_done;
                }
                ones_A = bitA ? (ones_A + 1) : 0;
                shift_A >>= 1;
                if (bitA) shift_A |= 0x80;

                if (shift_A == 0x7E) {
                    if (flag_run_A < 255) flag_run_A++;
                } else {
                    flag_run_A = 0;
                }
decA_done:

                // ---- Decoder B (invert) ----
                uint8_t bitB = (tone_hi != last_tone_hi_B) ? 1 : 0;
                last_tone_hi_B = tone_hi;

                if (ones_B == 5) {
                    if (bitB == 0) { ones_B = 0; goto decB_done; }
                    ones_B = 0;
                    shift_B = 0;
                    flag_run_B = 0;
                    goto decB_done;
                }
                ones_B = bitB ? (ones_B + 1) : 0;
                shift_B >>= 1;
                if (bitB) shift_B |= 0x80;

                if (shift_B == 0x7E) {
                    if (flag_run_B < 255) flag_run_B++;
                } else {
                    flag_run_B = 0;
                }
decB_done:

                // ถ้า A หรือ B เจอ flags พอ -> lock
                if (flag_run_A >= FLAGS_TO_LOCK || flag_run_B >= FLAGS_TO_LOCK)
                {
                    packet_active = true;
                    use_invert = (flag_run_B >= FLAGS_TO_LOCK);
                    bit_counter_since_lock = 0;

                    // init active shift/ones ตาม mode ที่เลือก
                    if (!use_invert) {
                        // ใช้ A
                        // (shift_A ตอนนี้เป็น 0x7E พอดี)
                    } else {
                        // ใช้ B
                        // (shift_B ตอนนี้เป็น 0x7E พอดี)
                    }

                    memset(raw_text, ' ', 19); raw_text[19] = 0;
                    memset(shifted_text, ' ', 19); shifted_text[19] = 0;
                    text_idx = 0;
                    ui_div = 0;

                    UI_DisplayClear();
                    UI_PrintStringSmallBold("SYNC LOCKED!", 0, 0, 0);
                    UI_PrintStringSmallNormal(use_invert ? "Mode: INV" : "Mode: NORM", 0, 0, 2);
                    ST7565_BlitFullScreen();
                    SYSTEM_DelayMs(200);
                }
				
				// ---- UI debug ตอน Learning (อัปเดต ~10Hz) ----
				ui_tick++;
				if (ui_tick >= 120) { // 1200bps / 120 ≈ 10Hz (เพราะมาในจังหวะต่อบิตแล้ว)
					ui_tick = 0;

					// ค่าหลักที่อยากเห็น
					snprintf(dbg1, sizeof(dbg1), "m:%ld t:%ld", (long)last_m, (long)last_thr);
					snprintf(dbg2, sizeof(dbg2), "min:%ld max:%ld", (long)bit_min, (long)bit_max);
					snprintf(dbg3, sizeof(dbg3), "A:%u B:%u", (unsigned)flag_run_A, (unsigned)flag_run_B);

					UI_DisplayClear();
					UI_PrintStringSmallBold("SIGNAL", 0, 0, 0);
					UI_PrintStringSmallNormal("Learning...", 0, 0, 1);
					UI_PrintStringSmallNormal(dbg1, 0, 0, 3);
					UI_PrintStringSmallNormal(dbg2, 0, 0, 5);
					UI_PrintStringSmallNormal(dbg3, 0, 0, 6);
					ST7565_BlitFullScreen();
				}


                goto bit_done;
            }

            // --------------------------------------------------------
            // หลัง lock: decode ตาม mode ที่เลือกอันเดียว
            // --------------------------------------------------------

                uint8_t decoded_bit;
                static bool last_tone_hi_sel = false;
                static uint8_t shift_sel = 0;
                static int ones_sel = 0;

                // init static ครั้งแรกหลัง lock
                if (bit_counter_since_lock == 0) {
                    last_tone_hi_sel = tone_hi;
                    shift_sel = 0;
                    ones_sel = 0;
                }

                decoded_bit = use_invert
                    ? ((tone_hi != last_tone_hi_sel) ? 1 : 0)
                    : ((tone_hi != last_tone_hi_sel) ? 0 : 1);

                last_tone_hi_sel = tone_hi;

                // bitstuff
                if (ones_sel == 5) {
                    if (decoded_bit == 0) { ones_sel = 0; goto bit_done; }
                    // abort-ish -> drop lock
                    packet_active = false;
                    goto bit_done;
                }
                ones_sel = decoded_bit ? (ones_sel + 1) : 0;

                // shift in
                shift_sel >>= 1;
                if (decoded_bit) shift_sel |= 0x80;

                bit_counter_since_lock++;
                if (bit_counter_since_lock > PACKET_TIMEOUT_BITS) {
                    packet_active = false;
                    goto bit_done;
                }

                // flag inside packet: skip storing
                if (shift_sel == 0x7E) {
                    goto bit_done;
                }

                // store debug chars
                if (text_idx >= 16) {
                    text_idx = 0;
                    memset(raw_text, ' ', 19); raw_text[19] = 0;
                    memset(shifted_text, ' ', 19); shifted_text[19] = 0;

                    UI_DisplayClear();
                    UI_PrintStringSmallBold("RECEIVING...", 0, 0, 0);
                    UI_PrintStringSmallNormal(use_invert ? "Mode: INV" : "Mode: NORM", 0, 0, 1);
                }

                char c_raw = (char)shift_sel;
                raw_text[text_idx] = ((uint8_t)c_raw >= 32 && (uint8_t)c_raw <= 126) ? c_raw : '.';
                shifted_text[text_idx] = '.';
                text_idx++;

                // draw slow
                ui_div++;
                if (ui_div >= UI_RX_DIVBITS) {
                    ui_div = 0;

                    // debug สั้นๆ ให้รู้ว่าค่า mean/thr วิ่งไหม
                    snprintf(dbg1, sizeof(dbg1), "m:%ld t:%ld", (long)bit_mean, (long)thr);

                    UI_PrintStringSmallNormal(raw_text, 0, 0, 2);
                    UI_PrintStringSmallNormal(dbg1, 0, 0, 6);
                    ST7565_BlitFullScreen();
                }


bit_done:
            ;
        }

        SYSTICK_DelayUs(SAMPLE_US);
    }
}




 
void SubApp_APRS_Menu(void) {
    int cursor = 0;
	int MAX_ITEMS = 4;
	
	// ตัวแปรเก็บค่าชั่วคราว (static เพื่อจำค่าล่าสุดไว้ ไม่ต้องพิมพ์ใหม่ทุกรอบ)
    static char target_call[10] = ""; 
    static char msg_text[40]    = "";
	
	// 1. ประกาศตัวนับเวลาสำหรับอ่าน GPS
    uint32_t gps_update_timer = 0;
	
    WaitForRelease();
    while(1) {
        UI_DisplayClear();
        UI_PrintStringSmallBold("APRS ACTIONS", 0, 0, 0);
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);

		UI_PrintStringSmallBold(cursor==0 ? "> SETTINGS"    : "  SETTINGS",    0, 0, 2);
        UI_PrintStringSmallBold(cursor==1 ? "> SEND BEACON" : "  SEND BEACON", 0, 0, 3);
		UI_PrintStringSmallBold(cursor==2 ? "> SEND MSG"    : "  SEND MSG",    0, 0, 4);
		//UI_PrintStringSmallBold(cursor==3 ? "> RX MONITOR" : "  RX MONITOR", 0, 0, 5);
        //UI_PrintStringSmallBold(cursor==2 ? "> VIEW LOG"    : "  VIEW LOG",    0, 0, 4);
		//UI_PrintStringSmallBold(cursor==3 ? "> RX DECODER" : "  RX DECODER", 0, 0, 5);
		//UI_PrintStringSmallBold(cursor==4 ? "> RX TEST LAB" : "  RX TEST LAB", 0, 0, 6);
		//UI_PrintStringSmallBold(cursor==4 ? "> RX SCOPE" : "  RX SCOPE", 0, 0, 6);
		UI_PrintStringSmallBold(cursor==3 ? "> SMART B. STATUS" : "  SMART B. STATUS", 0, 0, 5); // เพิ่มบรรทัดนี้

        ST7565_BlitFullScreen();
		
		// -------------------------------------------------------------
        // 2. แทรกโค้ดแอบอ่าน GPS ตรงนี้ (เหมือนในหน้าเมนูหลัก)
        // -------------------------------------------------------------
        // อัปเดตทุกๆ ~2 วินาที (100 รอบ x 20ms) เพื่อไม่ให้หน่วงเครื่องมากเกินไป
        if (gps_update_timer > 100) {
            int dummy = 0;
            // อ่านแบบ silent=true (ไม่โชว์ Progress Bar, แอบอ่านเงียบๆ)
            GPS_Fetch_Data(true, &dummy); 
            gps_update_timer = 0;
        }
        gps_update_timer++;
        // -------------------------------------------------------------
		
        KEY_Code_t key = KEYBOARD_Poll();
		
		HandleBacklight(key);
        
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        if (key == KEY_UP)   { cursor--; if(cursor<0) cursor=MAX_ITEMS - 1; WaitForRelease(); }
        if (key == KEY_DOWN) { cursor++; if(cursor>MAX_ITEMS-1) cursor=0; WaitForRelease(); }
        
        if (key == KEY_MENU) {
            WaitForRelease();
            if (cursor == 0) {
                SubApp_APRS_Settings(); // เข้าหน้าตั้งค่า
            }
            if (cursor == 1) {
                APRS_SendBeacon_Now(); // สั่งส่งทันที
                WaitForRelease();
            }
			// *** เพิ่ม Logic ส่งข้อความ ***
            if (cursor == 2) {
                // ขั้นตอนที่ 1: กรอก Callsign ปลายทาง
                if (EditString("DEST CALL", target_call, 9, false)) {
                    // ถ้ากรอกเสร็จและไม่ใช่ค่าว่าง
                    if (strlen(target_call) > 0) {
                        // ขั้นตอนที่ 2: กรอกข้อความ
                        if (EditString("MESSAGE", msg_text, 35, false)) {
                            // ส่งข้อความจริง
                            APRS_Send_Message(target_call, msg_text);
                            WaitForRelease();
                        }
                    }
                }
            }
			/* if (cursor == 3) {
				SubApp_APRS_RX();
			} */
			/* if (cursor == 3) {
				SubApp_APRS_RX_RealDecoder();
			} */
/* 			if (cursor == 4) {
				SubApp_APRS_RX_Test(); // เรียกใช้ห้องทดลอง
			} */
			//if (cursor == 4) SubApp_RX_Scope();
			if (cursor == 3) {
				SubApp_SmartBeacon_Status();
			}
        }
        
        // อย่าลืมใส่ Task นี้เพื่อให้ Beacon อัตโนมัติทำงานแม้จะอยู่ในหน้านี้
        APRS_Task(); 
		
		// อย่าลืม delay เพื่อให้ Timer ทำงานสัมพันธ์กับเวลาจริง
		SYSTEM_DelayMs(20);
    }
}

// ==========================================
// 5. Existing Apps (GPS Info, Grid Calc, Main)
// ==========================================
// (ส่วน DrawGPSInfoContent, HandleNavEdit, SubApp_GPS_Info, SubApp_GridCalc ผมย่อไว้เหมือนเดิม ท่านก๊อปของเก่ามาได้ แต่ต้องมั่นใจว่าไม่มี APRS_Settings_t เก่าหลงเหลือ)
// เพื่อให้ชัวร์ ผมใส่ตัวหลักๆ ให้

void DrawGPSInfoContent(int page) {
	char buff[32];
	
    if (page == 0) {
        UI_PrintStringSmallBold("GPS INFO  (1/3)", 0, 0, 0);
		UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        UI_PrintStringSmallBold("Lat:", 0, 0, 2); UI_PrintStringSmallNormal(val_lat, 38, 0, 2);
        UI_PrintStringSmallBold("Lon:", 0, 0, 3); UI_PrintStringSmallNormal(val_lon, 38, 0, 3);
        UI_PrintStringSmallBold("Alt:", 0, 0, 4); 
        char alt_buff[16]; sprintf(alt_buff, "%s m", val_alt); UI_PrintStringSmallNormal(alt_buff, 38, 0, 4);
        UI_PrintStringSmallBold("Grid:", 0, 0, 5); UI_PrintStringSmallNormal(val_grid, 38, 0, 5); 
        UI_PrintStringSmallBold("Sats:", 0, 0, 6); UI_PrintStringSmallNormal(val_sats, 38, 0, 6);
    } 
    else if (page == 1) {
        UI_PrintStringSmallBold("NAVIGATION  (2/3)", 0, 0, 0);
		UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        
        
        UI_PrintStringSmallBold("Lat Dst:", 0, 0, 2); UI_PrintStringSmallNormal(val_lat_dst, 64, 0, 2);
        UI_PrintStringSmallBold("Lon Dst:", 0, 0, 3); UI_PrintStringSmallNormal(val_lon_dst, 57, 0, 3);
        
        float lat1 = my_atof(val_lat);
        float lon1 = my_atof(val_lon);
        float lat2 = my_atof(val_lat_dst);
        float lon2 = my_atof(val_lon_dst);
        float dist, bear;
        CalculateDistanceBearing(lat1, lon1, lat2, lon2, &dist, &bear);
        
        sprintf(buff, "Dir To Dst: %d deg", (int)bear); UI_PrintStringSmallBold(buff, 0, 0, 4);
		//UI_PrintStringSmallBold("Dir To Dst:", 0, 0, 4); sprintf(buff, "%d", (int)bear); UI_PrintStringSmallNormal(buff, 78, 0, 4); UI_PrintStringSmallBold("deg", 106, 0, 4);
        int d_int = (int)dist; int d_dec = (int)((dist - d_int)*10);
        sprintf(buff, "Dist: %d.%d km", d_int, d_dec); UI_PrintStringSmallBold(buff, 0, 0, 5);
		//UI_PrintStringSmallBold("Distance:", 0, 0, 5); sprintf(buff, "%d.%d", d_int, d_dec); UI_PrintStringSmallNormal(buff, 63, 0, 5); UI_PrintStringSmallBold("km", 113, 0, 5);

        UI_PrintStringSmallBold("Spd:", 0, 0, 6); UI_PrintStringSmallNormal(val_speed, 30, 0, 6); 
        UI_PrintStringSmallBold("Dir:", 64, 0, 6); UI_PrintStringSmallNormal(val_course, 100, 0, 6);

        if (is_editing_nav) {
            int len_lat = strlen(val_lat_dst);
            int cursor_x = 0; int cursor_y = 0;
            if (nav_cursor < len_lat) {
                cursor_x = 65 + (nav_cursor * 7); cursor_y = 14;
            } else {
                cursor_x = 58 + ((nav_cursor - len_lat) * 7); cursor_y = 31;
            }
            UI_DrawRectangleBuffer(gFrameBuffer, cursor_x, cursor_y, cursor_x + 5, cursor_y, true);
        }
    }
    else {
        UI_PrintStringSmallBold("DATE/TIME  (3/3)", 0, 0, 0);
		UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
		//sprintf(buff, "Date: %s", val_date); UI_PrintStringSmallBold(buff, 38, 0, 3);
        UI_PrintStringSmallBold("Date:", 0, 0, 3); UI_PrintStringSmallNormal(val_date, 38, 0, 3);
		//sprintf(buff, "UTC Time: %s", val_time); UI_PrintStringSmallBold(buff, 65, 0, 5);
        UI_PrintStringSmallBold("UTC Time:", 0, 0, 5); UI_PrintStringSmallNormal(val_time, 65, 0, 5);
    }
}

// ฟังก์ชันจัดการปุ่มกดสำหรับหน้า NAV (Edit Mode)
void HandleNavEdit(KEY_Code_t key) {
    if (key == KEY_MENU) {
        int len_lat = strlen(val_lat_dst);
        int len_lon = strlen(val_lon_dst);
        int total_len = len_lat + len_lon;
        nav_cursor++; if (nav_cursor >= total_len) nav_cursor = 0;
        WaitForRelease();
    }
    else if (key == KEY_UP || key == KEY_DOWN) {
        int len_lat = strlen(val_lat_dst);
        char *target_str; int idx;
        if (nav_cursor < len_lat) { target_str = val_lat_dst; idx = nav_cursor; } 
        else { target_str = val_lon_dst; idx = nav_cursor - len_lat; }
        
        char c = target_str[idx];
        
        // Cycle: ' ' <-> '-' <-> '.' <-> '0'...'9'
        if (key == KEY_DOWN) {
            if (c == ' ') c = '-';
            else if (c == '-') c = '.';
            else if (c == '.') c = '0';
            else if (c >= '0' && c < '9') c++;
            else if (c == '9') c = ' ';
            else c = '0'; 
        } else {
            if (c == ' ') c = '9';
            else if (c > '0' && c <= '9') c--;
            else if (c == '0') c = '.';
            else if (c == '.') c = '-';
            else if (c == '-') c = ' ';
            else c = '0';
        }
        target_str[idx] = c;
        volatile uint32_t d = 600000; while(d--);
    }
}

// ฟังก์ชันอ่าน GPS แบบครอบจักรวาล (อ่าน + เช็คปุ่ม + วาดจอ)
// silent: true = แอบอ่านเงียบๆ (ใช้ในหน้าเมนู), false = โชว์ Progress Bar (ใช้ในหน้า GPS Info)
// page: ตัวแปรเก็บหน้าปัจจุบัน (Pointer) เพื่อให้กดเปลี่ยนหน้าได้ระหว่างโหลด
// Returns: false ถ้ากด Exit, true ถ้าทำงานจบ
// ฟังก์ชันอ่าน GPS แบบครอบจักรวาล (แก้ไขเรื่องเครื่องค้างตอนไม่เสียบสาย)

bool GPS_Fetch_Data(bool silent, int *page) {
    memset(rx_buffer, 0, RX_BUF_SIZE);
    int idx = 0;
    int max_len = 120;
	//int max_len = 99;
    bool complete = false;
    
    // ตัวแปรนับจำนวนครั้งที่ติดต่อไม่ได้
    int fail_counter = 0; 

    while (idx < max_len) {
        // ---------------------------------------------------------
        // A. ส่วนแทรก: เช็คปุ่มกดระหว่างรอ UART (เฉพาะโหมดไม่เงียบ)
        // ---------------------------------------------------------
        if (!silent && page != NULL) {
            KEY_Code_t key = KEYBOARD_Poll();
			
			HandleBacklight(key);
			
            if (key != KEY_INVALID) {
                if (key == KEY_EXIT) {
                    if (is_editing_nav) { is_editing_nav = false; WaitForRelease(); } 
                    else { return false; } // User กดออก -> return false
                }
                
                bool changed = false;
                if (is_editing_nav && *page == 1) { 
                    HandleNavEdit(key); changed = true; 
                } else {
                    if (key == KEY_DOWN) { (*page)++; if (*page > 2) *page = 0; changed = true; }
                    if (key == KEY_UP)   { (*page)--; if (*page < 0) *page = 2; changed = true; }
                    if (key == KEY_MENU && *page == 1) { is_editing_nav = true; nav_cursor = 0; changed = true; }
                }
                
                if (changed) {
                    WaitForRelease();
                    UI_DisplayClear(); DrawGPSInfoContent(*page); ST7565_BlitFullScreen();
                }
            }
            if (!is_editing_nav && (idx % 5 == 0)) {
                UI_DisplayClear(); DrawGPSInfoContent(*page); DrawProgressBar((idx * 100) / max_len); ST7565_BlitFullScreen();
            }
        }

        // ---------------------------------------------------------
        // B. ส่วน UART (แก้ไขป้องกันค้าง)
        // ---------------------------------------------------------
/*         UART_SendByte((uint8_t)idx);
        
        volatile uint32_t t = 20000;
        char c = 0; bool g = false;
        while (t > 0) { 
            t--; 
            if (UART1->SR & USART_SR_RXNE) { 
                c = (char)(UART1->DR & 0xFF); 
                g = true; 
                break; 
            } 
        }
        
        if (g) {
            // ถ้าได้รับข้อมูล: รีเซ็ตตัวนับความล้มเหลว และไปต่อ
            fail_counter = 0; 
            
            if ((c >= '0' && c <= '9') || c == '.' || c == ',' || c == '[' || c == ']' || c == ':' || c == '-' || c == '/' || c == ' ') {
                rx_buffer[idx] = c;
                if (c == ']') { rx_buffer[idx+1] = '\0'; complete = true; break; }
                idx++;
            }
        } else {
            // ถ้าเงียบ (Timeout): ให้นับเพิ่ม
            fail_counter++;
            
            // *** จุดแก้: ถ้าเงียบติดต่อกันเกิน 50 ครั้ง ให้เลิกพยายาม ***
            // (ป้องกัน Infinite Loop)
            if (fail_counter > 50) {
                //break; // หลุดจาก Loop ทันที (ถือว่า GPS ไม่ต่ออยู่)
                // แทนที่จะ break ให้ Reset ทุกอย่างเพื่อเริ่มใหม่
                idx = 0;               // กลับไปเริ่มตำแหน่งที่ 0
                fail_counter = 0;      // รีเซ็ตตัวนับความล้มเหลว
                memset(rx_buffer, 0, RX_BUF_SIZE); // ล้างข้อมูลขยะใน buffer ทิ้ง
                
                // Optional: หน่วงเวลานิดหน่อยเพื่อให้คลื่นกวนหายไปก่อนจะเริ่มใหม่
                volatile uint32_t reset_delay = 5000; while(reset_delay--){}
                
                continue; // วนกลับไปเริ่ม while ใหม่ทันที (โดย idx เป็น 0)
            }
        } */
		
        UART_SendByte((uint8_t)idx);
        
        volatile uint32_t t = 20000;
        char c = 0; bool g = false;
        while (t > 0) { 
            t--; 
            if (UART1->SR & USART_SR_RXNE) { 
                c = (char)(UART1->DR & 0xFF); 
                g = true; 
                break; 
            } 
        }
        
        // ตัวแปรสำหรับเช็คว่ารอบนี้ "ผ่าน" หรือไม่
        bool packet_valid = false;

        if (g) {
            // ถ้าได้รับข้อมูล: เช็คก่อนว่าเป็นขยะไหม
            if ((c >= '0' && c <= '9') || c == '.' || c == ',' || c == '[' || c == ']' || c == ':' || c == '-' || c == '/' || c == ' ') {
                
                // *** ข้อมูลถูกต้อง ค่อยถือว่าสำเร็จ ***
                rx_buffer[idx] = c;
                if (c == ']') { rx_buffer[idx+1] = '\0'; complete = true; break; }
                idx++;
                
                fail_counter = 0; // รีเซ็ตเฉพาะตอนที่ได้ของดี
                packet_valid = true;
            }
        }
        
        // ถ้าไม่ได้รับข้อมูล (Timeout) หรือ ได้รับแต่เป็นขยะ (packet_valid ยังเป็น false)
        if (!packet_valid) {
            fail_counter++; // นับความล้มเหลวเพิ่ม
            
            // ถ้าล้มเหลวติดต่อกันเกิน 50 ครั้ง (ไม่ว่าจะเงียบ หรือ ขยะล้วนๆ)
            if (fail_counter > 50) {
                idx = 0;               // กลับไปเริ่มใหม่
                fail_counter = 0;      
                memset(rx_buffer, 0, RX_BUF_SIZE);
                
                volatile uint32_t reset_delay = 5000; while(reset_delay--){}
                //continue;
				break; // หลุดจาก Loop ทันที (ถือว่า GPS ไม่ต่ออยู่)
            }
        }
        
        volatile uint32_t delay = 1100; while (delay--) {}
    }

    // ---------------------------------------------------------
    // C. ส่วน Parsing (แกะข้อมูล) - จะทำก็ต่อเมื่อข้อมูลครบถ้วนเท่านั้น
    // ---------------------------------------------------------
    if (complete && rx_buffer[0] == '[') {
         char *p = rx_buffer + 1, *comma;
         comma = strchr(p, ','); if(comma) { *comma=0; strcpy(val_lat, p); p=comma+1; }
         comma = strchr(p, ','); if(comma) { *comma=0; strcpy(val_lon, p); p=comma+1; }
         comma = strchr(p, ','); if(comma) { *comma=0; strcpy(val_alt, p); p=comma+1; }
         comma = strchr(p, ','); if(comma) { *comma=0; strcpy(val_sats, p); p=comma+1; }
         comma = strchr(p, ','); if(comma) { *comma=0; strcpy(val_speed, p); p=comma+1; }
         comma = strchr(p, ','); if(comma) { *comma=0; strcpy(val_course, p); p=comma+1; }
         comma = strchr(p, ','); if(comma) { *comma=0; strcpy(val_time, p); p=comma+1; }
         char *b = strchr(p, ']'); if(b) { *b=0; strcpy(val_date, p); }
         CalculateGrid(); 
    }
    
    // return true เพื่อบอกว่าฟังก์ชันทำงานจบแล้ว (ไม่ได้ถูก user สั่งยกเลิก)
    // หน้าจอ GPS Info จะได้ไม่เด้งออก
    return true; 
}

void SubApp_GPS_Info(void) {
    SetupUART(); 
    int current_page = 0; 
    WaitForRelease(); 
    
    while (1) {
        // เรียกฟังก์ชันอ่าน GPS (silent=false เพื่อโชว์กราฟิก)
        // ถ้าฟังก์ชัน return false แปลว่ากด Exit -> ให้ออกเลย
        if (!GPS_Fetch_Data(false, &current_page)) {
             WaitForRelease();
             return;
        }

        // Loop แสดงผลนิ่งๆ หลังโหลดเสร็จ (Refresh Rate)
        for (int t = 0; t < 5; t++) {
            // เช็คปุ่มอีกทีเผื่อกดตอนนิ่ง
            KEY_Code_t key = KEYBOARD_Poll();
            if (key == KEY_EXIT) { WaitForRelease(); return; }
			
			HandleBacklight(key);
            
            if (key == KEY_DOWN) {
				current_page++;
				if (current_page > 2) current_page = 0;
				WaitForRelease();
				UI_DisplayClear();
				DrawGPSInfoContent(current_page);
				ST7565_BlitFullScreen();
			}
            if (key == KEY_UP)   {
				current_page--;
				if (current_page < 0) current_page = 2;
				WaitForRelease();
				UI_DisplayClear();
				DrawGPSInfoContent(current_page);
				ST7565_BlitFullScreen(); 
			}

            APRS_Task();
            SYSTEM_DelayMs(20);
        }
        
        // วาดหน้าจอครั้งสุดท้ายให้ชัดเจน
        UI_DisplayClear(); DrawGPSInfoContent(current_page); ST7565_BlitFullScreen();
    }
}

/* void SubApp_GridCalc(void) {
    char my_grid_edit[7]; char dx_grid_edit[7] = "------";
    int cursor = 0; float my_lat, my_lon, target_lat, target_lon, dist, bear; char buff[32];
	
	// 1. เพิ่มตัวนับเวลา
    uint32_t gps_update_timer = 0;
	
    CalculateGrid(); strcpy(my_grid_edit, val_grid); 
    WaitForRelease();
    while (1) {
        UI_DisplayClear(); UI_PrintStringSmallBold("GRID DISTANCE", 0, 0, 0); UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        sprintf(buff, "MY: %s", my_grid_edit); UI_PrintStringSmallBold(buff, 0, 0, 2);
        sprintf(buff, "DX: %s", dx_grid_edit); UI_PrintStringSmallBold(buff, 0, 0, 3);
        int cursor_x = 29 + ((cursor < 6 ? cursor : cursor - 6) * 7);
        int cursor_y = (cursor < 6) ? 14 : 32;
        UI_DrawRectangleBuffer(gFrameBuffer, cursor_x, cursor_y, cursor_x + 5, cursor_y, true);

        int my_len = 0; for(int i=0;i<6;i++)if(my_grid_edit[i]!='-')my_len++;
        int dx_len = 0; for(int i=0;i<6;i++)if(dx_grid_edit[i]!='-')dx_len++;
        
        if (my_len >= 2 && my_len % 2 == 0 && dx_len >= 2 && dx_len % 2 == 0) {
            char t[7];
            strcpy(t, my_grid_edit); t[my_len]='\0'; GridToLatLon(t, &my_lat, &my_lon);
            strcpy(t, dx_grid_edit); t[dx_len]='\0'; GridToLatLon(t, &target_lat, &target_lon);
            CalculateDistanceBearing(my_lat, my_lon, target_lat, target_lon, &dist, &bear);
            int di = (int)dist; int dd = (int)((dist - di) * 10);
            sprintf(buff, "DST: %d.%d km", di, dd); UI_PrintStringSmallBold(buff, 0, 0, 5);
            sprintf(buff, "BRG: %d deg", (int)bear); UI_PrintStringSmallBold(buff, 0, 0, 6);
        } else { UI_PrintStringSmallBold("Enter 2,4,6 chars", 0, 0, 5); }

        ST7565_BlitFullScreen();
		
		// -------------------------------------------------------------
        // 2. แทรกโค้ดแอบอ่าน GPS
        // -------------------------------------------------------------
        if (gps_update_timer > 200) {
            int dummy = 0;
            GPS_Fetch_Data(true, &dummy); 
            // อัปเดต Grid ของเราใหม่ด้วย (เผื่อเดินไปเดินมาแล้ว Grid เปลี่ยน)
            CalculateGrid(); 
            // ถ้า Cursor ไม่ได้อยู่ที่ Grid เรา ให้ update ค่าที่โชว์ด้วย
            if (cursor >= 6) strcpy(my_grid_edit, val_grid);
            
            gps_update_timer = 0;
        }
        gps_update_timer++;
        // -------------------------------------------------------------
		
        APRS_Task();
		
		KEY_Code_t key = KEYBOARD_Poll();
		
		HandleBacklight(key);
		
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        if (key == KEY_MENU) { cursor++; if (cursor > 11) cursor = 0; WaitForRelease(); }
        if (key == KEY_UP || key == KEY_DOWN) {
            char *ag = (cursor < 6) ? my_grid_edit : dx_grid_edit;
            int idx = (cursor < 6) ? cursor : cursor - 6;
            char c = ag[idx];
            if (c == '-') { if (idx < 2) c = 'A'; else if (idx < 4) c = '0'; else c = 'a'; }
            if (key == KEY_DOWN) { c++; if (idx < 2 && c > 'R') c = 'A'; else if (idx >= 2 && idx < 4 && c > '9') c = '0'; else if (idx >= 4 && c > 'x') c = 'a'; }
            else { c--; if (idx < 2 && c < 'A') c = 'R'; else if (idx >= 2 && idx < 4 && c < '0') c = '9'; else if (idx >= 4 && c < 'a') c = 'x'; }
            ag[idx] = c;
            if (idx % 2 == 0 && ag[idx + 1] == '-') { if (idx < 2) ag[idx + 1] = 'A'; else if (idx < 4) ag[idx + 1] = '0'; else ag[idx + 1] = 'a'; }
            volatile uint32_t d = 600000; while (d--) {}
        }
		
		SYSTEM_DelayMs(20);
    }
} */



/* void SubApp_Settings(void) {
    int cursor = 0; const int NUM_ITEMS = 5; char buff[32]; WaitForRelease(); 
    while (1) {
        UI_DisplayClear(); UI_PrintStringSmallBold("APRS SETTINGS", 0, 0, 0); UI_PrintStringSmallBold("(Still working on it)", 0, 0, 1);
        sprintf(buff, "%c CALL:%s-%d", cursor==0?'>':' ', aprs_config.callsign, aprs_config.ssid); UI_PrintStringSmallBold(buff, 0, 0, 2);
        sprintf(buff, "%c PATH:%s", cursor==1?'>':' ', aprs_config.digipath); UI_PrintStringSmallBold(buff, 0, 0, 3);
        sprintf(buff, "%c ICON:%c%c", cursor==2?'>':' ', aprs_config.icon_table, aprs_config.icon_symbol); UI_PrintStringSmallBold(buff, 0, 0, 4);
        sprintf(buff, "%c TX: %d.%03d", cursor==3?'>':' ', (int)(aprs_config.tx_freq/100000), (int)((aprs_config.tx_freq/100)%1000)); UI_PrintStringSmallBold(buff, 0, 0, 5);
        char *pwr_str = (aprs_config.tx_power==0)?"LOW":(aprs_config.tx_power==1?"MID":"HIGH"); sprintf(buff, "%c PWR:%s", cursor==4?'>':' ', pwr_str); UI_PrintStringSmallBold(buff, 0, 0, 6);
        ST7565_BlitFullScreen(); KEY_Code_t key = KEYBOARD_Poll();
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        if (key == KEY_UP) { cursor--; if(cursor < 0) cursor = NUM_ITEMS-1; WaitForRelease(); }
        if (key == KEY_DOWN) { cursor++; if(cursor >= NUM_ITEMS) cursor = 0; WaitForRelease(); }
        if (key == KEY_MENU) { WaitForRelease(); if (cursor == 0) EditString("SET CALLSIGN", aprs_config.callsign, 7, false); else if (cursor == 4) { aprs_config.tx_power++; if(aprs_config.tx_power > 2) aprs_config.tx_power = 0; } }
    }
} */

void APP_RunGPS(void) {
	
	// 1. [สำคัญ] แบ็คอัพค่าเดิมของขา GPIOA ไว้ก่อน (Save Original State)
    // นี่คือค่าที่วิทยุใช้ปกติ (Keypad/I2C/etc.)
    uint32_t original_gpioa_moder = GPIOA->MODER;
    uint32_t original_gpioa_afrl = GPIOA->AFR[0];
    uint32_t original_gpioa_afrh = GPIOA->AFR[1];
	
	UI_DisplayClear();
	
	// 2. เรียกใช้ GPS (ซึ่งฟังก์ชันนี้จะไปเปลี่ยนค่าขา GPIOA เป็น UART)
    SetupUART();
	
    //NVIC_DisableIRQ(USART1_IRQn); 
    
    WaitForRelease(); 
    int menu_cursor = 0;
    
    uint32_t auto_update_timer = 0; // ตัวนับเวลา
    
    //isInitialized = true;

    while (1) {
        UI_DisplayClear(); 
        UI_PrintStringSmallBold("GPS & APRS", 0, 0, 0); 
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        
        //if (menu_cursor == 0) UI_PrintStringSmallBold("> GPS INFO", 3, 0, 2); else UI_PrintStringSmallNormal("  GPS INFO", 3, 0, 2);
        //if (menu_cursor == 1) UI_PrintStringSmallBold("> APRS MENU", 3, 0, 4); else UI_PrintStringSmallNormal("  APRS MENU", 3, 0, 4);
		//if (menu_cursor == 2) UI_PrintStringSmallBold("> GRID DISTANCE", 3, 0, 6); else UI_PrintStringSmallNormal("  GRID DISTANCE", 3, 0, 6);
		UI_PrintStringSmallBold(menu_cursor==0 ? "> GPS INFO" : "  GPS INFO", 3, 0, 2);
        UI_PrintStringSmallBold(menu_cursor==1 ? "> APRS MENU"    : "  APRS MENU",    3, 0, 4);
        //UI_PrintStringSmallBold(menu_cursor==2 ? "> GRID DISTANCE"    : "  GRID DISTANCE",    3, 0, 6);
        
        // โชว์สถานะดาวเทียมมุมจอ (เพื่อให้รู้ว่ามีข้อมูลเข้า)
        //char sat_buff[10]; sprintf(sat_buff, "Sats:%s", val_sats);
        //UI_PrintStringSmallNormal(sat_buff, 75, 0, 0);

        ST7565_BlitFullScreen();
        
        // ---------------------------------------------------------
        // *** AUTO UPDATE GPS ***
        // ---------------------------------------------------------
        // อัปเดตทันทีที่เข้า (timer=0) หรือทุกๆ ~4-5 วินาที (timer>200)
        if (auto_update_timer == 0 || auto_update_timer > 100) {
            int dummy_page = 0;
            // เรียกแบบ silent=true (แอบโหลด ไม่วาดจอ)
            GPS_Fetch_Data(true, &dummy_page); 
            auto_update_timer = 1; 
        }
        auto_update_timer++;
        // ---------------------------------------------------------

        // APRS Task ส่ง Beacon อัตโนมัติ (จะใช้พิกัดล่าสุดที่เพิ่งโหลดมา)
        APRS_Task();
        
        KEY_Code_t key = KEYBOARD_Poll();
		
		HandleBacklight(key);
		
        if (key == KEY_EXIT) { 
		
			// 3. [สำคัญ] คืนค่าเดิมก่อนออก (Restore Original State)
            
            // 1. ปิดการทำงานของ UART Hardware
            UART1->CR1 = 0;
			
			// 2. *** สำคัญ *** ปิด Interrupt ของ UART1 เพื่อกันไม่ให้ GPS กวนการทำงานของวิทยุหลัก
            NVIC_DisableIRQ(USART1_IRQn);
			
			// *** คืนค่าขา GPIOA ให้กลับเป็นเหมือนตอนก่อนเข้าแอพ ***
            GPIOA->MODER   = original_gpioa_moder;
            GPIOA->AFR[0]  = original_gpioa_afrl;
            GPIOA->AFR[1]  = original_gpioa_afrh;
			
			// 3. เปิด Global Interrupt คืน (เผื่อถูกปิดไว้)
			__enable_irq(); 
			
			// 4. สั่งเคลียร์หน้าจอก่อนออก เพื่อความสะอาดตา
			UI_DisplayClear();
			
			return; 
		}
        if (key == KEY_UP) { menu_cursor--; if (menu_cursor < 0) menu_cursor = 1; WaitForRelease(); }
        if (key == KEY_DOWN) { menu_cursor++; if (menu_cursor > 1) menu_cursor = 0; WaitForRelease(); }
        if (key == KEY_MENU) { 
            WaitForRelease();
            if (menu_cursor == 0) SubApp_GPS_Info(); 
            else if (menu_cursor == 1) SubApp_APRS_Menu();
            //else if (menu_cursor == 2) SubApp_GridCalc();
            WaitForRelease(); 
        }
        
        SYSTEM_DelayMs(20);
    }
}

#include <sys/stat.h>
int _close(int file) { return -1; }
int _fstat(int file, struct stat *st) { return 0; }
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _read(int file, char *ptr, int len) { return 0; }
int _write(int file, char *ptr, int len) { return len; }
void *_sbrk(int incr) { extern char _end; static unsigned char *heap = 0; unsigned char *prev_heap; if (heap == 0) { heap = (unsigned char *)&_end; } prev_heap = heap; heap += incr; return (void *)prev_heap; }
