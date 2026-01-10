#include "py32f0xx.h"
#include "gps_app.h"
#include <string.h>
#include <stdio.h>
// #include <math.h> // ไม่ใช้ math.h เพื่อประหยัดเมมโมรี่
#include "ui/helper.h"
#include "driver/st7565.h"
#include "driver/keyboard.h"
#include "driver/system.h"
#include "driver/uart.h"

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

// กำหนดค่าเริ่มต้น (Default Settings)
APRS_Config_t aprs_config = {
    .aprs_on = false,
    .callsign = "E25WOP",
    .ssid = 7,
    .dest_call = "APUVK1",
    .digipath = "WIDE1-1",
    .icon_table = '/',
    .icon_symbol = '[', // รูปคนวิ่ง
    .comment = "APRS from QuanSheng UV-K1",
    .tx_freq = 144390000,
    .tx_power = 1,      // MID
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
    .preamble = 150,
	.bl_time = 0, // *** เพิ่มค่าเริ่มต้น: 0 (เปิดตลอด) หรือจะใส่ 10, 20 ก็ได้ ***
    
    // Smart Beacon Defaults
    //.sb_conf = { 30, 600, 5, 80, 28 } 
};

// ตัวแปรนับเวลาไฟจอ (Global)
static uint32_t bl_timer_count = 0;

// ตัวแปรสำหรับจับเวลาส่งอัตโนมัติ
//static uint32_t last_beacon_time = 0;

// ประกาศ Prototype ฟังก์ชันที่อยู่ด้านล่าง (เพื่อแก้ Error implicit declaration)
bool EditString(char *title, char *buffer, int max_len, bool numeric_only);
void SubApp_APRS_Settings(void);
void SubApp_GridCalc(void);
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

extern char val_alt[10];
extern char val_speed[10];
extern char val_course[10];

extern void BACKLIGHT_TurnOn(void);
extern void BACKLIGHT_TurnOff(void);

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
static uint32_t aprs_timer_counter = 0;

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

void UART_SendByte(uint8_t data) {
    while (!(UART1->SR & USART_SR_TXE)) {}
    UART1->DR = data;
    while (!(UART1->SR & USART_SR_TC)) {}
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
    
    // ตั้งค่า cursor ไปที่ท้ายข้อความเสมอเมื่อเริ่ม
    cursor = len;
    
    WaitForRelease(); 
    
    while (1) {
        UI_DisplayClear(); 
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
        if (menu_pressed_once) {
            UI_PrintStringSmallBold("MENU AGAIN TO SAVE", 20, 0, 6);
        } else {
            // ย้ายขึ้นมาบรรทัด 5
            UI_PrintStringSmallBold("UP/DN:Char *:Del", 0, 0, 5);
            // อยู่บรรทัด 6 (Max สุดที่ปลอดภัย)
            UI_PrintStringSmallBold("MENU:Next  EXIT:Esc", 0, 0, 6);
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
            
            char c = buffer[cursor];
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
                if (key == KEY_DOWN) {
                    if (c == ' ') c = 'A';
                    else if (c >= 'A' && c < 'Z') c++;
                    else if (c == 'Z') c = '0';
                    else if (c >= '0' && c < '9') c++;
                    else if (c == '9') c = '-';
                    else if (c == '-') c = '.';
                    else if (c == '.') c = '/';
                    else if (c == '/') c = ':';
                    else if (c == ':') c = ' ';
                    else c = ' ';
                } else { 
                    if (c == ' ') c = ':';
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
            
            buffer[cursor] = c;
            if (cursor == len) { len++; buffer[len] = 0; }
            SYSTEM_DelayMs(150); 
        }
        
        SYSTEM_DelayMs(10);
    }
}

// ==========================================
// 4. APRS Menus
// ==========================================
void APRS_Task(void) {
    if (!aprs_config.aprs_on) return;
	
	// ถ้าตั้ง Interval เป็น 0 ถือว่าปิด Auto Send
    if (aprs_config.manual_interval <= 0) return;

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
    
    aprs_timer_counter++;
    
    // แปลงวินาทีเป็นรอบ (สมมติ rough estimate)
    //uint32_t target_count = aprs_config.manual_interval * 2000; 
	
	// จูนค่านี้ตามความเร็ว Loop เครื่อง (ลองที่ 4000 ก่อนครับ ถ้าช้าไปให้ลดเลขลง)
    // สมมติ Loop วิ่งที่ 2ms -> 1 วินาที = 500 รอบ 
    // แต่ถ้า Loop เร็วมาก อาจจะถึง 2000-4000 รอบต่อวิ
    //uint32_t loops_per_sec = 4000; 
	// จากการทดสอบจริง เครื่องวิ่งได้ประมาณ 85-100 รอบต่อวินาที (ในหน้า GPS Info)
	uint32_t loops_per_sec = 85; 
    uint32_t target_count = aprs_config.manual_interval * loops_per_sec;

    if (aprs_timer_counter > target_count) {
        aprs_timer_counter = 0;
        
        // ส่ง Beacon อัตโนมัติ
        APRS_SendBeacon_Now();
    }
}

void SubApp_APRS_Settings(void) {
    int cursor = 0;
    const int N_ITEMS = 14;
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
			
			if (item_idx == 0) sprintf(buff, "%c APRS: %s", marker, aprs_config.aprs_on ? "ON" : "OFF");
			// --- เมนูใหม่: INTERVAL ---
            else if (item_idx == 1) {
                if(aprs_config.manual_interval == 0)
                    sprintf(buff, "%c INTERVAL: OFF", marker);
                else
                    //sprintf(buff, "%c INTERVAL: %ds", marker, aprs_config.manual_interval);
				sprintf(buff, "%c INTERVAL: %d", marker, aprs_config.manual_interval);
            }
			// --- เมนูใหม่: TX FREQ (5 ทศนิยม) ---
            else if (item_idx == 2) {
                uint32_t mhz = aprs_config.tx_freq / 1000000;
                // เอาเศษ 6 หลัก แล้วหาร 10 เพื่อเหลือ 5 หลัก (หน่วย 10Hz)
                uint32_t decimals = (aprs_config.tx_freq % 1000000) / 10; 
                sprintf(buff, "%c FREQ: %lu.%05lu", marker, mhz, decimals);
            }
			
			// --- แก้ไขเมนู TX PWR ให้โชว์ L1-L5, MID, HIGH ---
            else if (item_idx == 3) {
                char pwr_str[10];
                if (aprs_config.tx_power < 5) sprintf(pwr_str, "L%d", aprs_config.tx_power + 1);
                else if (aprs_config.tx_power == 5) strcpy(pwr_str, "MID");
                else strcpy(pwr_str, "HIGH");
                
                sprintf(buff, "%c TX PWR: %s", marker, pwr_str);
            }
			// --- เมนูใหม่: PREAMBLE ---
            else if (item_idx == 4) {
                sprintf(buff, "%c PREAMBLE: %d", marker, aprs_config.preamble);
            }
            //else if (item_idx == 5) sprintf(buff, "%c MY: %s-%d", marker, aprs_config.callsign, aprs_config.ssid);
			// ของใหม่: เช็คก่อนว่ามี SSID ไหม
			else if (item_idx == 5) {
				if (aprs_config.ssid == 0)
					sprintf(buff, "%c MY: %s", marker, aprs_config.callsign);
				else
					sprintf(buff, "%c MY: %s-%d", marker, aprs_config.callsign, aprs_config.ssid);
			}
            else if (item_idx == 6) sprintf(buff, "%c TO: %s", marker, aprs_config.dest_call);
            // เพิ่มเมนู PATH ตรงนี้
            else if (item_idx == 7) sprintf(buff, "%c PATH: %s", marker, aprs_config.digipath); 
            else if (item_idx == 8) sprintf(buff, "%c SYM: %c%c", marker, aprs_config.icon_table, aprs_config.icon_symbol);
            else if (item_idx == 9) sprintf(buff, "%c POS: %s", marker, aprs_config.use_gps ? "GPS" : "FIXED");
            else if (item_idx == 10) sprintf(buff, "%c LAT: %s", marker, aprs_config.fixed_lat);
            else if (item_idx == 11) sprintf(buff, "%c LON: %s", marker, aprs_config.fixed_lon);

			/*}  else if (item_idx == 5) { // เมนูใหม่: แก้ Lat
                sprintf(buff, "%c LAT: %s", marker, aprs_config.fixed_lat);
            } else if (item_idx == 6) { // เมนูใหม่: แก้ Lon
                sprintf(buff, "%c LON: %s", marker, aprs_config.fixed_lon);
            } */
			// --- เมนูใหม่: COMMENT ---
			else if (item_idx == 12) {
                // ตัดข้อความให้สั้นลงถ้ามันยาวเกินจอ
                char short_cmt[8];
                strncpy(short_cmt, aprs_config.comment, 5);
                short_cmt[5] = '\0';
                if(strlen(aprs_config.comment) > 5) strcat(short_cmt, "..");
                
                sprintf(buff, "%c COMMENT: %s", marker, short_cmt);
            }
			// *** เพิ่มเมนูใหม่: BACKLIGHT ***
            else if (item_idx == 13) {
                if(aprs_config.bl_time == 0) sprintf(buff, "%c BL TIME: ON", marker);
                else sprintf(buff, "%c BL TIME: %ds", marker, aprs_config.bl_time);
            }
			
            UI_PrintStringSmallBold(buff, 0, 0, y_pos);
        }

        ST7565_BlitFullScreen();
        
        KEY_Code_t key = KEYBOARD_Poll();
		HandleBacklight(key); // *** เพิ่มบรรทัดนี้: เลี้ยงไฟจอในหน้าเมนู ***
		
        if (key == KEY_EXIT) { WaitForRelease(); return; }
        
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
            if (cursor == 0) aprs_config.aprs_on = !aprs_config.aprs_on;
			// --- เมนูใหม่: INTERVAL ---
            else if (cursor == 1) {
                // แปลงค่า int เป็น string เพื่อแก้ไข
                char tmp_buf[10];
                sprintf(tmp_buf, "%d", aprs_config.manual_interval);
                
                // เรียกใช้ EditString (numeric_only = true)
                //if(EditString("INTERVAL (Sec)", tmp_buf, 5, true)) {
				if(EditString("INTERVAL", tmp_buf, 5, true)) {
                    // แปลงกลับเป็น int (ใช้ my_atof แล้ว cast เป็น int)
                    aprs_config.manual_interval = (int)my_atof(tmp_buf);
                }
            }
			// --- เมนูใหม่: TX FREQ (5 ทศนิยม) ---
            else if (cursor == 2) {
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
            else if (cursor == 3) { 
                aprs_config.tx_power++; 
                if(aprs_config.tx_power > 6) aprs_config.tx_power = 0; // 0=L1 ... 6=HIGH
            }
			
			// --- เมนูใหม่: PREAMBLE ---
            else if (cursor == 4) {
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
			else if (cursor == 5) {
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
            else if (cursor == 6) EditString("DEST CALL", aprs_config.dest_call, 6, false);
            // cursor 3 (ICON) ข้ามไปก่อน หรือจะทำเพิ่มก็ได้
			// แก้ไข PATH
            else if (cursor == 7) EditString("PATH", aprs_config.digipath, 9, false);
			
			// --- เมนูเลือก ICON (SYM) แบบ Presets + Custom ---
            else if (cursor == 8) {
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
			
            else if (cursor == 9) aprs_config.use_gps = !aprs_config.use_gps;
            
            // เข้าเมนูแก้ไขพิกัด (ใช้ EditString ตัวเดิมที่มีอยู่)
            //if (cursor == 5) EditString("FIX LAT (DDMM.mmN)", aprs_config.fixed_lat, 9, false);
            //if (cursor == 6) EditString("FIX LON (DDDMM.mmE)", aprs_config.fixed_lon, 10, false);
			
			// อนุญาตให้กรอกแบบ Decimal (ใช้ตัวเลข, จุด, ลบ)
            else if (cursor == 10) EditString("LAT (Decimal)", aprs_config.fixed_lat, 10, true);
            else if (cursor == 11) EditString("LON (Decimal)", aprs_config.fixed_lon, 11, true);
            
			// --- เมนูใหม่: COMMENT ---
            else if (cursor == 12) {
                // ให้พิมพ์ได้สูงสุด 30 ตัวอักษร
                EditString("COMMENT", aprs_config.comment, 30, false);
            }
			// *** เพิ่มเงื่อนไขสำหรับเมนู 13 ***
            else if (cursor == 13) {
                char tmp_buf[10]; sprintf(tmp_buf, "%d", aprs_config.bl_time);
                if(EditString("BL TIME (Sec)", tmp_buf, 3, true)) {
                    aprs_config.bl_time = (int)my_atof(tmp_buf);
                }
            }
			
            WaitForRelease();
        }
        // ลดการหน่วงเวลา Loop หลักลงเพื่อให้หน้าจอตอบสนองไวขึ้น
        // volatile uint32_t d = 10000; while (d--) {}
    }
}

void SubApp_APRS_Menu(void) {
    int cursor = 0;
	
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
        //UI_PrintStringSmallBold(cursor==2 ? "> VIEW LOG"    : "  VIEW LOG",    0, 0, 4);

        ST7565_BlitFullScreen();
		
		// -------------------------------------------------------------
        // 2. แทรกโค้ดแอบอ่าน GPS ตรงนี้ (เหมือนในหน้าเมนูหลัก)
        // -------------------------------------------------------------
        // อัปเดตทุกๆ ~4 วินาที (200 รอบ x 20ms) เพื่อไม่ให้หน่วงเครื่องมากเกินไป
        if (gps_update_timer > 200) {
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
        if (key == KEY_UP)   { cursor--; if(cursor<0) cursor=2; WaitForRelease(); }
        if (key == KEY_DOWN) { cursor++; if(cursor>2) cursor=0; WaitForRelease(); }
        
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
        UI_PrintStringSmallBold("Dir:", 64, 0, 6); UI_PrintStringSmallNormal(val_course, 94, 0, 6);

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
                    if (key == KEY_DOWN) { (*page)++; if (*page > 1) *page = 0; changed = true; }
                    if (key == KEY_UP)   { (*page)--; if (*page < 0) *page = 1; changed = true; }
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
            
            if (key == KEY_DOWN) { current_page++; if (current_page > 2) current_page = 0; WaitForRelease(); UI_DisplayClear(); DrawGPSInfoContent(current_page); ST7565_BlitFullScreen(); }
            if (key == KEY_UP)   { current_page--; if (current_page < 0) current_page = 2; WaitForRelease(); UI_DisplayClear(); DrawGPSInfoContent(current_page); ST7565_BlitFullScreen(); }

            APRS_Task();
            SYSTEM_DelayMs(20);
        }
        
        // วาดหน้าจอครั้งสุดท้ายให้ชัดเจน
        UI_DisplayClear(); DrawGPSInfoContent(current_page); ST7565_BlitFullScreen();
    }
}

void SubApp_GridCalc(void) {
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
}



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
    NVIC_DisableIRQ(USART1_IRQn); 
    SetupUART(); // ต้อง Setup UART ด้วย
    WaitForRelease(); 
    int menu_cursor = 0;
    
    uint32_t auto_update_timer = 0; // ตัวนับเวลา

    while (1) {
        UI_DisplayClear(); 
        UI_PrintStringSmallBold("GPS & APRS", 0, 0, 0); 
        UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        
        //if (menu_cursor == 0) UI_PrintStringSmallBold("> GPS INFO", 3, 0, 2); else UI_PrintStringSmallNormal("  GPS INFO", 3, 0, 2);
        //if (menu_cursor == 1) UI_PrintStringSmallBold("> APRS MENU", 3, 0, 4); else UI_PrintStringSmallNormal("  APRS MENU", 3, 0, 4);
		//if (menu_cursor == 2) UI_PrintStringSmallBold("> GRID DISTANCE", 3, 0, 6); else UI_PrintStringSmallNormal("  GRID DISTANCE", 3, 0, 6);
		UI_PrintStringSmallBold(menu_cursor==0 ? "> GPS INFO" : "  GPS INFO", 3, 0, 2);
        UI_PrintStringSmallBold(menu_cursor==1 ? "> APRS MENU"    : "  APRS MENU",    3, 0, 4);
        UI_PrintStringSmallBold(menu_cursor==2 ? "> GRID DISTANCE"    : "  GRID DISTANCE",    3, 0, 6);
        
        // โชว์สถานะดาวเทียมมุมจอ (เพื่อให้รู้ว่ามีข้อมูลเข้า)
        //char sat_buff[10]; sprintf(sat_buff, "Sats:%s", val_sats);
        //UI_PrintStringSmallNormal(sat_buff, 75, 0, 0);

        ST7565_BlitFullScreen();
        
        // ---------------------------------------------------------
        // *** AUTO UPDATE GPS ***
        // ---------------------------------------------------------
        // อัปเดตทันทีที่เข้า (timer=0) หรือทุกๆ ~4-5 วินาที (timer>200)
        if (auto_update_timer == 0 || auto_update_timer > 200) {
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
		
        if (key == KEY_EXIT) { UART1->CR1 = 0; NVIC_EnableIRQ(USART1_IRQn); __enable_irq(); return; }
        if (key == KEY_UP) { menu_cursor--; if (menu_cursor < 0) menu_cursor = 2; WaitForRelease(); }
        if (key == KEY_DOWN) { menu_cursor++; if (menu_cursor > 2) menu_cursor = 0; WaitForRelease(); }
        if (key == KEY_MENU) { 
            WaitForRelease();
            if (menu_cursor == 0) SubApp_GPS_Info(); 
            else if (menu_cursor == 1) SubApp_APRS_Menu();
            else if (menu_cursor == 2) SubApp_GridCalc();
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
