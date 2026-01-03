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

// โครงสร้างสำหรับเก็บการตั้งค่า APRS
typedef struct {
    char callsign[10];
    int  ssid;
    char digipath[20];
    char icon_table;
    char icon_symbol;
    uint32_t tx_freq; 
    uint8_t tx_power; 
    int prewave_ms;   
    bool use_gps;     
    char fixed_lat[10]; 
    char fixed_lon[11];
} APRS_Settings_t;

APRS_Settings_t aprs_config = {
    .callsign = "E25WOP",
    .ssid = 7,
    .digipath = "WIDE1-1",
    .icon_table = '/',
    .icon_symbol = '[',
    .tx_freq = 144390000,
    .tx_power = 1,
    .prewave_ms = 300,
    .use_gps = true,
    .fixed_lat = "13.0000",
    .fixed_lon = "100.0000"
};

#define RX_BUF_SIZE 180 
static char rx_buffer[RX_BUF_SIZE] = ""; 

// ตัวแปรเก็บค่า GPS
static char val_lat[16]   = "13.7415";
static char val_lon[16]   = "100.4893";
static char val_alt[10]   = "0.0";
static char val_sats[5]   = "0";
static char val_speed[10] = "0";
static char val_course[10]= "0";
static char val_time[10]  = "-";
static char val_date[16]  = "-"; 

// ตัวแปรเก็บค่า Grid Locator ของเรา (จาก GPS)
static char val_grid[7] = "------";

// ==========================================
// 2. Custom Math Functions (Clean Format)
// ==========================================

// แปลง String เป็น Float
float my_atof(const char* s) {
    float rez = 0, fact = 1;
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

// Square Root (ปรับปรุงใหม่: วนลูป 20 รอบ เพื่อความแม่นยำระยะใกล้)
float my_sqrt(float n) {
    if (n <= 0) return 0;
    float x = n;
    // ถ้าค่าน้อยกว่า 1 ให้เริ่มเดาที่ 1 เพื่อให้ลู่เข้าหาค่าจริงได้เร็วกว่า
    if (x < 1.0f) x = 1.0f; 
    
    // วนลูป 20 รอบ (Newton-Raphson method)
    for (int i = 0; i < 20; i++) {
        x = 0.5f * (x + n / x);
    }
    return x;
}

// Cosine (Taylor Series)
float my_cos(float x) {
    while (x > PI) { x -= 2 * PI; }
    while (x < -PI) { x += 2 * PI; }
    float x2 = x * x;
    return 1.0f - (x2 / 2.0f) + (x2 * x2 / 24.0f) - (x2 * x2 * x2 / 720.0f);
}

// Sine (Taylor Series)
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

// Haversine Formula (Distance)
void CalculateDistanceBearing(float lat1, float lon1, float lat2, float lon2, float *dist, float *bearing) {
    float rLat1 = lat1 * DEG2RAD;
    float rLon1 = lon1 * DEG2RAD;
    float rLat2 = lat2 * DEG2RAD;
    float rLon2 = lon2 * DEG2RAD;
    
    float dLat = rLat2 - rLat1;
    float dLon = rLon2 - rLon1;

    // 1. Distance using Haversine
    float a = (my_sin(dLat / 2.0f) * my_sin(dLat / 2.0f)) +
              (my_cos(rLat1) * my_cos(rLat2) * my_sin(dLon / 2.0f) * my_sin(dLon / 2.0f));
    
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;
    
    float c = 2.0f * my_atan2(my_sqrt(a), my_sqrt(1.0f - a));
    *dist = R_EARTH * c;

    // 2. Bearing
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
    UART1->BRR = 0x1388; UART1->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
}

// *** แก้ไขฟังก์ชันนี้เพื่อแก้ Warning ***
void UART_SendByte(uint8_t data) {
    while (!(UART1->SR & USART_SR_TXE)) {
        // รอจนกว่า TXE (Transmit Empty) จะพร้อม
    }
    UART1->DR = data;
    while (!(UART1->SR & USART_SR_TC)) {
        // รอจนกว่า TC (Transmit Complete) จะเสร็จสมบูรณ์
    }
}

void DrawProgressBar(int percent) {
    if (percent < 0) percent = 0;
	if (percent > 100) percent = 100;
    int x_end = ((percent * 128) / 100) + 65;
    if (x_end > 0) UI_DrawRectangleBuffer(gFrameBuffer, 65, 54, x_end, 55, true);
}

// ==========================================
// 4. Sub-Apps (UI)
// ==========================================

void DrawGPSInfoContent(int page) {
    if (page == 0) {
        UI_PrintStringSmallBold("GPS INFO", 0, 0, 0); UI_PrintStringSmallNormal("(1/3)", 70, 0, 0);
		UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        UI_PrintStringSmallBold("Lat:", 0, 0, 2); UI_PrintStringSmallNormal(val_lat, 38, 0, 2);
        UI_PrintStringSmallBold("Lon:", 0, 0, 3); UI_PrintStringSmallNormal(val_lon, 38, 0, 3);
        UI_PrintStringSmallBold("Alt:", 0, 0, 4); 
        char alt_buff[16]; sprintf(alt_buff, "%s m", val_alt); UI_PrintStringSmallNormal(alt_buff, 38, 0, 4);
        UI_PrintStringSmallBold("Grid:", 0, 0, 5); UI_PrintStringSmallNormal(val_grid, 38, 0, 5); 
        UI_PrintStringSmallBold("Sats:", 0, 0, 6); UI_PrintStringSmallNormal(val_sats, 38, 0, 6);
    } 
    else if (page == 1) {
        UI_PrintStringSmallBold("GPS NAV", 0, 0, 0); UI_PrintStringSmallNormal("(2/3)", 70, 0, 0);
		UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        UI_PrintStringSmallBold("Spd:", 0, 0, 3); UI_PrintStringSmallNormal(val_speed, 38, 0, 3); UI_PrintStringSmallNormal("km/h", 70, 0, 3);
        UI_PrintStringSmallBold("Dir:", 0, 0, 5); UI_PrintStringSmallNormal(val_course, 38, 0, 5); UI_PrintStringSmallNormal("deg", 70, 0, 5);
    }
    else {
        UI_PrintStringSmallBold("DATE/TIME", 0, 0, 0); UI_PrintStringSmallNormal("(3/3)", 70, 0, 0);
		UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        UI_PrintStringSmallBold("Date:", 0, 0, 3); UI_PrintStringSmallNormal(val_date, 38, 0, 3);
        UI_PrintStringSmallBold("UTC Time:", 0, 0, 5); UI_PrintStringSmallNormal(val_time, 65, 0, 5);
    }
}

void SubApp_GPS_Info(void) {
    SetupUART(); int current_page = 0; WaitForRelease(); 
    while (1) {
        memset(rx_buffer, 0, RX_BUF_SIZE); int idx = 0; bool complete = false; int max_len = 120; 
        while (idx < max_len) {
            KEY_Code_t key = KEYBOARD_Poll();
            if (key == KEY_EXIT) { WaitForRelease(); return; }
            if (key == KEY_DOWN) { current_page++; if (current_page > 2) current_page = 0; WaitForRelease(); }
            if (key == KEY_UP)  { current_page--; if (current_page < 0) current_page = 2; WaitForRelease(); }
            if (idx % 2 == 0) { UI_DisplayClear(); DrawGPSInfoContent(current_page); DrawProgressBar((idx * 100) / max_len); ST7565_BlitFullScreen(); }
            UART_SendByte((uint8_t)idx); volatile uint32_t t = 100000; char c = 0; bool g = false;
            while (t > 0) { t--; if (UART1->SR & USART_SR_RXNE) { c = (char)(UART1->DR & 0xFF); g = true; break; } }
            if (g) { if ((c >= '0' && c <= '9') || c == '.' || c == ',' || c == '[' || c == ']' || c == ':' || c == '-' || c == '/' || c == ' ') { rx_buffer[idx] = c; if (c == ']') { rx_buffer[idx+1] = '\0'; complete = true; break; } idx++; } }
            volatile uint32_t delay = 8000; while (delay--) {}
        }
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
        for (int t = 0; t < 50; t++) { 
            KEY_Code_t key = KEYBOARD_Poll();
            if (key == KEY_EXIT) { WaitForRelease(); return; }
            if (key == KEY_DOWN) { current_page++; if (current_page > 2) current_page = 0; WaitForRelease(); }
            if (key == KEY_UP)   { current_page--; if (current_page < 0) current_page = 2; WaitForRelease(); }
            UI_DisplayClear(); DrawGPSInfoContent(current_page); ST7565_BlitFullScreen(); volatile uint32_t d = 50000; while (d--) {}
        }
    }
}

bool EditString(char *title, char *buffer, int max_len, bool numeric_only) {
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
}

void SubApp_Settings(void) {
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
}

void SubApp_GridCalc(void) {
    char my_grid_edit[7]; char dx_grid_edit[7] = "------";
    int cursor = 0; float my_lat, my_lon, target_lat, target_lon, dist, bear; char buff[32];
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

        ST7565_BlitFullScreen(); KEY_Code_t key = KEYBOARD_Poll();
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
    }
}

void APP_RunGPS(void) {
    NVIC_DisableIRQ(USART1_IRQn); WaitForRelease(); int menu_cursor = 0;
    while (1) {
        UI_DisplayClear(); UI_PrintStringSmallBold("GPS & GRID", 0, 0, 0); UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 
        if (menu_cursor == 0) UI_PrintStringSmallBold("> GPS INFO", 3, 0, 2); else UI_PrintStringSmallNormal("  GPS INFO", 3, 0, 2);
        if (menu_cursor == 1) UI_PrintStringSmallBold("> GRID DISTANCE", 3, 0, 4); else UI_PrintStringSmallNormal("  GRID DISTANCE", 3, 0, 4);
        ST7565_BlitFullScreen(); KEY_Code_t key = KEYBOARD_Poll();
        if (key == KEY_EXIT) { UART1->CR1 = 0; NVIC_EnableIRQ(USART1_IRQn); __enable_irq(); return; }
        if (key == KEY_UP) { menu_cursor--; if (menu_cursor < 0) menu_cursor = 1; WaitForRelease(); }
        if (key == KEY_DOWN) { menu_cursor++; if (menu_cursor > 1) menu_cursor = 0; WaitForRelease(); }
        if (key == KEY_MENU) { WaitForRelease(); if (menu_cursor == 0) SubApp_GPS_Info(); else if (menu_cursor == 1) SubApp_GridCalc(); WaitForRelease(); }
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