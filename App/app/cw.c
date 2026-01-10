#include "app/cw.h"
#include "driver/bk4819.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "driver/keyboard.h"
#include "driver/gpio.h" 
#include "settings.h"
#include "../audio.h" 
#include "../radio.h" 
#include "ui/helper.h"
#include <string.h>
#include <stdio.h>

#ifdef ENABLE_FEAT_F4HWN_SCREENSHOT
#include "screenshot.h"
#endif

#ifndef BK4819_AF_OPEN
#define BK4819_AF_OPEN 1
#endif

// --- Config ---
#define RSSI_THRESHOLD_OFFSET 10 
#define MAX_COLS 18      
#define VISIBLE_ROWS 3
#define LOG_PAGES 3
// เก็บ Log ทั้งหมด 9 บรรทัด (3 หน้า x 3 บรรทัด)
#define TOTAL_ROWS (VISIBLE_ROWS * LOG_PAGES) 

#define MIN_PULSE_MS 30   
#define MIN_DOT_MS 40     
#define MAX_DOT_MS 240    

// ความถี่เสียง Sidetone (700Hz)
#define CW_SIDETONE_FREQ 700

// --- Variables ---
static bool isInitialized = false;

// เก็บประวัติข้อความ
static char historyLog[TOTAL_ROWS][MAX_COLS + 1]; 
static int curCol = 0;   

// ตัวแปรเก็บหน้าปัจจุบัน (2 = หน้าล่าสุดเสมอ)
static int viewPage = 2; 

static uint16_t noiseFloor = 0;
static uint32_t stateDuration = 0; 
static bool isSignalOn = false;    

// RX Variables
static uint16_t dotLen = 80; 

// TX Variables
static bool txMode = false;
static uint16_t txDotLen = 100; 
static uint32_t lastTxTime = 0; 

static char symbolBuf[8]; 
static uint8_t symbolCount = 0;

static uint16_t currentRSSI = 0;

static KEY_Code_t lastKey = KEY_INVALID;
static int keyHoldCounter = 0;

// [FIX] เปลี่ยนตัวแปรกันเบิ้ลปุ่มจาก PTT เป็น Key 1
static bool key1WasPressed = false;

// --- Helper Functions ---

// ฟังก์ชันนับจำนวนหน้าที่มีข้อความจริง
static int GetTotalActivePages() {
    // เช็คหน้า 1 (บรรทัด 0-2)
    bool page1HasData = false;
    for(int i=0; i<VISIBLE_ROWS; i++) {
        if(strlen(historyLog[i]) > 0) { page1HasData = true; break; }
    }
    
    // เช็คหน้า 2 (บรรทัด 3-5)
    bool page2HasData = false;
    for(int i=VISIBLE_ROWS; i<VISIBLE_ROWS*2; i++) {
        if(strlen(historyLog[i]) > 0) { page2HasData = true; break; }
    }

    // หน้า 3 (บรรทัด 6-8) ถือว่ามี data เสมอเพราะเป็นหน้าปัจจุบัน
    
    if (page1HasData) return 3; 
    if (page2HasData) return 2; 
    return 1; 
}

/*
static char DecodeMorse(const char* code) {
    if (strcmp(code, ".-") == 0) return 'A';
    if (strcmp(code, "-...") == 0) return 'B';
    if (strcmp(code, "-.-.") == 0) return 'C';
    if (strcmp(code, "-..") == 0) return 'D';
    if (strcmp(code, ".") == 0) return 'E';
    if (strcmp(code, "..-.") == 0) return 'F';
    if (strcmp(code, "--.") == 0) return 'G';
    if (strcmp(code, "....") == 0) return 'H';
    if (strcmp(code, "..") == 0) return 'I';
    if (strcmp(code, ".---") == 0) return 'J';
    if (strcmp(code, "-.-") == 0) return 'K';
    if (strcmp(code, ".-..") == 0) return 'L';
    if (strcmp(code, "--") == 0) return 'M';
    if (strcmp(code, "-.") == 0) return 'N';
    if (strcmp(code, "---") == 0) return 'O';
    if (strcmp(code, ".--.") == 0) return 'P';
    if (strcmp(code, "--.-") == 0) return 'Q';
    if (strcmp(code, ".-.") == 0) return 'R';
    if (strcmp(code, "...") == 0) return 'S';
    if (strcmp(code, "-") == 0) return 'T';
    if (strcmp(code, "..-") == 0) return 'U';
    if (strcmp(code, "...-") == 0) return 'V';
    if (strcmp(code, ".--") == 0) return 'W';
    if (strcmp(code, "-..-") == 0) return 'X';
    if (strcmp(code, "-.--") == 0) return 'Y';
    if (strcmp(code, "--..") == 0) return 'Z';
    
    if (strcmp(code, ".----") == 0) return '1';
    if (strcmp(code, "..---") == 0) return '2';
    if (strcmp(code, "...--") == 0) return '3';
    if (strcmp(code, "....-") == 0) return '4';
    if (strcmp(code, ".....") == 0) return '5';
    if (strcmp(code, "-....") == 0) return '6';
    if (strcmp(code, "--...") == 0) return '7';
    if (strcmp(code, "---..") == 0) return '8';
    if (strcmp(code, "----.") == 0) return '9';
    if (strcmp(code, "-----") == 0) return '0';
	
	if (strcmp(code, "----") == 0) return '_';
	if (strcmp(code, ".-.-.-") == 0) return '.';
	if (strcmp(code, "-...-") == 0) return '=';
	if (strcmp(code, "--..--") == 0) return ',';
	if (strcmp(code, "-..-.") == 0) return '/';
	if (strcmp(code, "..--..") == 0) return '?';
	
    return '?'; 
}
*/

// Lookup table: {morse_code, character}
static const struct {
    const char* code;
    char ch;
} morse_table[] = {
    // Letters A-Z
    {".-", 'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..", 'D'},
    {".", 'E'}, {"..-.", 'F'}, {"--.", 'G'}, {"....", 'H'},
    {"..", 'I'}, {".---", 'J'}, {"-.-", 'K'}, {".-..", 'L'},
    {"--", 'M'}, {"-.", 'N'}, {"---", 'O'}, {".--.", 'P'},
    {"--.-", 'Q'}, {".-.", 'R'}, {"...", 'S'}, {"-", 'T'},
    {"..-", 'U'}, {"...-", 'V'}, {".--", 'W'}, {"-..-", 'X'},
    {"-.--", 'Y'}, {"--..", 'Z'},
    
    // Numbers 0-9
    {".----", '1'}, {"..---", '2'}, {"...--", '3'}, {"....-", '4'},
    {".....", '5'}, {"-....", '6'}, {"--...", '7'}, {"---..", '8'},
    {"----.", '9'}, {"-----", '0'},
    
    // Special characters
    {"----", '_'}, {".-.-.-", '.'}, {"-...-", '='}, {"--..--", ','},
    {"-..-.", '/'}, {"..--..", '?'},
    
    {NULL, '\0'} // Terminator
};

static char DecodeMorse(const char* code) {
    for (int i = 0; morse_table[i].code != NULL; i++) {
        if (strcmp(code, morse_table[i].code) == 0) {
            return morse_table[i].ch;
        }
    }
    return '?'; // Unknown code
}

static void AppendChar(char c) {
    if (curCol >= MAX_COLS) {
        // Shift Log Up
        for (int i = 0; i < TOTAL_ROWS - 1; i++) {
            strcpy(historyLog[i], historyLog[i+1]);
        }
        memset(historyLog[TOTAL_ROWS - 1], 0, sizeof(historyLog[TOTAL_ROWS - 1]));
        curCol = 0;
    }
    historyLog[TOTAL_ROWS - 1][curCol++] = c;
    historyLog[TOTAL_ROWS - 1][curCol] = '\0'; 
    
    // Auto Snap to latest page
    viewPage = 2; 
}

static void ProcessSymbolBuffer() {
    if (symbolCount == 0) return;
    symbolBuf[symbolCount] = '\0'; 
    char decoded = DecodeMorse(symbolBuf);
    AppendChar(decoded);
    symbolCount = 0;
    memset(symbolBuf, 0, sizeof(symbolBuf));
}

static void Draw() {
    UI_DisplayClear();

    // Header
    uint32_t freq = gEeprom.VfoInfo[gEeprom.TX_VFO].pRX->Frequency;
    uint8_t mod = gEeprom.VfoInfo[gEeprom.TX_VFO].Modulation;
    uint8_t bw = gEeprom.VfoInfo[gEeprom.TX_VFO].CHANNEL_BANDWIDTH;
    
    char modStr[8];
    switch(mod) {
        case MODULATION_FM:  strcpy(modStr, "(FM)"); break;
        case MODULATION_AM:  strcpy(modStr, "(AM)"); break;
        case MODULATION_USB: strcpy(modStr, "(USB)"); break;
        default:      strcpy(modStr, "(\?\?)"); break;
    }

    char bwStr[6];
    switch(bw) {
        case BK4819_FILTER_BW_WIDE:     strcpy(bwStr, "25k"); break;
        case BK4819_FILTER_BW_NARROW:   strcpy(bwStr, "12k"); break;
        case BK4819_FILTER_BW_NARROWER: strcpy(bwStr, "6k"); break;
        default:                        strcpy(bwStr, "?k"); break;
    }

    char freqStr[32];
	sprintf(freqStr, "%lu.%05lu %s %s", freq / 100000, freq % 100000, modStr, bwStr);
    UI_PrintStringSmallBold(freqStr, 2, 0, 0);

    UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true);

    // --- Dynamic Page Display Logic ---
    int totalPages = GetTotalActivePages();
    int minValidPageIndex = 3 - totalPages; 
    if (viewPage < minValidPageIndex) viewPage = minValidPageIndex;

    // คำนวณบรรทัดที่จะแสดง
    int startRow = viewPage * VISIBLE_ROWS;

    // แสดงข้อความ 3 บรรทัด
    GUI_DisplaySmallest(historyLog[startRow], 25, 13, false, true);
    GUI_DisplaySmallest(historyLog[startRow + 1], 25, 21, false, true);
    
    // ถ้าเป็นหน้าล่าสุด บรรทัดสุดท้ายเป็นตัวหนา
    if (viewPage == 2) {
        UI_PrintStringSmallBold(historyLog[startRow + 2], 1, 0, 4);
    } else {
        GUI_DisplaySmallest(historyLog[startRow + 2], 25, 29, false, true);
    }

    // แสดงเลขหน้า
    int displayNum = viewPage - (3 - totalPages) + 1;
    char pageStr[32];
    sprintf(pageStr, "%d/%d", displayNum, totalPages);
    GUI_DisplaySmallest(pageStr, 108, 13, false, true);

    UI_DrawRectangleBuffer(gFrameBuffer, 0, 41, 127, 42, true);

    // Status Bar
    char symDisp[12];
    memset(symDisp, 0, sizeof(symDisp));
    for(int i=0; i<symbolCount; i++) symDisp[i] = symbolBuf[i];
    
    if(txMode) {
        GUI_DisplaySmallest("OUT:", 2, 44, false, true);
    } else {
        GUI_DisplaySmallest("IN:", 2, 44, false, true); 
    }
    
    GUI_DisplaySmallest(symDisp, 20, 44, false, true);

    char spd[32];
    int txWpm = 1200 / txDotLen;
    int rxWpm = 1200 / dotLen;
    sprintf(spd, "TX:%d RX:%d", txWpm, rxWpm);
    GUI_DisplaySmallest(spd, 62, 44, false, true);

	uint8_t pwr = gEeprom.VfoInfo[gEeprom.TX_VFO].OUTPUT_POWER;
    char pwrStr[16];
    if (pwr >= 1 && pwr <= 5) sprintf(pwrStr, "Pow:L%d", pwr);
    else if (pwr == 6) strcpy(pwrStr, "Pow:M");
    else if (pwr == 7) strcpy(pwrStr, "Pow:H");
    else strcpy(pwrStr, "Pow:L"); // เผื่อกรณีค่าเป็น 0
	GUI_DisplaySmallest(pwrStr, 105, 44, false, true);

    UI_DrawRectangleBuffer(gFrameBuffer, 1, 55, 25 + (currentRSSI/4), 60, true);
    
    int thValue = noiseFloor + RSSI_THRESHOLD_OFFSET;
    int thLine = 25 + (thValue / 4);
    UI_DrawRectangleBuffer(gFrameBuffer, thLine, 52, thLine+1, 63, true);
    
    char debugStr[32];
    sprintf(debugStr, "R:%d T:%d", currentRSSI, thValue);
    GUI_DisplaySmallest(debugStr, 85, 50, false, true);

    ST7565_BlitFullScreen();
}

static void UpdateDotLen(uint32_t newLen) {
    if (newLen < MIN_DOT_MS) newLen = MIN_DOT_MS;
    if (newLen > MAX_DOT_MS) newLen = MAX_DOT_MS;
    dotLen = ((dotLen * 3) + newLen) / 4;
}

static void TX_Start(void) {
    uint32_t txFreq = gEeprom.VfoInfo[gEeprom.TX_VFO].pRX->Frequency;
    BK4819_SetFrequency(txFreq);

    AUDIO_AudioPathOff();
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, false); 
    
    BK4819_WriteRegister(BK4819_REG_70, 0); 
    BK4819_WriteRegister(BK4819_REG_51, 0);
    BK4819_WriteRegister(BK4819_REG_7D, 0);

    BK4819_EnterTxMute();
    BK4819_EnableTXLink();
    
    BK4819_SetupPowerAmplifier(gEeprom.VfoInfo[gEeprom.TX_VFO].TXP_CalculatedSetting, txFreq);
}

static void TX_Key(bool on) {
    if (on) {
        BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, true);
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
    } else {
        BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
    }
}

static void TX_Stop(void) {
    TX_Key(false);
    
    BK4819_EnterTxMute();
    BK4819_TurnsOffTones_TurnsOnRX(); 
    
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
    
    RADIO_SetupRegisters(true); 
    
    AUDIO_AudioPathOn();
    BK4819_SetAF(BK4819_AF_OPEN); 
}

void APP_RunCW(void) {
    isInitialized = true;
    txMode = false;
    
    dotLen = 80; 
    txDotLen = 100;
    
    // Clear Logs
    for(int i=0; i<TOTAL_ROWS; i++) memset(historyLog[i], 0, sizeof(historyLog[i]));
    curCol = 0;
    viewPage = 2; // เริ่มที่หน้าล่าสุดเสมอ
    
    AUDIO_AudioPathOn();           
    BK4819_SetAF(BK4819_AF_OPEN); 

    uint32_t startFreq = gEeprom.VfoInfo[gEeprom.TX_VFO].pRX->Frequency;
    BK4819_SetFrequency(startFreq);
    BK4819_PickRXFilterPathBasedOnFrequency(startFreq);
    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
    BK4819_RX_TurnOn();
    
    memset(gStatusLine, 0, sizeof(gStatusLine));
    GUI_DisplaySmallest("CW TX/RX", 0, 1, true, true);
    ST7565_BlitStatusLine();
    
    symbolCount = 0;
    stateDuration = 0;
    isSignalOn = false;
    currentRSSI = 0;
    
    uint32_t sum = 0;
    for(int i=0; i<10; i++) {
        sum += BK4819_GetRSSI();
        SYSTEM_DelayMs(10);
    }
    noiseFloor = sum / 10;
    
    uint32_t lastDrawTime = 0;
	lastTxTime = 0;

    keyHoldCounter = 0;
    lastKey = KEY_INVALID;
    key1WasPressed = false; // Reset ตัวกันเบิ้ลปุ่ม 1

    while(isInitialized) {
        KEY_Code_t key = KEYBOARD_Poll();
        
        // --- Page Cycling Logic (กดปุ่ม 1) ---
        if (key == KEY_1) {
            if (!key1WasPressed) {
                int total = GetTotalActivePages();
                int minIdx = 3 - total; 
                
                if (viewPage == 2) {
                    viewPage = minIdx;
                } else {
                    viewPage++;
                }
                
                Draw();
                key1WasPressed = true;
            }
        } else {
            key1WasPressed = false;
        }

		if (key == KEY_6) {
             uint8_t *pwr = &gEeprom.VfoInfo[gEeprom.TX_VFO].OUTPUT_POWER;
             (*pwr)++;
             if (*pwr > 7) *pwr = 1; // วนลูป 1->7->1 (L1-H)
             
             // อัปเดตค่าไปยัง Hardware และคำนวณ Bias ใหม่
             RADIO_ConfigureSquelchAndOutputPower(&gEeprom.VfoInfo[gEeprom.TX_VFO]);
             
             Draw(); // วาดหน้าจอใหม่เพื่ออัปเดตสถานะ
             SYSTEM_DelayMs(200); // หน่วงเวลา
        }

        // [ADD 2] อ่านค่าจาก Paddle (ADC)
        // 0=None, 1=Dot, 2=Dash
        int paddleState = ReadPaddleADC();
        
        bool triggerDot  = (key == KEY_7) || (paddleState == 1);
        bool triggerDash = (key == KEY_9) || (paddleState == 2);
		
        // --- CW Sending Logic ---
        if (triggerDot || triggerDash) {
            if (!txMode) {
                TX_Start();
                txMode = true;
                viewPage = 2; // Auto snap
                Draw(); 
            }

            TX_Key(true);
            
            if(symbolCount < 7) {
                //symbolBuf[symbolCount++] = (key == KEY_7) ? '.' : '-';
				symbolBuf[symbolCount++] = triggerDot ? '.' : '-';
            }
            
            lastTxTime = 0; 

            //int duration = (key == KEY_7) ? txDotLen : (txDotLen * 3);
			int duration = triggerDot ? txDotLen : (txDotLen * 3);
            SYSTEM_DelayMs(duration);
            
            TX_Key(false);
            SYSTEM_DelayMs(txDotLen); 
            
        } else {
            if (txMode) {
                TX_Stop();
                txMode = false;
                
                isSignalOn = false;
                stateDuration = 0;
                Draw(); 
            }
            
            if (key == KEY_SIDE1) { 
                if (txDotLen > MIN_DOT_MS) txDotLen -= 5;
                Draw();
                SYSTEM_DelayMs(100); 
            } 
            else if (key == KEY_SIDE2) { 
                if (txDotLen < MAX_DOT_MS) txDotLen += 5;
                Draw();
                SYSTEM_DelayMs(100);
            }

            if(key == KEY_EXIT) isInitialized = false;
            if(key == KEY_MENU) noiseFloor = BK4819_GetRSSI();

            if (key == KEY_0) {
                uint8_t *mod = &gEeprom.VfoInfo[gEeprom.TX_VFO].Modulation;
                if (*mod == MODULATION_FM) *mod = MODULATION_AM;
                else if (*mod == MODULATION_AM) *mod = MODULATION_USB;
                else *mod = MODULATION_FM;
                RADIO_SetModulation(*mod);
                Draw();
                SYSTEM_DelayMs(200); 
            }
            if (key == KEY_F) { 
                uint8_t *bw = &gEeprom.VfoInfo[gEeprom.TX_VFO].CHANNEL_BANDWIDTH;
                if (*bw == BK4819_FILTER_BW_WIDE) *bw = BK4819_FILTER_BW_NARROW;
                else if (*bw == BK4819_FILTER_BW_NARROW) *bw = BK4819_FILTER_BW_NARROWER;
                else *bw = BK4819_FILTER_BW_WIDE;
                BK4819_SetFilterBandwidth(*bw, false);
                Draw();
                SYSTEM_DelayMs(200);
            }
            if (key == KEY_UP || key == KEY_DOWN) {
                if (key == lastKey) keyHoldCounter++; else { keyHoldCounter = 0; lastKey = key; }
                int step = 1; int throttle = 10;
                if (keyHoldCounter > 100) { step = 500; throttle = 8; }
                else if (keyHoldCounter > 80) { step = 200; throttle = 10; }
                else if (keyHoldCounter > 60) { step = 100; throttle = 15; }
                else if (keyHoldCounter > 40) { step = 50; throttle = 20; }
                else if (keyHoldCounter > 20) { step = 10; throttle = 30; }
                if (keyHoldCounter == 0 || (keyHoldCounter > 15 && keyHoldCounter % throttle == 0)) {
                    uint32_t *freq = &gEeprom.VfoInfo[gEeprom.TX_VFO].pRX->Frequency;
                    if (key == KEY_UP) *freq -= step; else *freq += step;
                    BK4819_SetFrequency(*freq);
                    BK4819_PickRXFilterPathBasedOnFrequency(*freq);
                    BK4819_ToggleGpioOut(BK4819_GPIO0_PIN28_RX_ENABLE, true);
                    BK4819_RX_TurnOn(); 
                    Draw();
                    lastDrawTime = 0; isSignalOn = false; stateDuration = 0;
                }
            } else {
                if (key == KEY_INVALID || (key != KEY_UP && key != KEY_DOWN && key != KEY_1)) {
                    keyHoldCounter = 0;
                    if (key != KEY_INVALID) lastKey = key; else lastKey = KEY_INVALID;
                }
            }
        } 
        
        // --- RX Decoder Logic ---
        if (!txMode) {
            currentRSSI = BK4819_GetRSSI();
            bool signalDetected = (currentRSSI > (noiseFloor + RSSI_THRESHOLD_OFFSET));
            
            if (signalDetected) {
                if (!isSignalOn) {
                    if (stateDuration >= (dotLen * 5)) {
                       ProcessSymbolBuffer();
                       AppendChar(' ');
                    }
                    isSignalOn = true;
                    stateDuration = 0;
                }
            } else {
                if (isSignalOn) {
                    if (stateDuration > MIN_PULSE_MS) { 
                        if (stateDuration < (dotLen * 1.5)) {
                            if(symbolCount < 7) symbolBuf[symbolCount++] = '.';
                             UpdateDotLen(stateDuration);
                        } else {
                            if(symbolCount < 7) symbolBuf[symbolCount++] = '-';
                            UpdateDotLen(stateDuration / 3);
                        }
                    }
                    isSignalOn = false;
                    stateDuration = 0;
                } else {
                    if (symbolCount > 0 && stateDuration > (dotLen * 3)) {
                         ProcessSymbolBuffer();
                    }
                }
            }
            stateDuration += 10; 
        }

        lastDrawTime += 10;
        if (keyHoldCounter == 0 && lastDrawTime >= 150) {
            Draw();
            lastDrawTime = 0;
        }
        
        SYSTEM_DelayMs(10); 
    }

    if(txMode) TX_Stop(); 
    BK4819_SetAF(BK4819_AF_MUTE); 
    AUDIO_AudioPathOff();         

}
