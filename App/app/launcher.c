#include "app/launcher.h"
//#include "app/breakout.h"
//#include "app/snake.h"
//#include "app/minesweeper.h"
#include "app/cw.h"
//#include "app/matrix.h"
//#include "app/rtttl.h"
//#include "app/motorcycle.h"
#include "app/morse.h"
#include "app/callsign.h"
#include "app/gps_app.h"
#include "app/app_aprs.h"
//#include "app/thai_test.h"

#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/bk4819.h"
#include "driver/system.h"
#include "ui/helper.h"
#include "settings.h" 
#include <string.h> 
#include <stdio.h>

// --- Config ---
#define MAX_ITEMS /
#define ITEMS_PER_PAGE 4 // โชว์ได้ทีละ 4 บรรทัด (เต็มจอพอดี)

static const char* menuItems[MAX_ITEMS] = {
	"GPS & APRS",
	"CW TX/RX",
	"CW Trainer",
	"Callsign Country Lookup"
	//"thai test",
	//"Breakout",
	//"Minesweeper",
	//"Snake"
	//"Matrix",
	//"Music Player",
	//"Motorcycle"
};

// --- Variables ---
static int selectedIndex = 0;
static int menuScrollIndex = 0; // ตัวแปรบอกว่าบรรทัดแรกที่โชว์คือรายการที่เท่าไหร่
static bool isLauncherRunning = false;

// --- Helper Functions ---

static void Draw() {
    UI_DisplayClear();
    
    // 1. Header
    UI_PrintStringSmallBold("APP LAUNCHER", 23, 0, 0);
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 10, 127, 11, true); 

    // 2. รายการเมนู (Loop แค่จำนวนที่จะโชว์บนหน้าจอ)
    for(int i=0; i<ITEMS_PER_PAGE; i++) {
        // คำนวณ index จริงในอาร์เรย์ (Offset + บรรทัดที่กำลังวาด)
        int itemIndex = menuScrollIndex + i;
        
        // ถ้า index เกินจำนวนของที่มี ก็หยุดวาด (กรณีหน้าสุดท้ายมีไม่ครบ 4)
        if (itemIndex >= MAX_ITEMS) break;

        // คำนวณตำแหน่ง Y (คงที่เสมอ เพราะเราเลื่อนข้อความแทนเลื่อนตำแหน่ง)
        // บรรทัดที่ 1=14, 2=25, 3=36, 4=47
        int y = 14 + (i * 11); 
        
        char buf[32];
        if (itemIndex == selectedIndex) {
            sprintf(buf, "> %s <", menuItems[itemIndex]); 
            // ตัวเลือกที่เลือกอยู่ ใช้ตัวหนา
            GUI_DisplaySmallest(buf, 5, y, false, true);
        } else {
            sprintf(buf, "  %s", menuItems[itemIndex]);
            // ตัวเลือกอื่น ใช้ตัวบาง
            GUI_DisplaySmallest(buf, 5, y, false, true);
        }
    }
    
    // (Optional) วาด Scrollbar ง่ายๆ ด้านขวา ถ้ามีรายการมากกว่า 1 หน้า
    if (MAX_ITEMS > ITEMS_PER_PAGE) {
        int barHeight = 40; // ความสูงพื้นที่ scroll
        int cursorH = barHeight / MAX_ITEMS; // ความสูง cursor
        if (cursorH < 2) cursorH = 2;
        
        int cursorY = 14 + (selectedIndex * (barHeight - cursorH) / (MAX_ITEMS - 1));
        
        // วาดเส้นแนวตั้ง
        UI_DrawRectangleBuffer(gFrameBuffer, 125, 14, 126, 14+barHeight, true);
        // วาดก้อน cursor
        UI_DrawRectangleBuffer(gFrameBuffer, 124, cursorY, 127, cursorY+cursorH, true);
    }
    
    ST7565_BlitFullScreen();
}

// --- Main Loop ---
void APP_RunLauncher(void) {
    isLauncherRunning = true;
    
    // ล้าง Status Bar
    memset(gStatusLine, 0, sizeof(gStatusLine));
    ST7565_BlitStatusLine();
    
    while(KEYBOARD_Poll() != KEY_INVALID) SYSTEM_DelayMs(10);

    while(isLauncherRunning) {
        KEY_Code_t key = KEYBOARD_Poll();
        
        if (key == KEY_UP || key == KEY_2) {
            if (selectedIndex > 0) {
                selectedIndex--;
                // Logic การเลื่อนจอขึ้น: ถ้าตัวเลือกอยู่เหนือจอ ให้ขยับหน้าจอตาม
                if (selectedIndex < menuScrollIndex) {
                    menuScrollIndex = selectedIndex;
                }
            } else {
                // วนจากบนสุดไปล่างสุด
                selectedIndex = MAX_ITEMS - 1;
                menuScrollIndex = MAX_ITEMS - ITEMS_PER_PAGE;
                if (menuScrollIndex < 0) menuScrollIndex = 0;
            }
            SYSTEM_DelayMs(150); 
        }
        else if (key == KEY_DOWN || key == KEY_8) {
            if (selectedIndex < MAX_ITEMS - 1) {
                selectedIndex++;
                // Logic การเลื่อนจอลง: ถ้าตัวเลือกหลุดขอบล่าง ให้ขยับหน้าจอตาม
                if (selectedIndex >= menuScrollIndex + ITEMS_PER_PAGE) {
                    menuScrollIndex = selectedIndex - ITEMS_PER_PAGE + 1;
                }
            } else {
                // วนจากล่างสุดไปบนสุด
                selectedIndex = 0;
                menuScrollIndex = 0;
            }
            SYSTEM_DelayMs(150);
        }
        else if (key == KEY_MENU || key == KEY_5) {
            switch(selectedIndex) {
				case 0: APP_RunGPS(); break;
                case 1: APP_RunCW(); break;
				case 2: Morse_App_Loop(); break;
				case 3: APP_RunCallSign(); break;
				//case 4: APP_ThaiTest_HandleKeys(); break;
                //case 4: APP_RunBreakout(); break;
                //case 5: APP_RunMinesweeper(); break;
                //case 6: APP_RunSnake(); break;
				//case 7: APP_RunMatrix(); break;
				//case 8: APP_RunRTTTL(); break;
				//case 7: APP_RunMotorcycle(); break;
            }
            
            UI_DisplayClear();
            memset(gStatusLine, 0, sizeof(gStatusLine));
            ST7565_BlitStatusLine();
            while(KEYBOARD_Poll() != KEY_INVALID) SYSTEM_DelayMs(10);
        }
        else if (key == KEY_EXIT) {
            isLauncherRunning = false;
        }

        Draw(); 
        SYSTEM_DelayMs(20); 
    }
}
