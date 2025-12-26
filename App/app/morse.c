/*
 * morse.c
 * Morse Code Learning App for Quansheng UV-K5
 * Created by: E25WOP & Gemini
 */

#include "morse.h"
#include <string.h>
//#include <stdlib.h> 

#include "driver/keyboard.h"
#include "../audio.h"
#include "driver/st7565.h"
#include "driver/bk4819.h"
#include "ui/helper.h"
#include "driver/system.h"
#include "external/printf/printf.h"

// ==========================================
// 1. DATA & VARIABLES
// ==========================================

// ตารางรหัสมอร์ส (คงเดิมไว้ตามที่ขอ)
const MorseChar morse_table[42] = {
    {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},
    {'E', "."},    {'F', "..-."}, {'G', "--."},  {'H', "...."},
    {'I', ".."},   {'J', ".---"}, {'K', "-.-"},  {'L', ".-.."},
    {'M', "--"},   {'N', "-."},   {'O', "---"},  {'P', ".--."},
    {'Q', "--.-"}, {'R', ".-."},  {'S', "..."},  {'T', "-"},
    {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"},
    {'Y', "-.--"}, {'Z', "--.."},
    {'1', ".----"},{'2', "..---"},{'3', "...--"},{'4', "....-"},{'5', "....."},
    {'6', "-...."},{'7', "--..."},{'8', "---.."},{'9', "----."},{'0', "-----"},
    {'.', ".-.-.-"},{',', "--..--"},{'?', "..--.."},{'/', "-..-."},{'@', ".--.-."},{'=', "-...-"}
};

// Enum สถานะของแอป
typedef enum {
    STATE_MAIN_MENU,
    STATE_LIST,
    STATE_SETTINGS,
    STATE_PRACTICE_SELECT, // หน้าเลือกโหมด (จับเวลา/ไม่จับ)
    STATE_GAME_LISTEN,     // หน้า "กำลังฟังเสียง..."
    STATE_GAME_ANSWER,     // หน้า "เลือกคำตอบ" (จับเวลาที่นี่)
    STATE_GAME_RESULT,     // หน้าเฉลย (ถูก/ผิด)
    STATE_GAME_SCORE,       // หน้าสรุปคะแนนตอนจบ
	STATE_PRACTICE_SEND, // โหมดฝึกส่ง
    STATE_PRACTICE_ECHO
} AppState;

// ชื่อเมนูที่จะโชว์
const char* menu_items[] = {
    "1. Morse List",
    "2. Practice",
    "3. Settings"
};

// ตัวแปร Global ของแอป
AppState current_state = STATE_MAIN_MENU;
int menu_cursor = 0;       // ตัวชี้ตำแหน่งเมนู (0-2)
const int MENU_COUNT = 3;  // จำนวนเมนูทั้งหมด

// เพิ่มตัวแปรจัดการหน้า List (ใส่ต่อจาก menu_cursor)
int list_cursor = 0;      // ตัวชี้ตำแหน่งในหน้า List
int list_scroll_index = 0; // บรรทัดแรกที่โชว์
#define LIST_ITEMS_PER_PAGE 8 // โชว์ 4 บรรทัดต่อหน้า

// --- ตัวแปรสำหรับ Game Logic ---
bool is_timed_mode = false;     // โหมดจับเวลาหรือไม่
int score_correct = 0;          // คะแนนที่ทำได้
int score_total = 0;            // จำนวนข้อที่ทำไป
int game_timer = 0;             // ตัวนับเวลา (5 วินาที)

char quiz_answer_str[12];       // โจทย์คำตอบที่ถูกต้อง (เช่น "ABC")
char quiz_choices[4][12];       // ตัวเลือก 4 ข้อ
int quiz_correct_index = 0;     // Index ของข้อที่ถูก (0-3)
int quiz_cursor = 0;            // Cursor เลือกช้อยส์
bool is_time_up = false;        // หมดเวลาหรือยัง

static uint32_t rng_seed = 12345;

uint8_t setting_wpm = MORSE_WPM_DEFAULT;
uint16_t setting_tone = MORSE_TONE_DEFAULT;
uint8_t setting_length = 5;     // จำนวนตัวอักษรที่จะสุ่ม (Default 5)
int setting_step = 0;           // 0=Tone, 1=WPM, 2=Length

// --- ตัวแปรสำหรับโหมด Send Practice ---
char send_buf[10];           // เก็บจุดขีดที่กำลังเคาะ เช่น ".-."
char send_history[21];       // เก็บตัวอักษรที่แปลได้แล้ว เช่น "HELLO"
bool is_key_pressed = false; // สถานะปุ่มกดปัจจุบัน
uint32_t key_timer = 0;      // จับเวลาตอนกด/ปล่อย
bool reload_send_screen = false;	// [เพิ่ม] ตัวช่วยรีเซ็ตหน้าจอ
char last_code_buf[16];	// [เพิ่ม] ตัวแปรเก็บรหัสล่าสุดเพื่อแสดงผล

// [เพิ่ม] ตัวแปรสำหรับ Echo Mode
char echo_target_char;      // ตัวโจทย์ (เช่น 'A')
char echo_input_char;       // ตัวที่เรากด (เช่น 'B')
char echo_input_code[10];   // รหัสที่เรากด (เช่น "-...")
int echo_score_correct = 0; // คะแนนที่ถูก
int echo_score_total = 0;   // จำนวนข้อทั้งหมด

bool is_quiz_mode = false; // ตัวแปรจำว่าเล่นโหมด Quiz (ฟังเสียง) อยู่หรือไม่

int my_rand() {
    rng_seed = rng_seed * 1103515245 + 12345;
    return (unsigned int)(rng_seed / 65536) % 32768;
}

// ==========================================
// 2. HELPER FUNCTIONS (UI WRAPPERS)
// ==========================================

void UI_DrawText(int x, int y, const char* text, bool isInverted) {
    // แก้ไข: บังคับให้พารามิเตอร์สุดท้ายเป็น true เสมอ เพื่อให้วาดเป็นสีดำ
    // ไม่ว่า isInverted จะเป็นอะไร เราจะใช้วิธีแสดงผลแค่ลูกศร > นำหน้าแทนการทำแถบสี
    GUI_DisplaySmallest(text, x, y, false, true);
}

void UI_ClearScreen() {
    UI_DisplayClear(); // ฟังก์ชันมาตรฐาน
}

void UI_Flush() {
    ST7565_BlitFullScreen(); // สั่งให้จอแสดงผล
}

// ==========================================
// 3. MAIN MENU LOGIC
// ==========================================

void Render_MainMenu() {
    UI_ClearScreen();

    // --- Line 0: Header ---
    UI_PrintStringSmallBold("CW TRAINER", 0, 0, 0);
    
    // วาดเส้นคั่นสวยๆ (ตำแหน่ง Y=9 ถึง 10)
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true);

    // --- Line 2-4: รายการเมนู ---
    for (int i = 0; i < MENU_COUNT; i++) {
        char buf[32];
        
        // เช็คว่าเมนูไหนถูกเลือก ให้ใส่ลูกศร >
        if (i == menu_cursor) {
            sprintf(buf, "> %s", menu_items[i]);
        } else {
            sprintf(buf, "  %s", menu_items[i]);
        }
        
        // พิมพ์เริ่มที่บรรทัด 2 (ต่อจาก Header)
        // i=0 -> Line 2
        // i=1 -> Line 3
        // i=2 -> Line 4
        UI_PrintStringSmallBold(buf, 0, 0, 2 + i);
    }
    
    // --- Line 6: Footer ---
    UI_PrintStringSmallBold("[MENU] Select", 0, 0, 6);

    UI_Flush();
}

void Render_MorseList() {
    UI_ClearScreen();

    // --- Line 0: Header ---
    UI_PrintStringSmallBold("CODE LIST", 0, 0, 0); 
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true);

    // --- Line 2-5: รายการ (Loop 8 รอบ) ---
    for (int i = 0; i < LIST_ITEMS_PER_PAGE; i++) {
        // คำนวณ Index
        int itemIndex = list_scroll_index + i;
        
        // ถ้าข้อมูลหมดแล้ว หยุดวาด
        if (itemIndex >= 42) break; 

        // คำนวณบรรทัด และ คอลัมน์
        int row = 2 + (i / 2);    // บรรทัด 2, 2, 3, 3, 4, 4, 5, 5
        int col = i % 2;          // คอลัมน์ 0 (ซ้าย), 1 (ขวา)
        
        // กำหนดจุดเริ่ม X (Start Pixel)
        // คอลัมน์ซ้ายเริ่ม 0, คอลัมน์ขวาเริ่ม 65 (เกินครึ่งมา 1px กันเหนียว)
        int x_start = (col == 0) ? 0 : 65; 

        char buf[20];
        char marker = (itemIndex == list_cursor) ? '>' : ' '; // ถ้าเลือกอยู่ใช้ > ถ้าไม่ใช้เว้นวรรค

        // สร้างข้อความแบบ "Compact" (ประหยัดที่สุดๆ)
        // รูปแบบ: ">A .-" (ตัดเว้นวรรคหลังลูกศรออก)
        sprintf(buf, "%c%c %s", marker, morse_table[itemIndex].character, morse_table[itemIndex].code);
        
        // พิมพ์ลงจอ โดยระบุ Start X
        UI_PrintStringSmallBold(buf, x_start, 0, row);
    }
    
    // --- Line 7: เลขหน้า (อยู่ล่างสุด) ---
    char pageBuf[16];
    sprintf(pageBuf, "%d/42", list_cursor + 1);
    UI_PrintStringSmallBold(pageBuf, 88, 0, 0); 

    UI_Flush();
}

void Morse_PlaySound(const char* code) {
    if (code == NULL) return;

    // 1. เปิด Audio Path (สำคัญ! ต้องปลุก Amp ให้ตื่นก่อน)
    AUDIO_AudioPathOn();
    SYSTEM_DelayMs(50); // รอให้ระบบเสียงพร้อม

    // คำนวณความยาว Dot
    uint32_t dot_length = 1200 / setting_wpm;

    for (int i = 0; code[i] != '\0'; i++) {
        uint32_t duration = 0;
        
        if (code[i] == '.') {
            duration = dot_length;
        } else if (code[i] == '-') {
            duration = dot_length * 3;
        }

        if (duration > 0) {
            // 2. เริ่มจ่ายเสียง
            BK4819_PlayTone(setting_tone, true);
            
            // 3. ปลด Mute (สำคัญ! ถ้าไม่ทำ เสียงจะไม่ออกลำโพง)
            BK4819_ExitTxMute(); 
            
            // รอจนจบเสียง
            SYSTEM_DelayMs(duration);
            
            // 4. Mute กลับ และปิด Tone
            BK4819_EnterTxMute(); 
            BK4819_PlayTone(0, false); 
        }

        // ระยะห่างระหว่างเสียง
        SYSTEM_DelayMs(dot_length);
    }
    
    // 5. ปิด Audio Path เมื่อเล่นจบเพื่อประหยัดแบต
    AUDIO_AudioPathOff();
}

// Helper: สร้าง Random String ตามความยาวที่ตั้งไว้
void GenerateRandomString(char* buffer) {
    for(int i=0; i<setting_length; i++) {
        // สุ่ม 0-41 (A-Z, 0-9, chars)
        // แก้ไข: ใช้ my_rand() แทน rand()
        int r = my_rand() % 42; 
        buffer[i] = morse_table[r].character;
    }
    buffer[setting_length] = '\0'; // ปิดท้าย string
}

// Helper: สร้างตัวหลอก (Distractor) ตามกลยุทธ์ที่กำหนด
// strategy: 0=สุ่มหมด, 1=ล็อคหน้า, 2=ล็อคหลัง, 3=ล็อคหัวท้าย
void GenerateDistractor(char* dest, const char* correct, int strategy) {
    // ก๊อปปี้คำตอบที่ถูกมาก่อน
    strcpy(dest, correct);
    
    int start_change = 0;
    int end_change = setting_length - 1;

    // กำหนดช่วงที่จะทำการสุ่มใหม่ (ส่วนที่ไม่ถูกล็อค)
    if (strategy == 1) { 
        // ล็อคตัวหน้า (เริ่มสุ่มที่ Index 1 ถึงจบ)
        start_change = 1; 
    } 
    else if (strategy == 2) { 
        // ล็อคตัวหลัง (เริ่มสุ่มที่ 0 ถึง รองสุดท้าย)
        end_change = setting_length - 2;
    }
    else if (strategy == 3) { 
        // ล็อคหัวท้าย (เริ่มสุ่มที่ 1 ถึง รองสุดท้าย)
        start_change = 1;
        end_change = setting_length - 2;
    }

    // วนลูปเปลี่ยนตัวอักษรเฉพาะในช่วงที่กำหนด
    for (int i = start_change; i <= end_change; i++) {
        // สุ่มตัวใหม่
        int r = my_rand() % 42;
        dest[i] = morse_table[r].character;
    }
    
    // Safety Check: ถ้าสุ่มแล้วดันไปเหมือนกับคำตอบเป๊ะๆ (เกิดขึ้นได้น้อยแต่กันไว้ก่อน)
    if (strcmp(dest, correct) == 0) {
        // บังคับเปลี่ยน 1 ตัว ในช่วงที่อนุญาตให้เปลี่ยน
        int idx = start_change; 
        do {
            int r = my_rand() % 42;
            dest[idx] = morse_table[r].character;
        } while (dest[idx] == correct[idx]); // เปลี่ยนจนกว่าจะไม่ซ้ำ
    }
}

void GenerateQuiz() {
    // 1. สุ่มข้อถูก (Correct Answer)
    GenerateRandomString(quiz_answer_str);
    
    // 2. เลือกกลยุทธ์การสร้างตัวหลอก (Distractor Strategy)
    int strategy = 0; // Default: 0 = สุ่มมั่ว (ใช้กรณี 1 ตัวอักษร)
    
    if (setting_length == 2) {
        // ถ้ามี 2 ตัว: สุ่มเลือกระหว่าง "ล็อคหน้า" (1) หรือ "ล็อคหลัง" (2)
        strategy = (my_rand() % 2) + 1;
    } 
    else if (setting_length >= 3) {
        // ถ้ามี 3 ตัวขึ้นไป: สุ่มเลือกระหว่าง "ล็อคหน้า"(1), "ล็อคหลัง"(2), "ล็อคหัวท้าย"(3)
        strategy = (my_rand() % 3) + 1;
    }

    // 3. สุ่มตำแหน่งข้อถูก (0-3)
    quiz_correct_index = my_rand() % 4;
    
    // 4. สร้างตัวเลือก 4 ข้อ
    for(int i=0; i<4; i++) {
        if(i == quiz_correct_index) {
            // ข้อนี้เป็นข้อถูก
            strcpy(quiz_choices[i], quiz_answer_str);
        } else {
            // ข้อนี้เป็นข้อผิด (สร้างตาม Strategy)
            if (strategy == 0) {
                GenerateRandomString(quiz_choices[i]);
            } else {
                GenerateDistractor(quiz_choices[i], quiz_answer_str, strategy);
            }
        }
    }
    
    // Reset ค่าต่างๆ
    quiz_cursor = 0;
    is_time_up = false;
    
    // Update Seed
    rng_seed += setting_wpm + score_total + game_timer; 

    // Reset Timer (5 วินาที = 250 ticks)
    game_timer = 250; 
}

// Helper: เล่นเสียงโจทย์ (แปลง String "ABC" เป็นเสียง)
void PlayQuizSound() {
    AUDIO_AudioPathOn();
    SYSTEM_DelayMs(50);
    
    uint32_t dot_len = 1200 / setting_wpm;

    for(int i=0; i<setting_length; i++) {
        char c = quiz_answer_str[i];
        const char* code = NULL;
        
        // หา Code ของตัวอักษรนั้น
        for(int j=0; j<42; j++) {
            if(morse_table[j].character == c) {
                code = morse_table[j].code;
                break;
            }
        }
        
        if(code) {
            // Logic เล่นเสียงเดิม (ย่อมา)
            for (int k = 0; code[k] != '\0'; k++) {
                uint32_t dur = (code[k] == '.') ? dot_len : (dot_len * 3);
                BK4819_PlayTone(setting_tone, true);
                BK4819_ExitTxMute();
                SYSTEM_DelayMs(dur);
                BK4819_EnterTxMute();
                BK4819_PlayTone(0, false);
                SYSTEM_DelayMs(dot_len);
            }
        }
        // เว้นวรรคระหว่างตัวอักษร (3 dots)
        SYSTEM_DelayMs(dot_len * 3);
    }
    AUDIO_AudioPathOff();
}

// Helper: แปลงรหัสจุดขีด เป็นตัวอักษร
char DecodeMorseStr(const char* code) {
    for (int i = 0; i < 42; i++) {
        if (strcmp(morse_table[i].code, code) == 0) {
            return morse_table[i].character;
        }
    }
    return '?'; // ไม่พบในตาราง
}

/* void Render_PracticeSelect() {
    UI_ClearScreen();
    UI_DrawText(32, 0, "PRACTICE", false);
    UI_DrawText(0, 10, "---------------------", false);
    
    UI_DrawText(10, 22, "1. Free Mode", (menu_cursor == 0));
    UI_DrawText(10, 37, "2. Timed (5s)", (menu_cursor == 1));
    
    UI_DrawText(8, 50, "[MENU] Start", false);
    UI_Flush();
} */

void Render_PracticeSelect() {
    UI_ClearScreen();
    
    // --- Line 0: Header ---
    UI_PrintStringSmallBold("PRACTICE", 0, 0, 0);
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true);
    
    char buf[32];

    // 1. Free Mode
    sprintf(buf, "%c 1. RX: Untimed", (menu_cursor == 0) ? '>' : ' ');
    UI_PrintStringSmallBold(buf, 0, 0, 2);
    
    // 2. Timed Mode
    sprintf(buf, "%c 2. RX: Timed(5s)", (menu_cursor == 1) ? '>' : ' ');
    UI_PrintStringSmallBold(buf, 0, 0, 3);

    // 3. Send Mode
    sprintf(buf, "%c 3. TX: Echo", (menu_cursor == 2) ? '>' : ' ');
    UI_PrintStringSmallBold(buf, 0, 0, 4);

    // 4. Echo Mode [เพิ่ม]
    sprintf(buf, "%c 4. TX: Free Key", (menu_cursor == 3) ? '>' : ' ');
    UI_PrintStringSmallBold(buf, 0, 0, 5);
    
    // --- Line 6: Footer ---
    UI_PrintStringSmallBold("[MENU] Start", 0, 0, 6);
    
    UI_Flush();
}

void Render_GameAnswer() {
    UI_ClearScreen();
    
    // --- Line 0: Header & Timer ---
    char headerBuf[32];
    if (is_timed_mode) {
        // รวมข้อความ GUESS และ เวลา ไว้ในบรรทัดเดียวกัน
        sprintf(headerBuf, "CW Training  %d.%d s", (game_timer*20)/1000, ((game_timer*20)%1000)/100);
    } else {
        sprintf(headerBuf, "CW Training  Free");
    }
    UI_PrintStringSmallBold(headerBuf, 0, 0, 0); // บรรทัดที่ 0
    
    // วาดเส้นคั่น (ถ้าต้องการ) ไว้ที่ความสูง pixel ที่ 10
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true); 

    // --- Line 2-5: ตัวเลือก 4 ข้อ ---
    for(int i=0; i<4; i++) {
        char buf[20];
        // เช็คว่าข้อนี้ถูกเลือกอยู่หรือไม่
        if(i == quiz_cursor) {
            sprintf(buf, "> %s", quiz_choices[i]);
        } else {
            sprintf(buf, "  %s", quiz_choices[i]);
        }
        
        // พิมพ์ที่บรรทัด 2, 3, 4, 5 ตามลำดับ
        // (i=0 -> Line 2, i=1 -> Line 3, ...)
        UI_PrintStringSmallBold(buf, 0, 0, 2 + i);
    }
    
    UI_Flush();
}

void Render_GameResult() {
    UI_ClearScreen();
    
    bool isCorrect = (quiz_cursor == quiz_correct_index);
    if(is_time_up) isCorrect = false;

    // --- Line 1-2: แสดงผลลัพธ์ ---
    if (is_time_up) {
         UI_PrintStringSmallBold("TIME'S UP!", 0, 0, 1);
    } else if (isCorrect) {
         UI_PrintStringSmallBold("CORRECT!", 0, 0, 1);
    } else {
         UI_PrintStringSmallBold("WRONG!", 0, 0, 1);
    }
    
    // --- Line 4: เฉลยคำตอบ (ตัวหนา) ---
    char buf[32];
    sprintf(buf, "Ans: %s", quiz_answer_str);
    UI_PrintStringSmallBold(buf, 0, 0, 3); 
    
    // --- [เพิ่มส่วนนี้] Line 5: เฉลยรหัสมอร์ส (ตัวเล็ก) ---
    char morseBuf[64] = ""; // เตรียมที่ว่างไว้เยอะๆ
    
    // วนลูปแปลงคำตอบเป็นรหัสจุดขีด
    for (int i = 0; quiz_answer_str[i] != '\0'; i++) {
        // หาตัวอักษรในตาราง
        for (int j = 0; j < 42; j++) {
            if (morse_table[j].character == quiz_answer_str[i]) {
                // ถ้าไม่ใช่ตัวแรก ให้เติมเว้นวรรคคั่น
                if (i > 0) strcat(morseBuf, "  ");
                strcat(morseBuf, morse_table[j].code);
                break;
            }
        }
    }
    
    // คำนวณหาตำแหน่งกึ่งกลาง (คร่าวๆ ตัวละ 6 pixel)
    int len = strlen(morseBuf);
    int x_pos = (128 - (len * 6)) / 2;
    if (x_pos < 0) x_pos = 0;

    // พิมพ์ที่ Y=41 (ตรงกับบรรทัดที่ 5 กว่าๆ)
    // false สุดท้ายคือ ไม่ตัวหนา (จะได้ประหยัดที่)
    GUI_DisplaySmallest(morseBuf, x_pos, 38, false, true);
    // ----------------------------------------------------
    
    // --- Line 6: คำแนะนำ ---
    UI_PrintStringSmallBold("Press [MENU] Next", 0, 0, 6);
    
    UI_Flush();
}

void Render_ScoreSummary() {
    UI_ClearScreen();
    
    // --- Line 0: Header ---
    UI_PrintStringSmallBold("SUMMARY", 0, 0, 0);
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true);
    
    char buf[32];
    
    // --- Line 2: คะแนน ---
    sprintf(buf, "Score: %d / %d", score_correct, score_total);
    UI_PrintStringSmallBold(buf, 0, 0, 2);
    

	
	// --- Line 3 และ 4 ---
    if (is_quiz_mode) {
		// --- Line 3: ความเร็ว ---
		sprintf(buf, "Speed: %d WPM", setting_wpm);
		UI_PrintStringSmallBold(buf, 0, 0, 3);
		// --- Line 4: จำนวนตัวอักษร (โชว์เฉพาะโหมด Quiz) ---
        sprintf(buf, "Length: %d chars", setting_length);
        UI_PrintStringSmallBold(buf, 0, 0, 4);
    } else {
        // --- Line 3: ถ้าเป็น Echo Mode อาจจะโชว์ว่าเป็นโหมดอะไรแทนก็ได้ (Optional)
        UI_PrintStringSmallBold("Mode: Sending", 0, 0, 4);
    }
    
    // --- Line 6: ปุ่มออก ---
    UI_PrintStringSmallBold("Press [EXIT] Quit", 0, 0, 6);
    
    UI_Flush();
}

void Render_SendPractice() {
    UI_ClearScreen();

    // Line 0: Header
    UI_PrintStringSmallBold("SEND PRACTICE", 0, 0, 0);
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true);
    
    // Line 2: History (ข้อความที่พิมพ์ไปแล้ว)
    int len = strlen(send_history);
    char* show_ptr = send_history;
    if (len > 14) show_ptr = &send_history[len - 14];
    
    char histBuf[32];
    sprintf(histBuf, "\"%s\"", show_ptr);
    UI_PrintStringSmallBold(histBuf, 0, 0, 2);

    // Line 4: Last Code (แก้ไขใหม่ ใช้ last_code_buf)
    char codeBuf[32];
    if (strlen(last_code_buf) > 0) {
        sprintf(codeBuf, "Last: %s", last_code_buf);
    } else {
        sprintf(codeBuf, "Last: -");
    }
    UI_PrintStringSmallBold(codeBuf, 0, 0, 3);
    
    // Line 6-7: Info
    UI_PrintStringSmallBold("[Side2] Manual", 0, 0, 5);
    UI_PrintStringSmallBold("[7] Dot  [9] Dash", 0, 0, 6);

    UI_Flush();
}

void Render_PracticeEcho() {
    UI_ClearScreen();

    // --- Line 0: Header & Score ---
    UI_PrintStringSmallBold("ECHO MODE", 0, 0, 0);
    
    char scoreBuf[16];
    sprintf(scoreBuf, "%d/%d", echo_score_correct, echo_score_total);
    UI_PrintStringSmallBold(scoreBuf, 85, 0, 0);

    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true);
    
    // --- Line 2: Target (โจทย์) ---
    char targetBuf[32];
    sprintf(targetBuf, "Target:  [ %c ]", echo_target_char);
    UI_PrintStringSmallBold(targetBuf, 0, 0, 2);

    // --- Line 4: Input (สิ่งที่เรากด) ---
    // [แก้ไข] เปลี่ยนจาก targetBuf เป็น inputBuf ให้หมดในท่อนนี้
    char inputBuf[32]; 
    if (echo_score_total > 0 && echo_input_char != 0) {
        char mark = (echo_input_char == echo_target_char) ? '/' : 'X';
        sprintf(inputBuf, "You:     [ %c ] %c", echo_input_char, mark); // แก้ตรงนี้
    } else {
        sprintf(inputBuf, "You:     [ ? ]"); // แก้ตรงนี้
    }
    UI_PrintStringSmallBold(inputBuf, 0, 0, 4); // แก้ตรงนี้
    
    // --- Line 5: Morse Code ---
    if (strlen(echo_input_code) > 0) {
        char codeDisp[32];
        sprintf(codeDisp, "Code: %s", echo_input_code);
        GUI_DisplaySmallest(codeDisp, 4, 42, false, true);
    }

    // --- Line 7: Footer ---
    UI_PrintStringSmallBold("[Side2] to Key", 0, 0, 7);

    UI_Flush();
}

void Render_Settings() {
    UI_ClearScreen();

    // --- Line 0: Header ---
    UI_PrintStringSmallBold("SETTINGS", 0, 0, 0);
    UI_DrawRectangleBuffer(gFrameBuffer, 0, 9, 127, 10, true);

    char val[32];

    // --- แสดงผลตามขั้นตอน (Step) ---
    if (setting_step == 0) {
        // Step 1: Tone
        UI_PrintStringSmallBold("1. Tone Freq (Hz)", 0, 0, 2);
        
        sprintf(val, "    < %d >", setting_tone); // เคาะวรรคจัดกึ่งกลางนิดหน่อย
        UI_PrintStringSmallBold(val, 0, 0, 3);

    } else if (setting_step == 1) {
        // Step 2: WPM
        UI_PrintStringSmallBold("2. Speed (WPM)", 0, 0, 2);
        
        sprintf(val, "    < %d >", setting_wpm);
        UI_PrintStringSmallBold(val, 0, 0, 3);

    } else if (setting_step == 2) {
        // Step 3: Length
        UI_PrintStringSmallBold("3. Random Length", 0, 0, 2);
        
        sprintf(val, "   < %d chars >", setting_length);
        UI_PrintStringSmallBold(val, 0, 0, 3);
    }
    
    // --- Footer คำแนะนำปุ่ม (Line 5-7) ---
    // ปุ่มปรับค่า
    UI_PrintStringSmallBold("UP/DWN Adjust", 0, 0, 5);

    // ปุ่มไปต่อ/บันทึก
    if (setting_step < 2) {
        UI_PrintStringSmallBold("MENU Next >", 0, 0, 6);
    } else {
        UI_PrintStringSmallBold("MENU Save & Exit", 0, 0, 6);
    }

    UI_Flush();
}

// ==========================================
// 4. KEYBOARD HANDLING
// ==========================================

// ฟังก์ชันนี้ถูกเรียกเมื่อมีการกดปุ่ม
// key_code: รหัสปุ่ม (KEY_UP, KEY_DOWN, KEY_MENU, KEY_EXIT)
// event: กดสั้น/กดค้าง
void Morse_HandleKeys(int key_code, bool is_long_press) {
    
    switch (current_state) {
        
        // --- Logic สำหรับหน้า Main Menu ---
        case STATE_MAIN_MENU:
            if (key_code == KEY_UP) {
                // เลื่อนขึ้น
                if (menu_cursor > 0) {
                    menu_cursor--;
                } else {
                    menu_cursor = MENU_COUNT - 1; // วนไปล่างสุด
                }
            } 
            else if (key_code == KEY_DOWN) {
                // เลื่อนลง
                if (menu_cursor < MENU_COUNT - 1) {
                    menu_cursor++;
                } else {
                    menu_cursor = 0; // วนไปบนสุด
                }
            }
            else if (key_code == KEY_MENU) {
                // กดเลือกเมนู
                if (menu_cursor == 0) {
                    current_state = STATE_LIST; // ไปหน้า List
                } 
                else if (menu_cursor == 1) {
                    current_state = STATE_PRACTICE_SELECT; // ไปหน้า Practice
					menu_cursor = 0;
                }
                else if (menu_cursor == 2) {
                    current_state = STATE_SETTINGS; // ไปหน้า Settings
                }
            }
            else if (key_code == KEY_EXIT) {
                // ออกจากแอป (Code ส่วนหลักของวิทยุจะจัดการเอง หรือ return ออกไป)
            }
            break;

        // --- Placeholder หน้าอื่นๆ (เดี๋ยวค่อยมาเติม) ---
        case STATE_LIST:
            if (key_code == KEY_EXIT) {
                current_state = STATE_MAIN_MENU;
            }
            else if (key_code == KEY_UP) {
                if (list_cursor > 0) {
                    list_cursor--;
                    // ถ้าหลุดขอบบน ให้ดึงจอกลับ (ถอยหลังทีละ 2 เพื่อรักษาระนาบคู่ขนาน)
                    if (list_cursor < list_scroll_index) {
                        list_scroll_index -= 2; 
                        if (list_scroll_index < 0) list_scroll_index = 0;
                    }
                } else {
                    // Wraparound ไปตัวสุดท้าย
                    list_cursor = 41;
                    // คำนวณหน้าสุดท้าย (42 - 8 = 34)
                    list_scroll_index = 34; 
                    // ตรวจสอบให้เป็นเลขคู่เสมอ (เพื่อไม่ให้คอลัมน์สลับด้าน)
                    if (list_scroll_index % 2 != 0) list_scroll_index--;
                }
            }
            else if (key_code == KEY_DOWN) {
                if (list_cursor < 41) {
                    list_cursor++;
                    // ถ้าหลุดขอบล่าง ให้ดันจอลง (เดินหน้าทีละ 2)
                    if (list_cursor >= list_scroll_index + LIST_ITEMS_PER_PAGE) {
                         list_scroll_index += 2; 
                    }
                } else {
                    // Wraparound ไปตัวแรก
                    list_cursor = 0;
                    list_scroll_index = 0;
                }
            }
			else if (key_code == KEY_MENU) {
                // ดึงรหัสจากตาราง ตามตำแหน่ง cursor ปัจจุบัน
                const char* code_to_play = morse_table[list_cursor].code;
                
                // เรียกฟังก์ชันเล่นเสียง
                Morse_PlaySound(code_to_play);
            }
            break;

        case STATE_PRACTICE_SELECT:
            if (key_code == KEY_EXIT) {
                current_state = STATE_MAIN_MENU;
                menu_cursor = 1; 
            }
            // --- แก้ไขปุ่ม UP (ให้วนไปล่างสุด) ---
            else if (key_code == KEY_UP) {
                if (menu_cursor > 0) {
                    menu_cursor--;
                } else {
                    menu_cursor = 3; // ถ้าอยู่บนสุด (0) ให้วนไปล่างสุด (3)
                }
            }
            // --- แก้ไขปุ่ม DOWN (ให้วนไปบนสุด) ---
            else if (key_code == KEY_DOWN) {
                if (menu_cursor < 3) {
                    menu_cursor++;
                } else {
                    menu_cursor = 0; // ถ้าอยู่ล่างสุด (3) ให้วนไปบนสุด (0)
                }
            }
            else if (key_code == KEY_MENU) {
                if (menu_cursor == 2) {
                    // [เพิ่ม] เข้า Echo Mode
                    current_state = STATE_PRACTICE_ECHO;
                    
                    // Reset ค่า
                    memset(send_buf, 0, sizeof(send_buf));
                    memset(echo_input_code, 0, sizeof(echo_input_code));
                    echo_score_correct = 0;
                    echo_score_total = 0;
                    echo_input_char = 0;
                    
                    // สุ่มโจทย์ข้อแรก
                    int r = my_rand() % 42;
                    echo_target_char = morse_table[r].character;
                    
                    AUDIO_AudioPathOn();
                    reload_send_screen = true; // ใช้ตัวแปรช่วยวาดจอตัวเดิม
					
					is_quiz_mode = false; // ไม่ใช่ Quiz (ถือเป็นโหมดเคาะ)
                } else if (menu_cursor == 3) {
                    // เข้าโหมด Send Practice
                    current_state = STATE_PRACTICE_SEND;
                    
                    // Reset ค่าต่างๆ
                    memset(send_buf, 0, sizeof(send_buf));
                    memset(send_history, 0, sizeof(send_history));
					memset(last_code_buf, 0, sizeof(last_code_buf));
                    is_key_pressed = false;
                    key_timer = 0;

                    // [แก้ไข] เปิดระบบเสียงค้างไว้เลย เพื่อความไวสูงสุด!
                    AUDIO_AudioPathOn();
					
					// [เพิ่ม] สั่งให้วาดหน้าจอใหม่แน่นอน
                    reload_send_screen = true;
					
					is_quiz_mode = false; // ไม่ใช่ Quiz
                }
				else {
                    // โหมดเดิม (0, 1)
                    is_timed_mode = (menu_cursor == 1);
                    score_correct = 0;
                    score_total = 0;
					
					is_quiz_mode = true;
					
                    GenerateQuiz(); 
                    current_state = STATE_GAME_LISTEN;
                }
            }
            break;
		
		case STATE_GAME_LISTEN:
            // สถานะนี้เราฟังเสียงอย่างเดียว ไม่รับปุ่มกด
            break;
			
        case STATE_GAME_ANSWER:
            if (key_code == KEY_EXIT) {
                // ออกกลางคัน ไปหน้าสรุปผล
                current_state = STATE_GAME_SCORE;
            }
            else if (key_code == KEY_UP) {
                if(quiz_cursor > 0) quiz_cursor--;
                else quiz_cursor = 3;
            }
            else if (key_code == KEY_DOWN) {
                if(quiz_cursor < 3) quiz_cursor++;
                else quiz_cursor = 0;
            }
            else if (key_code == KEY_MENU) {
                // ตอบคำถาม
                score_total++;
                if (quiz_cursor == quiz_correct_index) {
                    score_correct++;
                }
                current_state = STATE_GAME_RESULT; // ไปหน้าเฉลย
            }
            break;

        case STATE_GAME_RESULT:
            if (key_code == KEY_EXIT) {
                current_state = STATE_GAME_SCORE;
            }
            else if (key_code == KEY_MENU) {
                // ข้อถัดไป
                GenerateQuiz();
                current_state = STATE_GAME_LISTEN;
            }
            break;

        case STATE_GAME_SCORE:
             if (key_code == KEY_EXIT) {
                current_state = STATE_MAIN_MENU;
             }
             break;

		case STATE_PRACTICE_SEND:
            if (key_code == KEY_EXIT) {
                // ออกจากโหมดฝึกส่ง
                current_state = STATE_PRACTICE_SELECT;
                
                // [แก้ไข] ปิดเสียงเมื่อออกจากหน้านี้
                BK4819_EnterTxMute();
                BK4819_PlayTone(0, false);
                AUDIO_AudioPathOff();
            }
            break;
			
		case STATE_PRACTICE_ECHO:
            // ไม่ต้องทำอะไร เพราะเราจัดการปุ่มใน App Loop แล้ว
            if (key_code == KEY_EXIT) {
				// ออกจากโหมด Echo -> ไปหน้าสรุปคะแนน
                score_correct = echo_score_correct;
                score_total = echo_score_total;
				
				current_state = STATE_GAME_SCORE;
				
				// ปิดเสียง
                BK4819_EnterTxMute();
                BK4819_PlayTone(0, false);
                AUDIO_AudioPathOff();
				
                menu_cursor = 1; 
            }
            break;
			
        case STATE_SETTINGS:
            if (key_code == KEY_EXIT) {
                // ยกเลิกและกลับเมนูหลัก (Reset step กลับไป 0 เผื่อเข้ามาใหม่)
                setting_step = 0;
                current_state = STATE_MAIN_MENU;
            }
            else if (key_code == KEY_MENU) {
                // ไปขั้นตอนถัดไป
                setting_step++;
                
                // ถ้าครบ 3 ขั้นตอนแล้ว (0, 1, 2 -> 3) ให้บันทึกและออก
                if (setting_step > 2) {
                    setting_step = 0; // Reset รอไว้ครั้งหน้า
                    current_state = STATE_MAIN_MENU;
                    
                    // (Optional) เล่นเสียง Beep สั้นๆ ยืนยันการบันทึก
                    AUDIO_AudioPathOn();
                    BK4819_PlayTone(1000, true);
                    BK4819_ExitTxMute();
                    SYSTEM_DelayMs(100);
                    BK4819_EnterTxMute();
                    BK4819_PlayTone(0, false);
                    AUDIO_AudioPathOff();
                }
            }
            else if (key_code == KEY_DOWN) {
                // ปรับค่าเพิ่มขึ้น ตามขั้นตอนปัจจุบัน
                if (setting_step == 0) {
                    // Tone: เพิ่มทีละ 10, สูงสุด 1000
                    if (setting_tone < 1000) setting_tone += 10;
                    
                    // (แถม) เล่นเสียง Preview สั้นๆ
                    AUDIO_AudioPathOn();
                    BK4819_PlayTone(setting_tone, true);
                    BK4819_ExitTxMute();
                    SYSTEM_DelayMs(50); // สั้นๆ พอ
                    BK4819_EnterTxMute();
                    BK4819_PlayTone(0, false);
                    AUDIO_AudioPathOff();
                } 
                else if (setting_step == 1) {
                    // WPM: เพิ่มทีละ 1, สูงสุด 50
                    if (setting_wpm < 50) setting_wpm++;
                }
                else if (setting_step == 2) {
                    // Length: เพิ่มทีละ 1, สูงสุด 10
                    if (setting_length < 10) setting_length++;
                }
            }
            else if (key_code == KEY_UP) {
                // ปรับค่าลดลง
                if (setting_step == 0) {
                    // Tone: ลดทีละ 10, ต่ำสุด 300
                    if (setting_tone > 300) setting_tone -= 10;
                    
                     // (แถม) เล่นเสียง Preview
                    AUDIO_AudioPathOn();
                    BK4819_PlayTone(setting_tone, true);
                    BK4819_ExitTxMute();
                    SYSTEM_DelayMs(50);
                    BK4819_EnterTxMute();
                    BK4819_PlayTone(0, false);
                    AUDIO_AudioPathOff();
                } 
                else if (setting_step == 1) {
                    // WPM: ลดทีละ 1, ต่ำสุด 5
                    if (setting_wpm > 5) setting_wpm--;
                }
                else if (setting_step == 2) {
                    // Length: ลดทีละ 1, ต่ำสุด 1
                    if (setting_length > 1) setting_length--;
                }
            }
            break;
    }
}

// ==========================================
// 5. MAIN APP LOOP (แก้ไขใหม่)
// ==========================================

void Morse_App_Loop() {
    // 1. กำหนดสถานะเริ่มต้น
    current_state = STATE_MAIN_MENU;
    bool is_app_running = true;
    
    // 2. เคลียร์ปุ่มค้าง (Debounce)
    while(KEYBOARD_Poll() != KEY_INVALID) SYSTEM_DelayMs(10);

    // 3. เริ่ม Loop หลักของ App
    while(is_app_running) {
        
        // A. รับค่าปุ่มกด
        KEY_Code_t key = KEYBOARD_Poll();
        
        if (key != KEY_INVALID) {
            // จัดการปุ่มกด
            if (key == KEY_EXIT && current_state == STATE_MAIN_MENU) {
                // ถ้ากด EXIT ที่หน้าเมนูหลัก ให้จบการทำงาน Loop นี้ (กลับไป Launcher)
                is_app_running = false;
            } else {
                // ส่งปุ่มไปให้ Logic ของ App จัดการ
                Morse_HandleKeys(key, false);
            }

			// [แก้ไข] ใส่เงื่อนไขดักไว้
            // ถ้าอยู่ในโหมดฝึกส่ง (Send/Echo) ห้าม Delay นาน! (เดี๋ยวคีย์สะดุด)
            if (current_state == STATE_PRACTICE_SEND || current_state == STATE_PRACTICE_ECHO) {
                 // ไม่ต้อง Delay หรือ Delay น้อยมากๆ
            } else {
                 // โหมดอื่น Delay ปกติเพื่อกันปุ่มเบิ้ล
                 SYSTEM_DelayMs(100); 
            }
			
			// ถ้าเป็นปุ่ม MENU หรือ EXIT ให้รอจนกว่าจะปล่อยมือ (ป้องกันการทะลุเมนู)
            if (key == KEY_MENU || key == KEY_EXIT) {
                // รอจนกว่าค่าปุ่มจะหายไป (ปล่อยมือ)
                while(KEYBOARD_Poll() == key) {
                    SYSTEM_DelayMs(10); 
                }
                // แถม Delay อีกนิดกันสัญญาณกระเพื่อมตอนปล่อย
                SYSTEM_DelayMs(50); 
            } else {
                // ปุ่มอื่นๆ (เช่น UP/DOWN) ให้หน่วงเวลาปกติ (เพื่อให้กดค้างเลื่อนเร็วๆ ได้)
                SYSTEM_DelayMs(100); 
            }
            // ------------------------------------
            
            // หน่วงเวลาเล็กน้อยหลังกดปุ่ม
            //SYSTEM_DelayMs(100);
        }

        // B. วาดหน้าจอตามสถานะปัจจุบัน
        switch (current_state) {
            case STATE_MAIN_MENU:
                Render_MainMenu();
                break;
                
            case STATE_LIST:
				Render_MorseList();
                break;
            
            case STATE_PRACTICE_SELECT: 
                Render_PracticeSelect(); 
                break;

            case STATE_GAME_LISTEN:
                UI_ClearScreen();
                
                // แสดงข้อความตัวหนา ไว้ที่บรรทัด 3 (กึ่งกลางจอ)
                // ผมเคาะวรรคข้างหน้าเพิ่มให้ เพื่อจัดให้มันดูอยู่กึ่งกลางจอ (Manual Center)
                UI_PrintStringSmallBold("     LISTEN...", 0, 0, 3);
                
                UI_Flush();
                
                // เล่นเสียงโจทย์ (โปรแกรมจะหยุดรอตรงนี้จนกว่าเสียงจะจบ)
                PlayQuizSound();
                
                // เล่นจบแล้ว ไปหน้าตอบคำถามทันที
                current_state = STATE_GAME_ANSWER;
                break;

            case STATE_GAME_ANSWER:
                Render_GameAnswer();
                
                // --- Logic จับเวลา ---
                if (is_timed_mode && !is_time_up) {
                    if (game_timer > 0) {
                        game_timer--; // ลดเวลาลงทีละ tick (20ms)
                    } else {
                        // หมดเวลา!
                        is_time_up = true;
                        score_total++; // นับว่าเป็น 1 ข้อที่ทำแล้ว
                        // (ไม่บวกคะแนน)
                        current_state = STATE_GAME_RESULT;
                        
                        // เล่นเสียงเตือนหมดเวลาสั้นๆ
                        AUDIO_AudioPathOn();
                        BK4819_PlayTone(300, true); // เสียงต่ำ
                        BK4819_ExitTxMute();
                        SYSTEM_DelayMs(300);
                        BK4819_EnterTxMute();
                        BK4819_PlayTone(0, false);
                        AUDIO_AudioPathOff();
                    }
                }
                break;

            case STATE_GAME_RESULT: 
                Render_GameResult(); 
                break;
                
            case STATE_GAME_SCORE: 
                Render_ScoreSummary(); 
                break;

			case STATE_PRACTICE_SEND: {
                // 1. วาดหน้าจอ (Render)
                if (reload_send_screen) {
                    Render_SendPractice();
                    reload_send_screen = false;
                }

                // 2. อ่านปุ่ม
                KEY_Code_t key = KEYBOARD_Poll();

                // 3. ปุ่ม EXIT
                if (key == KEY_EXIT) {
                    current_state = STATE_PRACTICE_SELECT;
                    BK4819_EnterTxMute();
                    BK4819_PlayTone(0, false);
                    AUDIO_AudioPathOff();
                    break;
                }

                uint32_t dot_len_ms = 1200 / setting_wpm;

                // --- A. ปุ่ม 7 (Dot) / 9 (Dash) [ปรับปรุงใหม่: Blocking Loop] ---
                if (key == KEY_7 || key == KEY_9) {
                    // กำหนดค่าตามปุ่มที่กด
                    uint32_t duration = (key == KEY_7) ? dot_len_ms : (dot_len_ms * 3);
                    char symbol = (key == KEY_7) ? '.' : '-';

                    // [สำคัญ] ใช้ do-while วนลูปตราบเท่าที่ยังกดปุ่มเดิมค้างไว้
                    // จะทำให้จังหวะต่อเนื่อง ไหลลื่น เหมือน Keyer ของจริง
                    do {
                        // 1. Tone ON
                        BK4819_PlayTone(setting_tone, true);
                        BK4819_ExitTxMute();
                        SYSTEM_DelayMs(duration); // ความยาวเสียงเป๊ะ

                        // 2. Tone OFF
                        BK4819_EnterTxMute();
                        BK4819_PlayTone(0, false);

                        // 3. เก็บข้อมูล
                        if (strlen(send_buf) < 9) {
                            int len = strlen(send_buf);
                            send_buf[len] = symbol;
                            send_buf[len+1] = '\0';
                        }

                        // 4. Gap (เว้นวรรคระหว่างเสียง)
                        // ตรงนี้สำคัญ! การใส่ Delay ในลูปนี้ ทำให้ Gap เท่ากับ dot_len_ms เป๊ะๆ
                        SYSTEM_DelayMs(dot_len_ms);

                        // เช็คปุ่มซ้ำ (ถ้ายังกดค้าง ก็วนไปเล่นเสียงต่อไปทันที)
                    } while (KEYBOARD_Poll() == key);

                    // เมื่อปล่อยปุ่มแล้ว ให้รีเซ็ตตัวจับเวลา เพื่อรอตัดจบตัวอักษร
                    key_timer = 0; 
                }
                
                // --- B. ปุ่ม Side Key (Manual) - แบบ Blocking Loop ---
                else if (key == KEY_SIDE2) {
                    // >> เริ่มกดปุ่ม <<
                    BK4819_PlayTone(setting_tone, true);
                    BK4819_ExitTxMute();
                    
                    uint32_t press_duration = 0;
                    
                    // [สำคัญ] วนลูปอยู่ตรงนี้จนกว่าจะปล่อยมือ!
                    // ไม่สนใจการวาดจอ ไม่สนใจอย่างอื่น สนแค่จับเวลา
                    while (KEYBOARD_Poll() == KEY_SIDE2) {
                        SYSTEM_DelayMs(5); // หน่วงเวลา 5ms เป๊ะๆ
                        press_duration++;  // 1 หน่วย = 5ms
                    }
                    
                    // >> ปล่อยปุ่มแล้ว <<
                    BK4819_EnterTxMute();
                    BK4819_PlayTone(0, false);
                    
                    // ตัดสินใจ Dot/Dash
                    // Threshold: 120ms / 5ms = 24 หน่วย
                    // ถ้ากดน้อยกว่า 24 (120ms) เป็น Dot, มากกว่าเป็น Dash
                    if (strlen(send_buf) < 9) {
                        if (press_duration < 24) { 
                            strcat(send_buf, "."); 
                        } else {
                            strcat(send_buf, "-"); 
                        }
                    }
                    
                    key_timer = 0; // รีเซ็ตตัวนับ Gap เพราะเพิ่งปล่อยมือ
                }
                
                // --- C. Logic ช่วงปล่อยมือ (Gap Detection) ---
                // ส่วนนี้จะทำงานเฉพาะตอน "ไม่ได้กดปุ่มอะไรเลย"
                else {
                    key_timer++; // นับเวลา Gap ไปเรื่อยๆ
                    
                    // Gap Threshold: จบตัวอักษร
                    // ประมาณ 300-400ms (70-80 หน่วย คูณ 5ms)
                    if (key_timer > 25 && strlen(send_buf) > 0) {
                        // 1. แปลผล
                        char c = DecodeMorseStr(send_buf);
                        
                        // 2. เก็บลง History
                        int hLen = strlen(send_history);
                        if (hLen < 14) {
                            send_history[hLen] = c;
                            send_history[hLen+1] = '\0';
                        } else {
                            for(int i=0; i<14; i++) send_history[i] = send_history[i+1];
                            send_history[13] = c;
                        }
                        
                        // 3. ก๊อปปี้ไป Last Code Display [แก้แล้ว]
                        strcpy(last_code_buf, send_buf);
                        
                        // 4. สั่งวาดจอ
                        reload_send_screen = true; 
                        
                        // 5. เคลียร์ Buffer (เตรียมตัวต่อไป)
                        memset(send_buf, 0, sizeof(send_buf));
                        
                        key_timer = 0; // หยุดนับ
                    }
                }

                // หน่วงเวลาลูปหลัก (เพื่อให้ key_timer ของ Gap ทำงานแม่นยำประมาณหนึ่ง)
                SYSTEM_DelayMs(5); 
                break;
			}
			
			case STATE_PRACTICE_ECHO: {
                // 1. วาดหน้าจอ
                if (reload_send_screen) {
                    Render_PracticeEcho();
                    reload_send_screen = false;
                }

                // 2. ใช้ปุ่มจาก Main Loop (ตัวแปร key ด้านบน) แทนการ Poll ใหม่!
                // ลบ KEY_Code_t key = KEYBOARD_Poll(); ออกไปเลย
                
                // ไม่ต้องเช็ค KEY_EXIT ตรงนี้แล้ว (เพราะ HandleKeys จัดการเปลี่ยน state ให้แล้ว)
                // ถ้า state เปลี่ยน มันจะ break loop ออกไปเอง

				// ไม่ต้องเช็ค KEY_EXIT ตรงนี้แล้ว (เพราะ HandleKeys จัดการเปลี่ยน state ให้แล้ว)
                // ถ้า state เปลี่ยน มันจะ break loop ออกไปเอง
				
                // 2. ปุ่ม EXIT -> ไปหน้าสรุปคะแนน (ใช้ STATE_GAME_RESULT แก้ขัดไปก่อน)
/*                 if (key == KEY_EXIT) {
                    // ส่งคะแนนไปหน้าสรุปผล
                    score_correct = echo_score_correct;
                    score_total = echo_score_total;
                    is_time_up = false; // หลอกว่าเป็นโหมดปกติ
                    quiz_cursor = 0; quiz_correct_index = 0; // ค่าหลอก
                    
                    // ไปหน้า Result
                    current_state = STATE_GAME_SCORE; 
                    
                    BK4819_EnterTxMute();
                    BK4819_PlayTone(0, false);
                    AUDIO_AudioPathOff();
                    break;
                } */

                uint32_t dot_len_ms = 1200 / setting_wpm;

                // --- A. ปุ่ม 7/9 (Auto) ---
                if (key == KEY_7 || key == KEY_9) {
                    // ... (Copy โค้ดส่วนนี้จาก STATE_PRACTICE_SEND มาแปะได้เลยครับ เหมือนกันเป๊ะ) ...
                    uint32_t duration = (key == KEY_7) ? dot_len_ms : (dot_len_ms * 3);
                    char symbol = (key == KEY_7) ? '.' : '-';
                    do {
                        BK4819_PlayTone(setting_tone, true); BK4819_ExitTxMute();
                        SYSTEM_DelayMs(duration);
                        BK4819_EnterTxMute(); BK4819_PlayTone(0, false);
                        if (strlen(send_buf) < 9) {
                            int len = strlen(send_buf);
                            send_buf[len] = symbol; send_buf[len+1] = '\0';
                        }
                        SYSTEM_DelayMs(dot_len_ms);
                    } while (KEYBOARD_Poll() == key);
                    key_timer = 0; 
                }
                
                // --- B. Side Key (Manual) ---
                else if (key == KEY_SIDE2) {
                    // ... (Copy โค้ดส่วนนี้จาก STATE_PRACTICE_SEND มาแปะได้เลยครับ เหมือนกันเป๊ะ) ...
                    BK4819_PlayTone(setting_tone, true); BK4819_ExitTxMute();
                    uint32_t press_duration = 0;
                    while (KEYBOARD_Poll() == KEY_SIDE2) {
                        SYSTEM_DelayMs(5); press_duration++; 
                    }
                    BK4819_EnterTxMute(); BK4819_PlayTone(0, false);
                    if (strlen(send_buf) < 9) {
                        if (press_duration < 24) strcat(send_buf, "."); 
                        else strcat(send_buf, "-"); 
                    }
                    key_timer = 0; 
                }
                
                // --- C. ตรวจคำตอบ (Gap Detection) ---
                else {
                    key_timer++;
                    
                    // Gap Threshold (ประมาณ 350ms)
                    if (key_timer > 35 && strlen(send_buf) > 0) {
                        // 1. แปลผลสิ่งที่กดมา
                        char c = DecodeMorseStr(send_buf);
                        
                        // 2. บันทึกผลเพื่อแสดงหน้าจอ
                        echo_input_char = c;
                        strcpy(echo_input_code, send_buf);
                        
                        // 3. ตรวจคำตอบ & นับคะแนน (เทียบกับ Target ปัจจุบันก่อน!)
                        echo_score_total++;
                        if (echo_input_char == echo_target_char) {
                            echo_score_correct++;
                            // (แถม) เสียงติ๊ดสั้นๆ ยืนยันว่าถูก
                            BK4819_PlayTone(1200, true); BK4819_ExitTxMute();
                            SYSTEM_DelayMs(100);
                            BK4819_EnterTxMute(); BK4819_PlayTone(0, false);
                        } else {
                            // (แถม) เสียงทุ้มยืนยันว่าผิด
                            BK4819_PlayTone(300, true); BK4819_ExitTxMute();
                            SYSTEM_DelayMs(100);
                            BK4819_EnterTxMute(); BK4819_PlayTone(0, false);
                        }

                        // 4. [สำคัญ] สั่งวาดจอ "ทันที" เพื่อโชว์ว่าถูกหรือผิด (ของข้อนี้)
                        Render_PracticeEcho(); 

                        // 5. หน่วงเวลาให้คนเล่นดูผลงานตัวเอง (เช่น 1 วินาที)
                        SYSTEM_DelayMs(1000);
                        
                        // -----------------------------------------
                        // เริ่มรอบใหม่ (Next Round)
                        // -----------------------------------------

                        // 6. สุ่มโจทย์ข้อใหม่
                        int r = my_rand() % 42;
                        echo_target_char = morse_table[r].character;
                        
                        // 7. ล้างค่า Input ทิ้ง (เพื่อให้หน้าจอแสดงเป็น [ ? ] รอรับค่าใหม่)
                        echo_input_char = 0;
                        memset(echo_input_code, 0, sizeof(echo_input_code));
                        memset(send_buf, 0, sizeof(send_buf));
                        
                        // 8. สั่งให้วาดจอรอบหน้า (จะโชว์โจทย์ใหม่ และช่องว่าง)
                        reload_send_screen = true; 
                        
                        key_timer = 0;
                    }
                }
                SYSTEM_DelayMs(5);
                break;
			}
			
            case STATE_SETTINGS:
                Render_Settings();
                break;
        }
        
        // C. หน่วงเวลาเพื่อไม่ให้ CPU ทำงานหนักเกินไป
        SYSTEM_DelayMs(20);
    }
}
