#include "callsign.h"
#include <string.h>
#include "driver/st7565.h"
#include "driver/keyboard.h"
#include "driver/system.h"
#include "ui/helper.h"

// --- 1. โครงสร้างข้อมูล ---
typedef struct {
    const char start[4];   
    const char end[4];     
    const char country[16]; // ชื่อประเทศ (ย่อบางชื่อเพื่อให้พอดีจอ)
} CallSignRange;

// --- 2. ฐานข้อมูล ITU Prefix ทั่วโลก ---
static const CallSignRange DATABASE[] = {
    // --- A ---
    {"AAA", "ALZ", "USA"},
    {"AMA", "AOZ", "Spain"},
    {"APA", "ASZ", "Pakistan"},
    {"ATA", "AWZ", "India"},
    {"AXA", "AXZ", "Australia"},
    {"AYA", "AZZ", "Argentina"},
    {"A2A", "A2Z", "Botswana"},
    {"A3A", "A3Z", "Tonga"},
    {"A4A", "A4Z", "Oman"},
    {"A5A", "A5Z", "Bhutan"},
    {"A6A", "A6Z", "UAE"},
    {"A7A", "A7Z", "Qatar"},
    {"A9A", "A9Z", "Bahrain"},

    // --- B ---
    // จีน (China) ช่วงแรก BAA - BUZ
    {"BAA", "BUZ", "China"},
    
    // ไต้หวัน (Taiwan) แทรกตรงกลาง BVA - BVZ
    {"BVA", "BVZ", "Taiwan"}, 
    
    // จีน (China) ช่วงหลัง BWA - BZZ
    {"BWA", "BZZ", "China"},
    
    // หมายเหตุ: BYA, BZA ที่เคยแยกไว้ ไม่ต้องใส่แล้ว 
    // เพราะมันรวมอยู่ใน BWA-BZZ แล้วครับ,

    // --- C ---
    {"CAA", "CEZ", "Chile"},
    {"CFA", "CKZ", "Canada"},
    {"CLA", "CMZ", "Cuba"},
    {"CNA", "CNZ", "Morocco"},
    {"COA", "COZ", "Cuba"},
    {"CPA", "CPZ", "Bolivia"},
    {"CQA", "CUZ", "Portugal"},
    {"CVA", "CXZ", "Uruguay"},
    {"CYA", "CZZ", "Canada"},
    {"C2A", "C2Z", "Nauru"},
    {"C3A", "C3Z", "Andorra"},
    {"C5A", "C5Z", "Gambia"},
    {"C6A", "C6Z", "Bahamas"},
    {"C8A", "C9Z", "Mozambique"},

    // --- D ---
    {"DAA", "DRZ", "Germany"},
    {"DSA", "DTZ", "South Korea"},
    {"DUA", "DZZ", "Philippines"},
    {"D2A", "D3Z", "Angola"},
    {"D4A", "D4Z", "Cape Verde"},
    {"D6A", "D6Z", "Comoros"},

    // --- E ---
    {"EAA", "EHZ", "Spain"},
    {"EIA", "EJZ", "Ireland"},
    {"EKA", "EKZ", "Armenia"},
    {"ELA", "ELZ", "Liberia"},
    {"EMA", "EOZ", "Iran"},
    {"EPA", "EQZ", "Iran"},
    {"ERA", "ERZ", "Moldova"},
    {"ESA", "ESZ", "Estonia"},
    {"ETA", "ETZ", "Ethiopia"},
    {"EUA", "EWZ", "Belarus"},
    {"EXA", "EXZ", "Kyrgyzstan"},
    {"EYA", "EYZ", "Tajikistan"},
    {"EZA", "EZZ", "Turkmenistan"},
    {"E2A", "E2Z", "Thailand"}, // <-- THAILAND
    {"E3A", "E3Z", "Eritrea"},
    {"E4A", "E4Z", "Palestine"},
    {"E5A", "E5Z", "Cook Is."},
    {"E7A", "E7Z", "Bosnia"},

    // --- F ---
    {"FAA", "FZZ", "France"},

    // --- G ---
    {"GAA", "GZZ", "UK"},

    // --- H ---
    {"HAA", "HAZ", "Hungary"},
    {"HBA", "HBZ", "Switzerland"},
    {"HCA", "HDZ", "Ecuador"},
    {"HEA", "HEZ", "Switzerland"},
    {"HFA", "HFZ", "Poland"},
    {"HGA", "HGZ", "Hungary"},
    {"HHA", "HHZ", "Haiti"},
    {"HIA", "HIZ", "Dominican Rep"},
    {"HJA", "HKZ", "Colombia"},
    {"HLA", "HLZ", "South Korea"},
    {"HMA", "HMZ", "North Korea"},
    {"HNA", "HNZ", "Iraq"},
    {"HOA", "HPZ", "Panama"},
    {"HQA", "HRZ", "Honduras"},
    {"HSA", "HSZ", "Thailand"}, // <-- THAILAND
    {"HTA", "HTZ", "Nicaragua"},
    {"HUA", "HUZ", "El Salvador"},
    {"HVA", "HVZ", "Vatican"},
    {"HWA", "HYZ", "France"},
    {"HZA", "HZZ", "Saudi Arabia"},

    // --- I ---
    {"IAA", "IZZ", "Italy"},

    // --- J ---
    {"JAA", "JSZ", "Japan"},
    {"JTA", "JVZ", "Mongolia"},
    {"JWA", "JXZ", "Norway"},
    {"JYA", "JYZ", "Jordan"},
    {"JZZ", "JZZ", "Indonesia"},
    {"J2A", "J2Z", "Djibouti"},
    {"J3A", "J3Z", "Grenada"},
    {"J4A", "J4Z", "Greece"},
    {"J5A", "J5Z", "Guinea-Bissau"},
    {"J6A", "J6Z", "St Lucia"},
    {"J7A", "J7Z", "Dominica"},
    {"J8A", "J8Z", "St Vincent"},

    // --- K ---
    {"KAA", "KZZ", "USA"},

    // --- L ---
    {"LAA", "LNZ", "Norway"},
    {"LOA", "LWZ", "Argentina"},
    {"LXA", "LXZ", "Luxembourg"},
    {"LYA", "LYZ", "Lithuania"},
    {"LZA", "LZZ", "Bulgaria"},
    {"L2A", "L9Z", "Argentina"},

    // --- M ---
    {"MAA", "MZZ", "UK"},

    // --- N ---
    {"NAA", "NZZ", "USA"},

    // --- O ---
    {"OAA", "OCZ", "Peru"},
    {"ODA", "ODZ", "Lebanon"},
    {"OEA", "OEZ", "Austria"},
    {"OFA", "OJZ", "Finland"},
    {"OKA", "OLZ", "Czech Rep"},
    {"OMA", "OMZ", "Slovakia"},
    {"ONA", "OTZ", "Belgium"},
    {"OUA", "OZZ", "Denmark"},

    // --- P ---
    {"PAA", "PIZ", "Netherlands"},
    {"PJA", "PJZ", "Netherlands Ant"},
    {"PKA", "POZ", "Indonesia"},
    {"PPA", "PYZ", "Brazil"},
    {"PZA", "PZZ", "Suriname"},
    {"P2A", "P2Z", "Papua N.Guinea"},
    {"P3A", "P3Z", "Cyprus"},
    {"P4A", "P4Z", "Aruba"},
    {"P5A", "P9Z", "North Korea"},

    // --- R ---
    {"RAA", "RZZ", "Russia"},

    // --- S ---
    {"SAA", "SMZ", "Sweden"},
    {"SNA", "SRZ", "Poland"},
    {"SSA", "SSZ", "Egypt"}, // หรือ Sudan ตาม ITU บางช่วง
    {"STA", "STZ", "Sudan"},
    {"SUA", "SUZ", "Egypt"},
    {"SVA", "SZZ", "Greece"},
    {"S2A", "S3Z", "Bangladesh"},
    {"S5A", "S5Z", "Slovenia"},
    {"S6A", "S6Z", "Singapore"},
    {"S7A", "S7Z", "Seychelles"},
    {"S8A", "S8Z", "South Africa"},
    {"S9A", "S9Z", "Sao Tome"},

    // --- T ---
    {"TAA", "TCZ", "Turkey"},
    {"TDA", "TDZ", "Guatemala"},
    {"TEA", "TEZ", "Costa Rica"},
    {"TFA", "TFZ", "Iceland"},
    {"TGA", "TGZ", "Guatemala"},
    {"THA", "THZ", "France"},
    {"TIA", "TIZ", "Costa Rica"},
    {"TJA", "TJZ", "Cameroon"},
    {"TKA", "TKZ", "Corsica (Fr)"},
    {"TLA", "TLZ", "Central Africa"},
    {"TMA", "TMZ", "France"},
    {"TNA", "TNZ", "Congo"},
    {"TOA", "TQZ", "France"},
    {"TRA", "TRZ", "Gabon"},
    {"TSA", "TSZ", "Tunisia"},
    {"TTA", "TTZ", "Chad"},
    {"TUA", "TUZ", "Ivory Coast"},
    {"TVA", "TXZ", "France"},
    {"TYA", "TYZ", "Benin"},
    {"TZA", "TZZ", "Mali"},
    {"T2A", "T2Z", "Tuvalu"},
    {"T3A", "T3Z", "Kiribati"},
    {"T5A", "T5Z", "Somalia"},
    {"T7A", "T7Z", "San Marino"},
    {"T8A", "T8Z", "Palau"},

    // --- U ---
    {"UAA", "UIZ", "Russia"},
    {"UJA", "UMZ", "Uzbekistan"},
    {"UNA", "UQZ", "Kazakhstan"},
    {"URA", "UZZ", "Ukraine"},

    // --- V ---
    {"VAA", "VGZ", "Canada"},
    {"VHA", "VNZ", "Australia"},
    {"VOA", "VOZ", "Canada"},
    {"VPA", "VQZ", "UK (Islands)"},
    {"VRA", "VRZ", "Hong Kong"},
    {"VSA", "VSZ", "India"},
    {"VTA", "VWZ", "India"},
    {"VXA", "VYZ", "Canada"},
    {"VZA", "VZZ", "Australia"},
    {"V2A", "V2Z", "Antigua"},
    {"V3A", "V3Z", "Belize"},
    {"V4A", "V4Z", "St Kitts"},
    {"V5A", "V5Z", "Namibia"},
    {"V6A", "V6Z", "Micronesia"},
    {"V7A", "V7Z", "Marshall Is."},
    {"V8A", "V8Z", "Brunei"},

    // --- W ---
    {"WAA", "WZZ", "USA"},

    // --- X ---
    {"XAA", "XIZ", "Mexico"},
    {"XJA", "XOZ", "Canada"},
    {"XPA", "XPZ", "Denmark"},
    {"XQA", "XRZ", "Chile"},
    {"XSA", "XSZ", "China"},
    {"XTA", "XTZ", "Burkina Faso"},
    {"XUA", "XUZ", "Cambodia"},
    {"XVA", "XVZ", "Vietnam"},
    {"XWA", "XWZ", "Laos"},
    {"XXA", "XXZ", "Macau"},
    {"XYA", "XZZ", "Myanmar"},

    // --- Y ---
    {"YAA", "YAZ", "Afghanistan"},
    {"YBA", "YHZ", "Indonesia"},
    {"YIA", "YIZ", "Iraq"},
    {"YJA", "YJZ", "Vanuatu"},
    {"YKA", "YKZ", "Syria"},
    {"YLA", "YLZ", "Latvia"},
    {"YMA", "YMZ", "Turkey"},
    {"YNA", "YNZ", "Nicaragua"},
    {"YOA", "YRZ", "Romania"},
    {"YSA", "YSZ", "El Salvador"},
    {"YTA", "YUZ", "Serbia"},
    {"YVA", "YYZ", "Venezuela"},
    {"YZA", "YZZ", "Serbia"},

    // --- Z ---
    {"ZAA", "ZAZ", "Albania"},
    {"ZBA", "ZJZ", "UK (Overseas)"},
    {"ZKA", "ZMZ", "New Zealand"},
    {"ZNA", "ZOZ", "UK"},
    {"ZPA", "ZPZ", "Paraguay"},
    {"ZQA", "ZQZ", "UK"},
    {"ZRA", "ZUZ", "South Africa"},
    {"ZVA", "ZZZ", "Brazil"},
    {"Z2A", "Z2Z", "Zimbabwe"},
    {"Z3A", "Z3Z", "Macedonia"},
    {"Z8A", "Z8Z", "South Sudan"},

    // --- Numeric Prefixes (Numbers First) ---
    {"1AA", "1ZZ", "Sov. Mil. Order"},
    {"3AA", "3AZ", "Monaco"},
    {"3BA", "3BZ", "Mauritius"},
    {"3CA", "3CZ", "Eq. Guinea"},
    {"3DA", "3DM", "Eswatini"}, // Swaziland
    {"3DA", "3DZ", "Fiji"},
    {"3EA", "3FZ", "Panama"},
    {"3GA", "3GZ", "Chile"},
    {"3HA", "3UZ", "China"},
    {"3VA", "3VZ", "Tunisia"},
    {"3WA", "3WZ", "Vietnam"},
    {"3XA", "3XZ", "Guinea"},
    {"3YA", "3YZ", "Norway"},
    {"3ZA", "3ZZ", "Poland"},
    
    {"4AA", "4CZ", "Mexico"},
    {"4DA", "4IZ", "Philippines"},
    {"4JA", "4KZ", "Azerbaijan"},
    {"4LA", "4LZ", "Georgia"},
    {"4MA", "4MZ", "Venezuela"},
    {"4NA", "4OZ", "Serbia"}, // Montenegro also here
    {"4PA", "4SZ", "Sri Lanka"},
    {"4TA", "4TZ", "Peru"},
    {"4UA", "4UZ", "UN (Geneva)"},
    {"4VA", "4VZ", "Haiti"},
    {"4XA", "4ZZ", "Israel"},

    {"5AA", "5AZ", "Libya"},
    {"5BA", "5BZ", "Cyprus"},
    {"5CA", "5GZ", "Morocco"},
    {"5HA", "5IZ", "Tanzania"},
    {"5JA", "5KZ", "Colombia"},
    {"5LA", "5MZ", "Liberia"},
    {"5NA", "5OZ", "Nigeria"},
    {"5PA", "5QZ", "Denmark"},
    {"5RA", "5SZ", "Madagascar"},
    {"5TA", "5TZ", "Mauritania"},
    {"5UA", "5UZ", "Niger"},
    {"5VA", "5VZ", "Togo"},
    {"5WA", "5WZ", "Samoa"},
    {"5XA", "5XZ", "Uganda"},
    {"5YA", "5ZZ", "Kenya"},

    {"6AA", "6BZ", "Egypt"},
    {"6CA", "6CZ", "Syria"},
    {"6DA", "6JZ", "Mexico"},
    {"6KA", "6NZ", "South Korea"},
    {"6OA", "6OZ", "Somalia"},
    {"6PA", "6SZ", "Pakistan"},
    {"6TA", "6UZ", "Sudan"},
    {"6VA", "6WZ", "Senegal"},
    {"6XA", "6XZ", "Madagascar"},
    {"6YA", "6YZ", "Jamaica"},
    {"6ZA", "6ZZ", "Liberia"},

    {"7AA", "7IZ", "Indonesia"},
    {"7JA", "7NZ", "Japan"},
    {"7OA", "7OZ", "Yemen"},
    {"7PA", "7PZ", "Lesotho"},
    {"7QA", "7QZ", "Malawi"},
    {"7RA", "7RZ", "Algeria"},
    {"7SA", "7SZ", "Sweden"},
    {"7TA", "7YZ", "Algeria"},
    {"7ZA", "7ZZ", "Saudi Arabia"},

    {"8AA", "8IZ", "Indonesia"},
    {"8JA", "8NZ", "Japan"},
    {"8OA", "8OZ", "Botswana"},
    {"8PA", "8PZ", "Barbados"},
    {"8QA", "8QZ", "Maldives"},
    {"8RA", "8RZ", "Guyana"},
    {"8SA", "8SZ", "Sweden"},
    {"8TA", "8YZ", "India"},
    {"8ZA", "8ZZ", "Saudi Arabia"},

    {"9AA", "9AZ", "Croatia"},
    {"9BA", "9DZ", "Iran"},
    {"9EA", "9EZ", "Ethiopia"},
    {"9FA", "9GZ", "Ghana"},
    {"9HA", "9HZ", "Malta"},
    {"9IA", "9JZ", "Zambia"},
    {"9KA", "9KZ", "Kuwait"},
    {"9LA", "9LZ", "Sierra Leone"},
    {"9MA", "9MZ", "Malaysia"},
    {"9NA", "9NZ", "Nepal"},
    {"9OA", "9TZ", "Congo (Dem)"},
    {"9UA", "9UZ", "Burundi"},
    {"9VA", "9VZ", "Singapore"},
    {"9WA", "9WZ", "Malaysia"},
    {"9XA", "9XZ", "Rwanda"},
    {"9YA", "9ZZ", "Trinidad/Tobago"},
};

#define DB_SIZE (sizeof(DATABASE) / sizeof(CallSignRange))

// --- 3. ฟังก์ชันค้นหา ---
const char* FindCountry(const char* input) {
    if (strlen(input) < 1) return "";
    
    // แปลง Input ให้เป็นตัวพิมพ์ใหญ่ (ถ้ามี)
    // แต่ใน Main เราส่งตัวพิมพ์ใหญ่อยู่แล้ว

    for (int i = 0; i < DB_SIZE; i++) {
        size_t len = strlen(input);
        
        // เปรียบเทียบกับ Start
        int cmp_start = strncmp(input, DATABASE[i].start, len);
        // เปรียบเทียบกับ End
        int cmp_end = strncmp(input, DATABASE[i].end, len);

        // ต้องมากกว่าหรือเท่ากับ Start และ น้อยกว่าหรือเท่ากับ End
        if (cmp_start >= 0 && cmp_end <= 0) {
            return DATABASE[i].country;
        }
    }
    return "Unknown";
}

// --- 4. Main App ---

void APP_RunCallSign(void) {
    char input_buf[4] = "   "; 
    uint8_t cursor_pos = 0; 
    
    UI_DisplayClear();
    ST7565_BlitFullScreen();

    // รอปล่อยปุ่ม
    while (KEYBOARD_Poll() != KEY_INVALID) SYSTEM_DelayMs(10);

    bool is_running = true;
    bool need_redraw = true;

    while (is_running) {
        if (need_redraw) {
            UI_DisplayClear();
            
            UI_PrintStringSmallBold("DX CALLSIGN", 0, 0, 0);
            
            // วาด Input Box
            for(int i=0; i<2; i++) {
                char char_str[2] = {input_buf[i], '\0'};
                if (char_str[0] == ' ') char_str[0] = '_'; 
                
                if (i == cursor_pos) {
                     UI_PrintStringSmallBold("[", (i*20)+30, 0, 2);
                     UI_PrintStringSmallBold(char_str, (i*20)+38, 0, 2);
                     UI_PrintStringSmallBold("]", (i*20)+48, 0, 2);
                } else {
                     UI_PrintStringSmallBold(char_str, (i*20)+38, 0, 2);
                }
            }
            
            UI_PrintStringSmallBold("UP/Down:Char", 0, 0, 5);
			UI_PrintStringSmallBold("MENU:Pos", 0, 0, 6);

            // ค้นหา
            char search_str[4] = {0};
            int s_idx = 0;
            for(int i=0; i<2; i++) {
                if(input_buf[i] != ' ') {
                    search_str[s_idx++] = input_buf[i];
                }
            }
            search_str[s_idx] = '\0';
            
            if (s_idx > 0) {
                const char* result = FindCountry(search_str);
                // แสดงผลลัพธ์
                UI_PrintStringSmallBold(result, 0, 0, 3); 
            } else {
                UI_PrintStringSmallBold("Select Prefix", 0, 0, 3);
            }

            ST7565_BlitFullScreen();
            need_redraw = false;
        }

        KEY_Code_t key = KEYBOARD_Poll();
        
        if (key != KEY_INVALID) {
            // ปุ่มเปลี่ยนตัวอักษร
            if (key == KEY_DOWN) {
                char c = input_buf[cursor_pos];
                if (c == ' ') c = 'A';
                else if (c == 'Z') c = '0';
                else if (c == '9') c = ' ';
                else c++;
                input_buf[cursor_pos] = c;
                need_redraw = true;
            }
            else if (key == KEY_UP) {
                char c = input_buf[cursor_pos];
                if (c == ' ') c = '9';
                else if (c == 'A') c = ' ';
                else if (c == '0') c = 'Z';
                else c--;
                input_buf[cursor_pos] = c;
                need_redraw = true;
            }
            // ปุ่มเลื่อนตำแหน่ง
            else if (key == KEY_MENU) {
                cursor_pos++;
                if (cursor_pos > 1) cursor_pos = 0;
                need_redraw = true;
            }
            // ปุ่มออก
            else if (key == KEY_EXIT) {
                is_running = false;
            }
            
            SYSTEM_DelayMs(150);
        }
        SYSTEM_DelayMs(10); 
    }

    UI_DisplayClear();
    ST7565_BlitFullScreen();
}