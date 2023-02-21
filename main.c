#include <stdio.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include "ym2151.h"
//#include "music/vgmsample.h"
#include "music/01_Opening_Theme.h"
#include "music/02_Fighter_Captured.h"
#include "music/03_Scroll_Stage_BGM.h"
#include "music/04_Bonus_Stage_Start.h"
#include "music/05_Waltz.h"
#include "music/06_Tango.h"
#include "music/07_Big_Band_Jazz.h"
#include "music/08_Salsa.h"
#include "music/09_March.h"
#include "music/10_Perfect.h"
#include "music/11_Ending.h"
#include "music/12_Name_Entry.h"
#include "music/13_Game_Over.h"

uint8_t *vgmdata;
unsigned int vgmpos = 0;

bool vgmend = false;
unsigned long startTime;
unsigned long duration;

uint8_t getByte() {
    uint8_t ret = vgmdata[vgmpos];
    vgmpos++;
    return ret;
}

unsigned int read16() {
    return getByte() + (getByte() << 8);
}

void pause(long samples){
    duration = ((1000.0 / (44100.0 / (float)samples)) * 1000);
    startTime = esp_timer_get_time();
    if (samples > 441) {
        vTaskDelay(1); // 10ms
    }
}

void vgmplay() {
    if((esp_timer_get_time() - startTime) <= duration) {
        return;
    }

    uint8_t command = getByte();
    uint8_t reg;
    uint8_t dat;

    switch (command) {
        case 0x54:
            // YM2151
            reg = getByte();
            dat = getByte();
            writeData(reg, dat);
            break;
        case 0x61:
            pause(read16());
            break;
        case 0x62:
            pause(735);
            break;
        case 0x63:
            pause(882);
            break;
        case 0x66:
            vgmend = true;
            printf("vgm end\n");
            break;
        case 0x70:
        case 0x71:
        case 0x72:
        case 0x73:
        case 0x74:
        case 0x75:
        case 0x76:
        case 0x77:
        case 0x78:
        case 0x79:
        case 0x7A:
        case 0x7B:
        case 0x7C:
        case 0x7D:
        case 0x7E:
        case 0x7F:
            pause((command & 0x0f) + 1);
            break;
        default:
            break;
    }
}

const int songs = 13;
uint8_t *playlist[] = {
    //vgmdata00,
    vgmdata01_Opening_Theme,
    vgmdata02_Fighter_Captured,
    vgmdata03_Scroll_Stage_BGM,
    vgmdata04_Bonus_Stage_Start,
    vgmdata05_Waltz,
    vgmdata06_Tango,
    vgmdata07_Big_Band_Jazz,
    vgmdata08_Salsa,
    vgmdata09_March,
    vgmdata10_Perfect,
    vgmdata11_Ending,
    vgmdata12_Name_Entry,
    vgmdata13_Game_Over
};

void loop() {
    printf("loop(): %d\n", xPortGetCoreID()); // 1
    while (true) {
        for (int i=0; i<songs; i++) {
            vgmdata = playlist[i];
            vgmend = false;
            vgmpos = (vgmdata[0x37]<<24) | (vgmdata[0x36]<<16) | (vgmdata[0x35]<<8) | vgmdata[0x34];
            printf("start %d\n", i);
            while (!vgmend) {
                vgmplay();
            }
            printf("delay %d\n", i);
            vTaskDelay(2000 / portTICK_PERIOD_MS); // 2000ms
            printf("end %d\n", i);
        }
    }
}

void app_main() {
    TaskHandle_t ym2151taskHandle, vgmtaskHandle;
    printf("app_main():%d\n", xPortGetCoreID()); // 0
    init();
    xTaskCreatePinnedToCore(ym2151Task, "ym2151", 8192, NULL, 1, &ym2151taskHandle, 1);
    xTaskCreatePinnedToCore(loop, "vgmplay", 8192, NULL, 1, &vgmtaskHandle, 1);
    while (true) {
        vTaskDelay(1000);
    }
 }