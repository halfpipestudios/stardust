#pragma once

#include <sd_platform.h>

#define SD_SD_KEY_BACKESPACE	0x08
#define SD_SD_KEY_ESCAPE      0x1B
#define SD_SD_KEY_0           0x30 
#define SD_KEY_1           0x31 
#define SD_KEY_2           0x32 
#define SD_KEY_3           0x33 
#define SD_KEY_4           0x34 
#define SD_KEY_5           0x35 
#define SD_KEY_6           0x36 
#define SD_KEY_7           0x37 
#define SD_KEY_8           0x38 
#define SD_KEY_9           0x39 
#define SD_KEY_A           0x41 
#define SD_KEY_B           0x42 
#define SD_KEY_C           0x43 
#define SD_KEY_D           0x44 
#define SD_KEY_E           0x45 
#define SD_KEY_F           0x46 
#define SD_KEY_G           0x47 
#define SD_KEY_H           0x48 
#define SD_KEY_I           0x49 
#define SD_KEY_J           0x4A 
#define SD_KEY_K           0x4B 
#define SD_KEY_L           0x4C 
#define SD_KEY_M           0x4D 
#define SD_KEY_N           0x4E 
#define SD_KEY_O           0x4F 
#define SD_KEY_P           0x50 
#define SD_KEY_Q           0x51 
#define SD_KEY_R           0x52 
#define SD_KEY_S           0x53 
#define SD_KEY_T           0x54 
#define SD_KEY_U           0x55 
#define SD_KEY_V           0x56 
#define SD_KEY_W           0x57 
#define SD_KEY_X           0x58 
#define SD_KEY_Y           0x59 
#define SD_KEY_Z           0x5A 
#define SD_KEY_NUMPAD0     0x60	    
#define SD_KEY_NUMPAD1     0x61	    
#define SD_KEY_NUMPAD2     0x62	    
#define SD_KEY_NUMPAD3     0x63	    
#define SD_KEY_NUMPAD4     0x64	    
#define SD_KEY_NUMPAD5     0x65	    
#define SD_KEY_NUMPAD6     0x66	    
#define SD_KEY_NUMPAD7     0x67	    
#define SD_KEY_NUMPAD8     0x68	    
#define SD_KEY_NUMPAD9     0x69	    
#define SD_KEY_RETURN      0x0D
#define SD_KEY_SPACE       0x20
#define SD_KEY_TAB         0x09
#define SD_KEY_CONTROL     0x11
#define SD_KEY_SHIFT       0x10
#define SD_KEY_ALT         0x12
#define SD_KEY_CAPS        0x14
#define SD_KEY_LEFT        0x25
#define SD_KEY_UP          0x26
#define SD_KEY_RIGHT       0x27
#define SD_KEY_DOWN        0x28


struct SDInput {
    bool keys[256];
    bool mouse_buttons[3];

    f32 left_stick_x;
    f32 left_stick_y;
    f32 right_stick_x;
    f32 right_stick_y;
};

void sd_set_key(u32 key, bool value);
void sd_set_mouse_button(u32 button, bool value);

bool sd_key_down(u32 keycode);
bool sd_key_just_down(u32 keycode);
bool sd_key_up(u32 keycode);
bool sd_key_just_up(u32 keycode);
void sd_store_input_for_next_frame();

void sd_set_left_stick(f32 x, f32 y);
void sd_set_right_stick(f32 x, f32 y);

f32 sd_get_left_stick_x();
f32 sd_get_left_stick_y();
f32 sd_get_right_stick_x();
f32 sd_get_right_stick_y();