#include <sd_input.h>

static SDInput input[2];

void sd_set_key(uint32_t key, bool value)
{
    input[0].keys[key] = value; 
}

void sd_set_mouse_button(uint32_t button, bool value)
{
    input[0].mouse_buttons[button] = value;
}

void sd_store_input_for_next_frame()
{
    input[1] = input[0];
}

bool sd_key_down(uint32_t keycode)
{
    return input[0].keys[keycode];
}

bool sd_key_just_down(uint32_t keycode)
{
    return input[0].keys[keycode] && !input[1].keys[keycode];
}

bool sd_key_up(uint32_t keycode)
{  
    return !input[0].keys[keycode];
}

bool sd_key_just_up(uint32_t keycode)
{
    return !input[0].keys[keycode] && input[1].keys[keycode];
}
