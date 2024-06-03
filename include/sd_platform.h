#pragma once

#include <sd_common.h>
#include <sd_memory.h>

enum SDLogType {
    SD_LOG_TYPE_FATAL,
    SD_LOG_TYPE_ERROR,
    SD_LOG_TYPE_WARNING,
    SD_LOG_TYPE_INFO
};

void sd_init(char *title, u32 width, u32 height);
void sd_shutdown();
void sd_present();

void sd_process_events();
bool sd_should_close();
u32 sd_window_width();
u32 sd_window_height();
u32 *sd_back_buffer();
f32 *sd_depth_buffer();
SDMemory *sd_memory();

void sd_log_message(SDLogType type, const char *message, ...);