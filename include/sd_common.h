#pragma once

#include <stdint.h>

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int64_t i64;
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t  i8;
typedef double f64;
typedef float  f32;

#define SD_PI 3.14159265359f

#define SD_ASSERT(condition) if(!(condition)) { *(uint32_t *)0 = 0; }
#define SD_ARRAY_LENGTH(array) ( sizeof(array)/sizeof((array)[0]) )

#define SD_FATAL(message, ...) sd_log_message(SD_LOG_TYPE_FATAL, message, ##__VA_ARGS__); SD_ASSERT(!"Fatal error")
#define SD_ERROR(message, ...) sd_log_message(SD_LOG_TYPE_ERROR, message, ##__VA_ARGS__)
#define SD_WARNING(message, ...) sd_log_message(SD_LOG_TYPE_WARNING, message, ##__VA_ARGS__)
#define SD_INFO(message, ...) sd_log_message(SD_LOG_TYPE_INFO, message, ##__VA_ARGS__)

#define SD_MAX(a, b) ((a) >= (b) ? (a) : (b))
#define SD_MIN(a, b) ((a) <= (b) ? (a) : (b))

#define SD_KB(value) ((value)*1024LL)
#define SD_MB(value) (SD_KB(value)*1024LL)
#define SD_GB(value) (SD_MB(value)*1024LL)
#define SD_TB(value) (SD_GB(value)*1024LL)