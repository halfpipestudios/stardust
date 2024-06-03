#pragma once

#include <sd_common.h>

struct SDMemory {
    u64 size;
    u64 used;
    u8 *data;
};

struct SDArena {
    u64 size;
    u64 used;
    u8 *base;
};

struct SDTempArena {
    SDArena *arena;
    u64 pos;
};

#define SD_MAX_SCRATCH_ARENA_COUNT 16

void sd_init_scratch_arenas(SDMemory *memory, u32 count, u64 size);
SDArena *sd_get_scratch_arena(i32 index);

#define sd_arena_push_struct(arena, type) (type *)sd_arena_push_size(arena, sizeof(type))
#define sd_arena_push_array(arena, count, type) (type *)sd_arena_push_size(arena, count * sizeof(type))

SDArena sd_arena_create(SDMemory *memory, u64 size);
void *sd_arena_push_size(SDArena *arena, u64 size);
void sd_arena_clear(SDArena *arena);
SDTempArena sd_temp_arena_begin(SDArena *arena);
void sd_temp_arena_end(SDTempArena tmp);