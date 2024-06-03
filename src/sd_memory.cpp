#include <sd_memory.h>

static SDArena scratch_arenas[SD_MAX_SCRATCH_ARENA_COUNT];

static u32 scratch_arenas_count = 0;

void sd_init_scratch_arenas(SDMemory *memory, u32 count, u64 size) {
    SD_ASSERT(count <= SD_MAX_SCRATCH_ARENA_COUNT);
    scratch_arenas_count = count;
    for(i32 i = 0; i < count; i++) {
        scratch_arenas[i] = sd_arena_create(memory, size);
    }
}

SDArena *sd_get_scratch_arena(i32 index) {
    SD_ASSERT(index < scratch_arenas_count);
    return scratch_arenas + index;
}



SDArena sd_arena_create(SDMemory *memory, u64 size) {
    SD_ASSERT((memory->used + size) <= memory->size);
    SDArena arena;
    arena.used = 0;
    arena.size = size;
    arena.base = memory->data + memory->used;
    memory->used += size;
    return arena;
}

void *sd_arena_push_size(SDArena *arena, u64 size) {
    SD_ASSERT((arena->used + size) <= arena->size);
    void *data = arena->base + arena->used;
    arena->used += size;
    return data;
}

void sd_arena_clear(SDArena *arena) {
    arena->used = 0;
}

SDTempArena sd_temp_arena_begin(SDArena *arena) {
    SDTempArena tmp{};
    tmp.arena = arena;
    tmp.pos = arena->used;
    return tmp;
}

void sd_temp_arena_end(SDTempArena tmp) {
    tmp.arena->used = tmp.pos;
}