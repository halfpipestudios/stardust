#include <sd_memory.h>
#include <memory.h>
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

SDBlockAllocator sd_block_allocator_create(SDMemory *memory, u64 block_size, u32 block_count) {
    SDBlockAllocator allocator;
    allocator.arena = sd_arena_create(memory, block_size * block_count);
    allocator.block_size = block_size;
    allocator.block_count = block_count;
    allocator.block_used = 0;
    allocator.free_list = 0;
    // init the block allocator memory
    u8 *data = (u8 *)sd_arena_push_size(&allocator.arena, block_size * block_count);
    for(u32 i = 0; i < block_count; i++) {
        u8 *block = data + (i * block_size);
        u32 *header = (u32 *)block;
        if(i == block_count - 1) {
            *header = 0xFFFFFFFF;
        }
        else {
            *header = i + 1;
        }
    }
    return allocator;
}

void *sd_block_allocator_alloc(SDBlockAllocator *allocator) {
    if (allocator->block_used >= allocator->block_count) {
        SD_ASSERT(!"ERROR: the block allocator is empty");
    }
    SD_ASSERT(allocator->free_list < 0xFFFFFFFF);

    // get the first free
    u32 free_index = allocator->free_list;

    // get a pinter to the free block
    u8 *free_block = allocator->arena.base + (free_index * allocator->block_size);

    // update the free list to point to the next free
    u32 *header = (u32 *)free_block;
    allocator->free_list = *header;

    allocator->block_used++;

    memset(free_block, 0, allocator->block_size);
    return free_block;
}

void sd_block_allocator_free(SDBlockAllocator *allocator, void *block) {
    if (allocator->block_used <= 0) {
        SD_ASSERT(!"ERROR: the block allocator is empty");
    }
    u32 *header = (u32 *)block;
    u32 index_to_free = (u32)(((u64)block - (u64)allocator->arena.base) / allocator->block_size);

    *header = allocator->free_list;
    allocator->free_list = index_to_free;

    allocator->block_used--;
}