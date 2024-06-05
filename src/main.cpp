#include <stardust.h>

i32 main() {
    // TODO: pass the memory size here
    sd_init("Stardust Demo", 800, 600);

    sd_init_scratch_arenas(sd_memory(), 3, SD_MB(10));

    SDArena arena = sd_arena_create(sd_memory(), SD_MB(10));

    SDTexture *warrior_tex = sd_create_texture(&arena, "../data/warrior.png");

    SDMesh *warrior = sd_mesh_create(&arena, "../data/warrior.dae");
    SDSkeleton  *warior_skeleton  = sd_skeleton_create(&arena, "../data/warrior.dae");

    SDAnimation *idle = sd_animation_create(&arena, "../data/idle.dae");
    SDAnimation *walk_front = sd_animation_create(&arena, "../data/walk_front.dae");
    SDAnimation *walk_back = sd_animation_create(&arena, "../data/walk_back.dae");
    SDAnimation *walk_left = sd_animation_create(&arena, "../data/walk_left.dae");
    SDAnimation *walk_right = sd_animation_create(&arena, "../data/walk_right.dae");

    sd_set_view_mat(&sd_mat4_lookat(SDVec3(0, 5, -7), SDVec3(0, 3, 0), SDVec3(0, 1, 0)));
    sd_set_proj_mat(&sd_mat4_perspective(60, (float)sd_window_width()/sd_window_height(), 0.1f, 100.0f));
    
    while(sd_should_close() == false) {
        sd_process_events();

        static f32 anim_speed = 0.033f;
        
        SDVec3 front = SDVec3(0, 0, 1);
        SDVec3 right = SDVec3(1, 0, 0);

        SDVec3 velocity = SDVec3(0, 0, 0);
        velocity.x = sd_get_left_stick_x();
        velocity.z = sd_get_left_stick_y();

        f32 speed = sd_vec3_len(velocity);

        f32 angle = atan2(velocity.x, velocity.z) - atan2(front.x, front.z);
        angle = (angle/SD_PI) *180.0f;


        if(sd_key_down(SD_KEY_UP)) {
            anim_speed += 0.001f;
        }
        if(sd_key_down(SD_KEY_DOWN)) {
            anim_speed -= 0.001f;
        }

        sd_skeleton_interpolate_4_animations(warior_skeleton, walk_left, walk_front, walk_right, walk_back, idle, angle, speed, anim_speed);

        sd_clear_back_buffer(0.2f, 0.3f, 0.4f);

        sd_set_texture(warrior_tex);
        sd_set_world_mat(&(sd_mat4_scale(3, 3, 3)));
        sd_draw_anim_vertex_buffer(warior_skeleton->final_bone_matrices, warrior->vbuffer);

        sd_present();
        sd_store_input_for_next_frame();
    }

    sd_shutdown();

    return 0;
}