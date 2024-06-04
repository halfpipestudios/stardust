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

    sd_set_view_mat(&sd_mat4_lookat(SDVec3(0, 0, 5), SDVec3(), SDVec3(0, 1, 0)));
    sd_set_proj_mat(&sd_mat4_perspective(60, (float)sd_window_width()/sd_window_height(), 0.1f, 100.0f));
    
    while(sd_should_close() == false) {
        sd_process_events();

        static float angle = 0.0f;
        static float speed = 0.033f;
        static float x_pos = 0.0f;

        if(sd_key_down(SD_KEY_D)) {
            sd_skeleton_animate(warior_skeleton, walk_right, speed);
        }
        else if(sd_key_down(SD_KEY_A)) {
            sd_skeleton_animate(warior_skeleton, walk_left, speed);
        }
        else if(sd_key_down(SD_KEY_W)) {
            sd_skeleton_animate(warior_skeleton, walk_front, speed);
        }
        else if(sd_key_down(SD_KEY_S)) {
            sd_skeleton_animate(warior_skeleton, walk_back, speed);
        }
        else {
            sd_skeleton_animate(warior_skeleton, idle, speed);
        }



        if(sd_key_down(SD_KEY_UP)) {
            speed += 0.001f;
        }
        if(sd_key_down(SD_KEY_DOWN)) {
            speed -= 0.001f;
        }
        if(sd_key_down(SD_KEY_RIGHT)) {
            angle += 0.1f;
        }
        if(sd_key_down(SD_KEY_LEFT)) {
            angle -= 0.1f;
        }

        sd_clear_back_buffer(0.2f, 0.3f, 0.4f);

        sd_set_texture(warrior_tex);
        SDQuat quat = sd_quat_angle_axis(angle, SDVec3(0, 1, 0));
        sd_set_world_mat(&(sd_mat4_translation( x_pos, -2, -2) * sd_mat4_scale(3, 3, 3) * sd_quat_to_mat4(quat)));
        sd_draw_anim_vertex_buffer(warior_skeleton, warrior->vbuffer);

        sd_present();
        sd_store_input_for_next_frame();
    }

    sd_shutdown();

    return 0;
}