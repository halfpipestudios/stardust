#include <stardust.h>

i32 main() {
    // TODO: pass the memory size here
    sd_init("Stardust Demo", 800, 600);

    sd_init_scratch_arenas(sd_memory(), 3, SD_MB(10));

    SDArena arena = sd_arena_create(sd_memory(), SD_MB(40));


    SDTexture *cool_tex = sd_create_texture(&arena, "../data/cool.png");
    SDTexture *warrior_tex = sd_create_texture(&arena, "../data/warrior.png");

    SDMesh *cube = sd_mesh_create(&arena, "../data/cube.dae");
    SDMesh *mona = sd_mesh_create(&arena, "../data/mona.dae");
    SDMesh *warrior = sd_mesh_create(&arena, "../data/warrior.dae");

    SDAnimation *warior_anim = sd_animation_create(&arena, "../data/warrior.dae");
    SDAnimator *animator = sd_animator_create(&arena, warrior, warior_anim);

    sd_animator_play(animator);

    sd_set_view_mat(&sd_mat4_lookat(SDVec3(0, 0, 5), SDVec3(), SDVec3(0, 1, 0)));
    sd_set_proj_mat(&sd_mat4_perspective(60, (float)sd_window_width()/sd_window_height(), 0.1f, 100.0f));
    
    while(sd_should_close() == false) {
        sd_process_events();

        static float angle = 0.0f;
        static float speed = 0.033f;
        static float x_pos = 0.0f;
        if(sd_key_down(SD_KEY_D)) {
            angle += 0.07f;
        }
        if(sd_key_down(SD_KEY_A)) {
            angle -= 0.07f;
        }
        if(sd_key_down(SD_KEY_W)) {
            speed += 0.001f;
        }
        if(sd_key_down(SD_KEY_S)) {
            speed -= 0.001f;
        }
        if(sd_key_down(SD_KEY_RIGHT)) {
            x_pos += 0.1f;
        }
        if(sd_key_down(SD_KEY_LEFT)) {
            x_pos -= 0.1f;
        }

        sd_animator_update(animator, speed);

        sd_clear_back_buffer(0.2f, 0.3f, 0.4f);

        sd_set_texture(warrior_tex);

        SDQuat quat = sd_quat_angle_axis(angle, SDVec3(0, 1, 0));
        sd_set_world_mat(&(sd_mat4_translation( x_pos, -2, -2) * sd_mat4_scale(2, 2, 2) * sd_quat_to_mat4(quat)));
        sd_draw_anim_vertex_buffer(animator, warrior->vbuffer);

        sd_set_texture(cool_tex);

        sd_set_world_mat(&(sd_quat_to_mat4(quat) * sd_mat4_rotation_x((-90.0f/180.0f) * SD_PI)));
        sd_draw_vertex_buffer(mona->vbuffer, 1, 1, 1);


        sd_present();
        sd_store_input_for_next_frame();
    }

    sd_shutdown();

    return 0;
}