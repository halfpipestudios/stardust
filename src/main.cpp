#include <stardust.h>

i32 main() {
    // TODO: pass the memory size here
    sd_init("Stardust Demo", 800, 600);

    sd_init_scratch_arenas(sd_memory(), 3, SD_MB(10));

    SDArena arena = sd_arena_create(sd_memory(), SD_MB(40));


    SDTexture *cool_tex = sd_create_texture(&arena, "../data/cool.png");
    SDTexture *warrior_tex = sd_create_texture(&arena, "../data/warrior.png");

    SDStaticMesh *cube = sd_mesh_create(&arena, "../data/cube.dae");
    SDStaticMesh *mona = sd_mesh_create(&arena, "../data/mona.dae");


    SDAnimMesh *warrior = sd_anim_mesh_create(&arena, "../data/warrior.dae");
    SDAnimation *warior_anim = sd_animation_create(&arena, "../data/warrior.dae", warrior);
    SDAnimator *animator = sd_animator_create(&arena, warrior, warior_anim);

    sd_animator_play(animator);

    sd_set_view_mat(&sd_mat4_lookat(SDVec3(0, 0, 3), SDVec3(), SDVec3(0, 1, 0)));
    sd_set_proj_mat(&sd_mat4_perspective(60, (float)sd_window_width()/sd_window_height(), 0.1f, 100.0f));
    
    while(sd_should_close() == false) {
        sd_process_events();

        static float angle = 0.0f;
        static float speed = 0.016f;
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

        sd_animator_update(animator, speed);

        sd_clear_back_buffer(0.2f, 0.3f, 0.4f);

        sd_set_texture(warrior_tex);

        SDQuat quat = sd_quat_angle_axis(angle, SDVec3(0, 1, 0));
        sd_set_world_mat(&(sd_mat4_translation( 0, -2, -2) * sd_mat4_scale(2, 2, 2) * sd_quat_to_mat4(quat)));
        
        sd_draw_anim_vertex_buffer(animator, warrior->vbuffer);


        sd_present();
        sd_store_input_for_next_frame();
    }

    sd_shutdown();

    return 0;
}