#include <stardust.h>


struct Camera {
    SDVec3 pos;
    SDVec2 rot;
    SDVec3 front;
    SDVec3 right;
    SDVec3 up;
    f32 dist;
};

Camera camera_create(SDVec3 target) {
    Camera camera;
    camera.front = SDVec3(0, 0, -1);
    camera.right = SDVec3(1, 0, 0);
    camera.up = SDVec3(0, 1, 0);
    camera.dist = 10.0f;
    camera.pos = target - (camera.front * camera.dist);
    return camera;
}

void camera_update(Camera *camera, SDVec3 target, f32 dt) {

    camera->rot.y -= sd_get_right_stick_x() * 2.5f * dt;
    camera->rot.x += sd_get_right_stick_y() * 2.5f * dt;

    if (camera->rot.x >  (89.0f/180.0f) * (f32)SD_PI)
        camera->rot.x =  (89.0f/180.0f) * (f32)SD_PI;
    if (camera->rot.x < -(89.0f/180.0f) * (f32)SD_PI)
        camera->rot.x = -(89.0f/180.0f) * (f32)SD_PI;
    
    camera->front = SDVec3(0, 0, -1);
    camera->front = sd_mat4_transform_vector(sd_mat4_rotation_x(camera->rot.x), camera->front);
    camera->front = sd_mat4_transform_vector(sd_mat4_rotation_y(camera->rot.y), camera->front);
    sd_vec3_normalize(camera->front);
    camera->right = sd_vec3_normalized(sd_vec3_cross(camera->front, SDVec3(0, 1, 0)));
    camera->up = sd_vec3_normalized(sd_vec3_cross(camera->right, camera->front));

    camera->pos = target - (camera->front * camera->dist);
}


i32 main() {
    // TODO: pass the memory size here
    sd_init("Stardust Demo", 960, 540);

    sd_init_scratch_arenas(sd_memory(), 3, SD_MB(10));

    SDArena arena = sd_arena_create(sd_memory(), SD_MB(10));

    SDTexture *sphere_tex = sd_create_texture(&arena, "../data/sphere.png");
    SDMesh *sphere = sd_mesh_create(&arena, "../data/sphere.dae");

    SDTexture *floor_tex = sd_create_texture(&arena, "../data/stars.png");
    SDMesh *floor = sd_mesh_create(&arena, "../data/floor.dae");

    SDTexture *warrior_tex = sd_create_texture(&arena, "../data/warrior.png");
    SDMesh *warrior = sd_mesh_create(&arena, "../data/warrior.dae");
    SDSkeleton  *warior_skeleton  = sd_skeleton_create(&arena, "../data/warrior.dae");

    SDAnimation *idle = sd_animation_create(&arena, "../data/idle.dae");
    SDAnimation *walk_front = sd_animation_create(&arena, "../data/walk_front.dae");
    SDAnimation *walk_back = sd_animation_create(&arena, "../data/walk_back.dae");
    SDAnimation *walk_left = sd_animation_create(&arena, "../data/walk_left.dae");
    SDAnimation *walk_right = sd_animation_create(&arena, "../data/walk_right.dae");

    sd_set_view_mat(sd_mat4_lookat(SDVec3(0, 5, 10), SDVec3(0, 3, 0), SDVec3(0, 1, 0)));
    sd_set_proj_mat(sd_mat4_perspective(60, (float)sd_window_width()/sd_window_height(), 0.1f, 100.0f));
    

    SDParticle hero;
    hero.position = SDVec3(0, 0, 0);
    hero.velocity = SDVec3();
    hero.acceleration = SDVec3();
    hero.force_accum = SDVec3();
    hero.damping = 0.01f;
    sd_particle_set_mass(&hero, 2);


    SDParticle ball;
    ball.position = SDVec3(0, 10, 0);
    ball.velocity = SDVec3();
    ball.acceleration = SDVec3();
    ball.force_accum = SDVec3();
    ball.damping = 0.5f;
    sd_particle_set_mass(&ball, 1);

    SDParticleForceGenerator fg_gravity{};
    fg_gravity.gravity.gravity = SDVec3(0, -9.8f, 0);

    SDParticleForceRegistry *fg_registry = sd_particle_force_registry_create(&arena, 100);
    sd_particle_force_registry_add(fg_registry, &ball, &fg_gravity);

    Camera camera = camera_create(hero.position + SDVec3(0, 3, 0));

    f32 fps_target = 1.0f / 60.0f;

    f64 last_time = sd_get_time();
    while(sd_should_close() == false) {

        f64 current_time = sd_get_time();
        f32 elapsed_time = current_time - last_time;
        while(elapsed_time < fps_target) {
            current_time = sd_get_time();
            elapsed_time = current_time - last_time;
        }
        

        f32 dt = (f32)(current_time - last_time);
        
        //SD_INFO("FPS: %lf", 1.0f/dt);

        last_time = current_time;

        sd_process_events();

       
        SDVec3 right = camera.right;
        SDVec3 front = sd_vec3_normalized(sd_vec3_cross(SDVec3(0, 1, 0),right));

        SDVec3 force = SDVec3(0, 0, 0);

        if(sd_key_down(SD_KEY_W)) {
            force += front;
        }
        if(sd_key_down(SD_KEY_S)) {
            force -= front;
        }
        if(sd_key_down(SD_KEY_D)) {
            force += right;
        }
        if(sd_key_down(SD_KEY_A)) {
            force -= right;
        }

        force += front * sd_get_left_stick_y();
        force += right * sd_get_left_stick_x();
        force = force * 50.0f;

        sd_particle_add_force(&hero, force);

        // update particles
        sd_particle_force_registry_update(fg_registry, dt);
        sd_particle_intergrate(&ball, dt);
        sd_particle_intergrate(&hero, dt);

        f32 speed = SD_MIN(sd_vec3_len(hero.velocity), 1.0f);

        SDMat4 hero_transform = sd_mat4_translation(hero.position) * sd_mat4_rotation_y(SD_PI + camera.rot.y);
        SDVec3 local_vel = sd_mat4_transform_vector(sd_mat4_inverse(hero_transform), hero.velocity);
        SDVec3 local_front = SDVec3(0, 0, -1);
        f32 angle = atan2(local_vel.x, local_vel.z);
        angle = (angle/SD_PI) *180.0f;

        SD_INFO("angle: %f", angle);

        sd_skeleton_interpolate_4_animations(warior_skeleton, walk_right, walk_front, walk_left, walk_back, idle, angle, speed, dt);

        camera_update(&camera, hero.position, dt);

        sd_set_view_mat(sd_mat4_lookat(camera.pos, hero.position + SDVec3(0, 3, 0), SDVec3(0, 1, 0)));
        sd_clear_back_buffer(0.2f, 0.3f, 0.4f);

        sd_set_texture(warrior_tex);
        sd_set_world_mat(hero_transform *  sd_mat4_scale(3, 3, 3));
        sd_draw_anim_vertex_buffer(warior_skeleton->final_bone_matrices, warrior->vbuffer);

        sd_set_texture(floor_tex);
        sd_set_world_mat(sd_mat4_translation(ball.position) * sd_mat4_scale(1, 1, 1));
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);

        sd_set_texture(sphere_tex);
        sd_set_world_mat(sd_mat4_translation(0, 0, 0) * sd_mat4_scale(10, 1, 10));
        sd_draw_vertex_buffer(floor->vbuffer, 0, 0, 0);

        //sd_draw_line(SDVec3(0, 2, 0), hero.position + SDVec3(0, 2, 0), 0, 1, 0);

        sd_present();
        sd_store_input_for_next_frame();
    }

    sd_shutdown();

    return 0;
}