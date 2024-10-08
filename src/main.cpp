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

    SDMesh *cube = sd_mesh_create(&arena, "../data/cube.dae");

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

//=============================================================================================
// Particle sample
//=============================================================================================
    SDParticle hero_anchor;
    hero_anchor.position = SDVec3(0, 0, 0);
    hero_anchor.velocity = SDVec3();
    hero_anchor.acceleration = SDVec3();
    hero_anchor.force_accum = SDVec3();
    hero_anchor.damping = 0;
    hero_anchor.inv_mass = 0;

    SDParticle hero;
    hero.position = SDVec3(0, 2, 0);
    hero.velocity = SDVec3();
    hero.acceleration = SDVec3(0, -9.8f*2.0f, 0);
    hero.force_accum = SDVec3();
    hero.damping = 0.01f;
    sd_particle_set_mass(&hero, 2);

    SDVec3 anchor = SDVec3(-10, 10, -10);
    SDVec3 anchor1 = SDVec3(10, 10, -10);

    SDParticle ball;
    ball.position = SDVec3(10, 10, -10);
    ball.velocity = SDVec3();
    ball.acceleration = SDVec3(0, -9.8f*2.0f, 0);
    ball.force_accum = SDVec3();
    ball.damping = 0.7f;
    sd_particle_set_mass(&ball, 0.5f);

    SDParticleForceGenerator fg_anchored_spring{};
    fg_anchored_spring.type = SD_PFG_ANCHORED_SPRING;
    fg_anchored_spring.anchored_spring.anchor = &anchor;
    fg_anchored_spring.anchored_spring.spring_constant = 1.0f;
    fg_anchored_spring.anchored_spring.rest_length = 2.0f;

    SDParticleForceGenerator fg_anchored_spring1{};
    fg_anchored_spring1.type = SD_PFG_ANCHORED_SPRING;
    fg_anchored_spring1.anchored_spring.anchor = &anchor1;
    fg_anchored_spring1.anchored_spring.spring_constant = 1.0f;
    fg_anchored_spring1.anchored_spring.rest_length = 2.0f;

    SDParticleForceRegistry fg_registry = sd_particle_force_registry_create(&arena, 100);
    sd_particle_force_registry_add(&fg_registry, &ball, &fg_anchored_spring);
    sd_particle_force_registry_add(&fg_registry, &ball, &fg_anchored_spring1);

    SDParticleContactGenerator cg_cable_hero{};
    cg_cable_hero.type = SD_PCG_CABLE;
    cg_cable_hero.cable.particle[0] = &hero_anchor;
    cg_cable_hero.cable.particle[1] = &hero;
    cg_cable_hero.cable.max_length = 50;
    cg_cable_hero.cable.restitution = 0;


    SDParticleContactGenerator cg_plane_y_zero_hero{};
    cg_plane_y_zero_hero.type = SD_PCG_PLANE_Y_ZERO;
    cg_plane_y_zero_hero.plane_y_zero.particle = &hero;


    SDParticleContactResolver pcr;
    pcr.iterations = 4;
    pcr.iterations_used = 0;
//=============================================================================================
//=============================================================================================


//=============================================================================================
// RigidBody Sample
//=============================================================================================
    f32 k = 1.0f/12.0f;
    f32 dx = 6;
    f32 dy = 6;
    f32 dz = 6;
    f32 hex = dx/2;
    f32 hey = dy/2;
    f32 hez = dz/2;
    f32 r = 1;
    f32 m = 4.0f*0.3333f*3.1415f * r*r*r;

    SDRigidBody platform;
    platform.position = SDVec3(-12, 10, -5);
    platform.orientation = SDQuat();
    platform.velocity = SDVec3();
    platform.rotation = SDVec3();
    platform.acceleration = SDVec3(0, -9.8f*2.0f, 0);
    platform.last_frame_acceleration = platform.acceleration;
    platform.force_accum = SDVec3();
    platform.torque_accum = SDVec3();
    platform.linear_damping = 0.8f;
    platform.angular_damping = 0.00001f;
    sd_body_set_mass(&platform, m);

    SDMat3 cube_inertia_tensor = SDMat3(
        k*m*((dy*dy) + (dz*dz)), 0, 0,
        0, k*m*((dx*dx) + (dz*dz)), 0,
        0, 0, k*m*((dx*dx) + (dy*dy))
    );
    platform.inertia_tensor = cube_inertia_tensor;
    platform.inverse_inertia_tensor = sd_mat3_inverse(cube_inertia_tensor);

    platform.is_awake = true;
    sd_body_calculate_derived_data(&platform);

    SDRigidBody anchor0_body = {};
    anchor0_body.position = SDVec3(-20, 10, -5);
    sd_body_calculate_derived_data(&anchor0_body);

    SDRigidBody anchor1_body = {};
    anchor1_body.position = SDVec3(-4, 15, -10);
    sd_body_calculate_derived_data(&anchor1_body);

    SDRigidBody anchor2_body = {};
    anchor2_body.position = SDVec3(-12, 15, 10);
    sd_body_calculate_derived_data(&anchor2_body);
    
    SDSpringForceGenerator spring0_fg{};
    spring0_fg.connection_point = SDVec3(-1, -1, -1);
    spring0_fg.other_conection_point = SDVec3(0, -1, 0);
    spring0_fg.other = &anchor0_body;
    spring0_fg.spring_constant = 10.0f;
    spring0_fg.rest_length = 2.0f;

    SDSpringForceGenerator spring1_fg{};
    spring1_fg.connection_point = SDVec3(1, 1, 1);
    spring1_fg.other_conection_point = SDVec3(0, -1, 0);
    spring1_fg.other = &anchor1_body;
    spring1_fg.spring_constant = 2.0f;
    spring1_fg.rest_length = 2.0f;

    SDSpringForceGenerator spring2_fg{};
    spring2_fg.connection_point = SDVec3(-1, 1, 1);
    spring2_fg.other_conection_point = SDVec3(0, -1, 0);
    spring2_fg.other = &anchor2_body;
    spring2_fg.spring_constant = 2.0f;
    spring2_fg.rest_length = 2.0f;

//=============================================================================================
//=============================================================================================

//=============================================================================================
// BVH Sample
//=============================================================================================
    SDRigidBody a = {};
    a.position = SDVec3(0, 2, 0);
    SDBoundingSphere a_bs = sd_bounding_sphere_create(a.position, 0.5f);

    SDRigidBody b = {};
    b.position = SDVec3(0.5, 2, 0);
    SDBoundingSphere b_bs = sd_bounding_sphere_create(b.position, 0.5f);

    SDRigidBody c = {};
    c.position = SDVec3(5, 2, 0);
    SDBoundingSphere c_bs = sd_bounding_sphere_create(c.position, 0.5f);

    SDBlockAllocator bvh_allocator = sd_block_allocator_create(sd_memory(), sizeof(SDBVHNode), 100);

    SDBVHNode *node = sd_bvh_node_create(&bvh_allocator, 0, a_bs, &a);
    sd_bvh_node_insert(&bvh_allocator, node, &b, &b_bs);
    sd_bvh_node_insert(&bvh_allocator, node, &c, &c_bs);

    SDPotentialContact contacts[10] = {};
    u32 count = sd_bvh_node_get_potetial_contacts(node, contacts, 10);

//=============================================================================================
//=============================================================================================

//=============================================================================================
// Collision Detection Test
//=============================================================================================
    SDMat3 sphere_inertia_tensor = SDMat3(
        (2.0f/5.0f)*m*(r*r), 0, 0,
        0, (2.0f/5.0f)*m*(r*r), 0,
        0, 0, (2.0f/5.0f)*m*(r*r)
    );

    SDRigidBody crate;
    crate.position = SDVec3(0, r, 0);
    crate.orientation = sd_quat_angle_axis(SD_PI, SDVec3(0, -0, 1));
    crate.velocity = SDVec3();
    crate.rotation = SDVec3();
    crate.acceleration = SDVec3(0, -9.8f*2, 0);
    crate.last_frame_acceleration = platform.acceleration;
    crate.force_accum = SDVec3();
    crate.torque_accum = SDVec3();
    crate.linear_damping = 0.9f;
    crate.angular_damping = 0.2f;
    sd_body_set_mass(&crate, m);
    crate.inertia_tensor = cube_inertia_tensor;
    crate.inverse_inertia_tensor = sd_mat3_inverse(sphere_inertia_tensor);

    crate.is_awake = true;
    sd_body_calculate_derived_data(&crate);

    //SDBox box;
    //box.body = &crate;
    //box.half_size = SDVec3(hex, hey, hez);
    //box.transform = crate.transform_matrix;

    SDSphere sp;
    sp.body = &crate;
    sp.radius = r;
    sp.transform = crate.transform_matrix;

    SDPlane plane;
    plane.normal = sd_mat3_rotation_x(0.0f) * SDVec3(0, 1, 0);
    plane.offset = 0.0f;

    SDPlane plane1;
    plane1.normal = sd_mat3_rotation_z(0.2f) * sd_mat3_rotation_x(0.0f) * SDVec3(0, 1, 0);
    plane1.offset = 0.0f;

    SDPlane plane2;
    plane2.normal = sd_mat3_rotation_z(0.0f) * sd_mat3_rotation_x(0.1f) * SDVec3(0, 1, 0);
    plane2.offset = 0.0f;

//=============================================================================================
//=============================================================================================

    Camera camera = camera_create(hero.position + SDVec3(0, 3, 0));
    camera.rot.x = -SD_PI/6;

    f32 fps_target = 1.0f / 60.0f;

    f64 last_time = sd_get_time();
    while(sd_should_close() == false) {

        f64 current_time = sd_get_time();
#if 1
        f32 elapsed_time = current_time - last_time;
        while(elapsed_time < fps_target) {
            current_time = sd_get_time();
            elapsed_time = current_time - last_time;
        }
#endif   

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


        // add forces
        sd_particle_add_force(&hero, force);
        sd_particle_force_registry_update(&fg_registry, dt);

        // update particles
        sd_particle_intergrate(&ball, dt);
        sd_particle_intergrate(&hero, dt);

        // Collision testing
        SDParticleContact contacts[20];
        u32 num_contact = 0;
        num_contact += sd_particle_contact_generator_add_contact(&cg_plane_y_zero_hero, contacts + num_contact, 0);
        num_contact += sd_particle_contact_generator_add_contact(&cg_cable_hero, contacts + num_contact, 0);
        if(num_contact > 0) {
            sd_particle_contact_resolver_resolve_contacts(&pcr, contacts, num_contact, dt);
        }

        sd_spring_force_generator_update(&spring0_fg, &platform, dt);
        sd_spring_force_generator_update(&spring1_fg, &platform, dt);
        sd_spring_force_generator_update(&spring2_fg, &platform, dt);
        sd_body_integrate(&platform, dt);

        if(sd_key_down(SD_KEY_LEFT)) {
            sd_body_add_force_at_point(&crate, SDVec3(-100, 0, 0), crate.position);
        }
        if(sd_key_down(SD_KEY_RIGHT)) {
            sd_body_add_force_at_point(&crate, SDVec3(100, 0, 0), crate.position);
        }

        if(sd_key_down(SD_KEY_UP)) {
            sd_body_add_force_at_point(&crate, SDVec3(0, 0, -100), crate.position);
        }
        if(sd_key_down(SD_KEY_DOWN)) {
            sd_body_add_force_at_point(&crate, SDVec3(0, 0, 100), crate.position);
        }

        sd_body_integrate(&crate, dt);
        //box.transform = crate.transform_matrix;
        sp.transform = crate.transform_matrix;

        // crate collision test
        SDCollisionResolver cr;



        SDContact body_contacts[100];
        memset(body_contacts, 0, sizeof(SDContact) * SD_ARRAY_LENGTH(body_contacts));
        SDCollisionData collision_data{};
        collision_data.contact_array = body_contacts;
        collision_data.contacts_left = 100;
        collision_data.contact_count = 0;
        sd_collision_detector_sphere_plane(&sp, &plane, &collision_data);
        //sd_collision_detector_sphere_plane(&sp, &plane1, &collision_data);
        //sd_collision_detector_sphere_plane(&sp, &plane2, &collision_data);
        //sd_collision_detector_box_plane(&box, &plane, &collision_data);
        if(collision_data.contact_count > 0) {
            sd_collision_resolver(&cr, collision_data.contact_array, collision_data.contact_count, dt);
        }


        f32 speed = SD_MIN(sd_vec3_len(hero.velocity), 1.0f);

        SDMat4 hero_transform = sd_mat4_translation(hero.position) * sd_mat4_rotation_y(SD_PI + camera.rot.y);
        SDVec3 local_vel = sd_mat4_transform_vector(sd_mat4_inverse(hero_transform), hero.velocity);
        f32 angle = atan2(local_vel.x, local_vel.z);
        angle = (angle/SD_PI) *180.0f;

        sd_skeleton_interpolate_4_animations(warior_skeleton, walk_right, walk_front, walk_left, walk_back, idle, angle, speed, dt);

        camera_update(&camera, hero.position, dt);

        sd_set_view_mat(sd_mat4_lookat(camera.pos, hero.position + SDVec3(0, 3, 0), SDVec3(0, 1, 0)));
        sd_clear_back_buffer(0.2f, 0.3f, 0.4f);

        sd_set_texture(warrior_tex);
        sd_set_world_mat(hero_transform *  sd_mat4_scale(3, 3, 3));
        //sd_set_world_mat(hero_transform *  sd_mat4_scale(0.025f, 0.025f, 0.025f));
        sd_draw_anim_vertex_buffer(warior_skeleton->final_bone_matrices, warrior->vbuffer);

        sd_set_texture(floor_tex);
        sd_set_world_mat(sd_mat4_translation(ball.position));
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);

        sd_set_world_mat(sd_mat4_translation(anchor));
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);

        sd_set_world_mat(sd_mat4_translation(anchor1));
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);

        sd_set_texture(sphere_tex);
        sd_set_world_mat(sd_mat4_translation(0, 0, 0) * sd_mat4_scale(10, 1, 10));
        sd_draw_vertex_buffer(floor->vbuffer, 0, 0, 0);

        //sd_set_texture(sphere_tex);
        //sd_set_world_mat(sd_mat4_rotation_z(0.5f) * sd_mat4_scale(10, 1, 10));
        //sd_draw_vertex_buffer(floor->vbuffer, 0, 0, 0);


        sd_draw_line(anchor, ball.position, 0, 1, 0);
        sd_draw_line(anchor1, ball.position, 0, 1, 0);

        sd_set_texture(floor_tex);
        sd_set_world_mat(platform.transform_matrix);
        sd_draw_vertex_buffer(cube->vbuffer, 0, 0, 0);
        sd_set_world_mat(anchor0_body.transform_matrix);
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);
        sd_set_world_mat(anchor1_body.transform_matrix);
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);
        sd_set_world_mat(anchor2_body.transform_matrix);
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);

        SDVec3 src0 = sd_body_get_point_in_world_space(&anchor0_body, spring0_fg.other_conection_point); 
        SDVec3 dst0 = sd_body_get_point_in_world_space(&platform, spring0_fg.connection_point);
        sd_draw_line(src0, dst0, 0, 1, 0);

        SDVec3 src1 = sd_body_get_point_in_world_space(&anchor1_body, spring1_fg.other_conection_point); 
        SDVec3 dst1 = sd_body_get_point_in_world_space(&platform, spring1_fg.connection_point);
        sd_draw_line(src1, dst1, 0, 1, 0);

        SDVec3 src2 = sd_body_get_point_in_world_space(&anchor2_body, spring2_fg.other_conection_point); 
        SDVec3 dst2 = sd_body_get_point_in_world_space(&platform, spring2_fg.connection_point);
        sd_draw_line(src2, dst2, 0, 1, 0);


        sd_set_texture(floor_tex);
        sd_set_world_mat(crate.transform_matrix * sd_mat4_scale(r, r, r));
        sd_draw_vertex_buffer(sphere->vbuffer, 0, 0, 0);
        SDVec3 sdr = crate.position;
        SDVec3 dst_x = crate.position + sd_mat4_to_mat3(sd_quat_to_mat4(crate.orientation)) * SDVec3(6, 0, 0);
        sd_draw_line(sdr, dst_x, 1, 0, 0);
        SDVec3 dst_y = crate.position + sd_mat4_to_mat3(sd_quat_to_mat4(crate.orientation)) * SDVec3(0, 6, 0);
        sd_draw_line(sdr, dst_y, 0, 1, 0);
        SDVec3 dst_z = crate.position + sd_mat4_to_mat3(sd_quat_to_mat4(crate.orientation)) * SDVec3(0, 0, 6);
        sd_draw_line(sdr, dst_z, 0, 0, 1);


        sd_present();
        sd_store_input_for_next_frame();
    }

    sd_shutdown();

    return 0;
}
