#include <sd_collision.h>

u32 collision_detector_box_plane(SDBox *box, SDPlane *plane, SDCollisionData *data) {

    // make sure we have contacts
    if(data->contacts_left <= 0) {
        return 0;
    }

    // TODO: check for intersection as a early out

    SDVec3 vertices[8] = {
        SDVec3(-box->half_size.x, -box->half_size.y, -box->half_size.z),
        SDVec3(-box->half_size.x, -box->half_size.y, +box->half_size.z),
        SDVec3(-box->half_size.x, +box->half_size.y, -box->half_size.z),
        SDVec3(-box->half_size.x, +box->half_size.y, +box->half_size.z),
        SDVec3(+box->half_size.x, -box->half_size.y, -box->half_size.z),
        SDVec3(+box->half_size.x, -box->half_size.y, +box->half_size.z),
        SDVec3(+box->half_size.x, +box->half_size.y, -box->half_size.z),
        SDVec3(+box->half_size.x, +box->half_size.y, +box->half_size.z)
    };

    SDContact *contact = data->contacts;
    u32 contacts_used = 0;
    for(i32 i = 0; i < 8; i++) {
        SDMat4 trans = sd_mat4_translation(box->body->position);
        SDMat4 rot   = sd_quat_to_mat4(box->body->orientation);
        SDMat4 model = trans * rot;
        SDVec3 vertex = sd_mat4_transform_point(model, vertices[i]);

        // calculate the distance from the plane
        f32 vertex_distance = sd_vec3_dot(vertex, plane->normal);
        if(vertex_distance <= plane->offset) {
            // create the contact data
            contact->contact_point = vertex;
            contact->contact_normal = plane->normal;
            contact->penetration = plane->offset - vertex_distance;

            contact++;
            contacts_used++;
            if(contacts_used == data->contacts_left) {
                return contacts_used;
            }
        }
    }
    return contacts_used;
}