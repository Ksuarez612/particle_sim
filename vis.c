#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <GLFW/glfw3.h>
#include <windows.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

typedef struct {
    float x, y, z;       // curr position
    float x_prev, y_prev, z_prev; // previous position
    float ax, ay, az;    // acceleration
    float radius; // radius of particle -> collision detection
    int collision_count;
} Particle;

void initialize_particles(Particle* particles, int num_particles);
void update_particles(Particle* particles, int num_particles, float dt);
void draw_particles(Particle* particles, int num_particles);
void draw_boundary();
void initOpenGL();
void draw();
// void render_text(const char* text, float x, float y, float scale, float r, float g, float b);

const char* border = "square";


int num_particles = 300;
Particle* particles;
const float GRAVITY = -1.98f;
const float sphere_radius = 1.0f;
const float time_step = 0.01f;


void interpolate_color(float t, float* r, float* g, float* b) {
    float r_low = 0.678f, g_low = 0.847f, b_low = 1.0f;
    float r_high = 1.0f, g_high = 0.0f, b_high = 0.0f;

    *r = (1 - t) * r_low + t * r_high;
    *g = (1 - t) * g_low + t * g_high;
    *b = (1 - t) * b_low + t * b_high;
}

// void render_text(float x, float y, const char* text) {
//     glRasterPos2f(x, y);
//     while (*text) {
//         glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *text);
//         text++;
//     }
// }



int main() {
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 600, "Particle Simulation", NULL, NULL);
    if (!window) {
        fprintf(stderr, "Failed to open GLFW window\n");
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    initOpenGL();

    particles = (Particle*)malloc(num_particles * sizeof(Particle)); //malloc for particles
    initialize_particles(particles, num_particles);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        update_particles(particles, num_particles, time_step); // assuming fixed time, move particles randomly
        draw();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    free(particles); // remove particles from mem 
    glfwTerminate();
    return 0;
}

// void draw_leaderboard(Particle* particles, int num_particles) {
//     // Sort particles based on collision count
//     Particle top3[3] = {{0}};
//     for (int i = 0; i < num_particles; i++) {
//         for (int j = 0; j < 3; j++) {
//             if (particles[i].collision_count > top3[j].collision_count) {
//                 for (int k = 2; k > j; k--) {
//                     top3[k] = top3[k - 1];
//                 }
//                 top3[j] = particles[i];
//                 break;
//             }
//         }
//     }
//      // Render the leaderboard
//     char buffer[256];
//     for (int i = 0; i < 3; i++) {
//         snprintf(buffer, sizeof(buffer), "Rank %d: Particle %d - Collisions: %d", i + 1, (int)(top3[i].x * 100), top3[i].collision_count);
//         render_text(-0.95f, -0.9f - 0.1f * i, buffer); // Adjust the position as needed
    
//     }
// }

void initialize_particles(Particle* particles, int num_particles) {
    if(border == "sphere") {
        int i;
        for (i = 0; i < num_particles; i++) {
            float theta = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
            float phi = ((float)rand() / RAND_MAX) * M_PI;

            particles[i].x = sphere_radius * sin(phi) * cos(theta);
            particles[i].y = sphere_radius * sin(phi) * sin(theta);
            particles[i].z = sphere_radius * cos(phi);
            particles[i].x_prev = particles[i].x;
            particles[i].y_prev = particles[i].y;
            particles[i].z_prev = particles[i].z;
            particles[i].ax = 0.0f;
            particles[i].ay = 0.0f;
            particles[i].az = 0.0f;
            particles[i].radius = 0.03f;
        }
        return;
    }

    int i;
    for (i = 0; i < num_particles; i++) {
        particles[i].x = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        particles[i].y = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        particles[i].z = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        particles[i].x_prev = particles[i].x;
        particles[i].y_prev = particles[i].y;
        particles[i].z_prev = particles[i].z;
        // particles[i].ax = ((float)rand() / RAND_MAX) * 0.05f - 0.01f;
        // particles[i].ay = ((float)rand() / RAND_MAX) * 0.05f - 0.01f;
        // particles[i].az = ((float)rand() / RAND_MAX) * 0.05f - 0.01f;
        particles[i].ax = 0.0f;
        particles[i].ay = 0.0f;
        particles[i].az = 0.0f;
        particles[i].radius = 0.05f;
    }
}

float random_perturbation() {
    return ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
}

bool detect_collision(Particle *p1, Particle *p2) {
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    float dz = p1->z - p2->z;

    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    float min_distance = p1->radius + p2->radius;

    return distance < min_distance;
}

void fix_position(Particle *p1, Particle *p2) {
    // calculate the delta vector based on the difference of particles
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    float dz = p1->z - p2->z;

    float distance = sqrt(dx * dx + dy * dy + dz * dz);

    // normalize the vectors
    float nx = dx / distance;
    float ny = dy / distance;
    float nz = dz / distance;

    // calculate delta 

    float delta = p1->radius + p2->radius - distance;

    // fix the norm
    nx *= (delta * 0.5f);
    ny *= (delta * 0.5f);
    nz *= (delta * 0.5f);

    // update the values
    p1->x += nx;
    p1->y += ny;
    p1->z += nz;

    p2->x -= nx;
    p2->y -= ny;
    p2->z -= nz;

    p1->collision_count++;
    p2->collision_count++;
}

void update_particles(Particle* particles, int num_particles, float dt) {
    // apply forces to particles
    int a;
    for (a = 0; a < num_particles; a++) {
        particles[a].ay += GRAVITY;
    }
    
    // handle collisions
    int j, k;
    for (j = 0; j < num_particles; j++) {
        for (k = 0; k < num_particles; k++) {
            if (j != k) {
                if (detect_collision(&particles[j], &particles[k])) {
                    fix_position(&particles[j], &particles[k]);
                }
            }
        }
    }

    // apply the constraints of the boundary
    int i;
    for (i = 0; i < num_particles; i++) {
        if (border == "sphere") {
            float distance = sqrt(particles[i].x * particles[i].x +
                                  particles[i].y * particles[i].y +
                                  particles[i].z * particles[i].z);

            if (distance + particles[i].radius > sphere_radius) {
                // Normalize the position vector and move the particle back to the sphere surface
                float norm_x = particles[i].x / distance;
                float norm_y = particles[i].y / distance;
                float norm_z = particles[i].z / distance;

                particles[i].x = norm_x * (sphere_radius - particles[i].radius);
                particles[i].y = norm_y * (sphere_radius - particles[i].radius);
                particles[i].z = norm_z * (sphere_radius - particles[i].radius);

                // Reflect the velocity
                float vx = (particles[i].x - particles[i].x_prev) / dt;
                float vy = (particles[i].y - particles[i].y_prev) / dt;
                float vz = (particles[i].z - particles[i].z_prev) / dt;

                float dot_product = vx * norm_x + vy * norm_y + vz * norm_z;

                vx -= 2 * dot_product * norm_x;
                vy -= 2 * dot_product * norm_y;
                vz -= 2 * dot_product * norm_z;

                particles[i].x_prev = particles[i].x - vx * dt;
                particles[i].y_prev = particles[i].y - vy * dt;
                particles[i].z_prev = particles[i].z - vz * dt;
            }
            // continue; // fix this later on to handle sphere
        } else {
            if (particles[i].x < -1.0f + particles[i].radius) {
                particles[i].x = -1.0f + particles[i].radius;
                particles[i].x_prev = particles[i].x + (particles[i].x - particles[i].x_prev);
            } else if (particles[i].x > 1.0f - particles[i].radius) {
                particles[i].x = 1.0f - particles[i].radius;
                particles[i].x_prev = particles[i].x + (particles[i].x - particles[i].x_prev);
            }

            if (particles[i].y < -1.0f + particles[i].radius) {
                particles[i].y = -1.0f + particles[i].radius;
                particles[i].y_prev = particles[i].y + (particles[i].y - particles[i].y_prev);
            } else if (particles[i].y > 1.0f - particles[i].radius) {
                particles[i].y = 1.0f - particles[i].radius;
                particles[i].y_prev = particles[i].y + (particles[i].y - particles[i].y_prev);
            }

            if (particles[i].z < -1.0f + particles[i].radius) {
                particles[i].z = -1.0f + particles[i].radius;
                particles[i].z_prev = particles[i].z + (particles[i].z - particles[i].z_prev);
            } else if (particles[i].z > 1.0f - particles[i].radius) {
                particles[i].z = 1.0f - particles[i].radius;
                particles[i].z_prev = particles[i].z + (particles[i].z - particles[i].z_prev);
            }
        }
    }

    // now we update our positions and acceleration
    int ii;
    for (ii = 0; ii < num_particles; ii++) {
        float disp_x = particles[ii].x - particles[ii].x_prev;
        float disp_y = particles[ii].y - particles[ii].y_prev;
        float disp_z = particles[ii].z - particles[ii].z_prev;

        particles[ii].x_prev = particles[ii].x;
        particles[ii].y_prev = particles[ii].y;
        particles[ii].z_prev = particles[ii].z;

        particles[ii].ax *= dt * dt;
        particles[ii].ay *= dt * dt;
        particles[ii].az *= dt * dt;

        particles[ii].x += disp_x + particles[ii].ax;
        particles[ii].y += disp_y + particles[ii].ay;
        particles[ii].z += disp_z + particles[ii].az;

        // reset acceleration
        particles[ii].ax = 0.0f;
        particles[ii].ay = 0.0f;
        particles[ii].az = 0.0f;
    }


}

void draw_particles(Particle* particles, int num_particles) {
    GLUquadricObj* quadric = gluNewQuadric();
    int i;
    for (i = 0; i < num_particles; i++) {
        glPushMatrix();
        glTranslatef(particles[i].x, particles[i].y, particles[i].z);
        
        // interpolate color

        float vel_x = (particles[i].x - particles[i].x_prev) / time_step;
        float vel_y = (particles[i].y - particles[i].y_prev) / time_step;
        float vel_z = (particles[i].z - particles[i].z_prev) / time_step;

        // Calculate the magnitude of the velocity
        float vel_magnitude = sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);

        float t = fabs(vel_magnitude) / (3.0f); 
        if (t > 1.0f) {
            t = 1.0f;
        }

        float r, g, b;
        interpolate_color(t, &r, &g, &b);

        glColor4f(r, g, b, 0.5f);

        gluSphere(quadric, particles[i].radius, 16, 16);
        glPopMatrix();
    }
    gluDeleteQuadric(quadric);
}

void draw_boundary(){
    if (border == "sphere") {
        GLUquadric* quad = gluNewQuadric();
        glColor4f(1.0f, 1.0f, 1.0f, 0.1f); 
        gluQuadricDrawStyle(quad, GLU_LINE); 
        gluSphere(quad, 1.0f, 32, 32); 
        gluDeleteQuadric(quad);
    } else {
        glColor3f(1.0f, 1.0f, 1.0f); 
        glBegin(GL_LINES);

        // bottom
        glVertex3f(-1.0f, -1.0f, -1.0f);
        glVertex3f(1.0f, -1.0f, -1.0f);

        glVertex3f(1.0f, -1.0f, -1.0f);
        glVertex3f(1.0f, -1.0f, 1.0f);

        glVertex3f(1.0f, -1.0f, 1.0f);
        glVertex3f(-1.0f, -1.0f, 1.0f);

        glVertex3f(-1.0f, -1.0f, 1.0f);
        glVertex3f(-1.0f, -1.0f, -1.0f);

        // top
        glVertex3f(-1.0f, 1.0f, -1.0f);
        glVertex3f(1.0f, 1.0f, -1.0f);

        glVertex3f(1.0f, 1.0f, -1.0f);
        glVertex3f(1.0f, 1.0f, 1.0f);

        glVertex3f(1.0f, 1.0f, 1.0f);
        glVertex3f(-1.0f, 1.0f, 1.0f);

        glVertex3f(-1.0f, 1.0f, 1.0f);
        glVertex3f(-1.0f, 1.0f, -1.0f);

        // vertical
        glVertex3f(-1.0f, -1.0f, -1.0f);
        glVertex3f(-1.0f, 1.0f, -1.0f);

        glVertex3f(1.0f, -1.0f, -1.0f);
        glVertex3f(1.0f, 1.0f, -1.0f);

        glVertex3f(1.0f, -1.0f, 1.0f);
        glVertex3f(1.0f, 1.0f, 1.0f);

        glVertex3f(-1.0f, -1.0f, 1.0f);
        glVertex3f(-1.0f, 1.0f, 1.0f);

        glEnd();
    }
}

void initOpenGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.0, 0.0, 0.0, 1.0);
}

void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(border == "sphere") {
        gluPerspective(0.0, 800.0 / 600.0, 0.1, 100.0);
    }
    gluPerspective(45.0, 800.0 / 600.0, 0.1, 100.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if (border == "sphere") {
        gluLookAt(0.0, 0.0, 3.0,  // Camera position
          0.0, 0.0, 0.0,  // Look-at point
          0.0, 1.0, 0.0); // Up direction
    } else {
        gluLookAt(3.0, 3.0, 3.0,  // Camera position
            0.0, 0.0, 0.0,  // Look-at point
            0.0, 1.0, 0.0); // Up direction
    }
    draw_boundary();

    draw_particles(particles, num_particles);

    //draw_leaderboard(particles, num_particles);

    glFlush();
}
