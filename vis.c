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


int num_particles = 1000;
Particle* particles;
const float GRAVITY = -0.098f;
const float sphere_radius = 1.0f;
const bool boolsphere = false; 
const float perturbation_factor = 0.01f; 
const float slowdown_factor = 0.5f;
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
    for (int i = 0; i < num_particles; i++) {
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
        particles[i].ay = GRAVITY;
        particles[i].az = 0.0f;
        particles[i].radius = 0.03f;
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
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    float dz = p1->z - p2->z;

    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    float overlap = 0.5f * (distance - p1->radius - p2->radius);

    p1->x -= overlap * (dx / distance);
    p1->y -= overlap * (dy / distance);
    p1->z -= overlap * (dz / distance);

    p2->x += overlap * (dx / distance);
    p2->y += overlap * (dy / distance);
    p2->z += overlap * (dz / distance);

    float vx1 = (p1->x - p1->x_prev) / time_step;
    float vy1 = (p1->y - p1->y_prev) / time_step;
    float vz1 = (p1->z - p1->z_prev) / time_step;

    float vx2 = (p2->x - p2->x_prev) / time_step;
    float vy2 = (p2->y - p2->y_prev) / time_step;
    float vz2 = (p2->z - p2->z_prev) / time_step;

    float relative_vx = vx1 - vx2;
    float relative_vy = vy1 - vy2;
    float relative_vz = vz1 - vz2;

    float dot_product = dx * relative_vx + dy * relative_vy + dz * relative_vz;
    float collision_norm = dx * dx + dy * dy + dz * dz;

    float impulse = (1 + 0.5f) * dot_product / collision_norm;

    p1->x_prev = p1->x + impulse * dx;
    p1->y_prev = p1->y + impulse * dy;
    p1->z_prev = p1->z + impulse * dz;

    p2->x_prev = p2->x - impulse * dx;
    p2->y_prev = p2->y - impulse * dy;
    p2->z_prev = p2->z - impulse * dz;

    float new_vx1 = p1->x - p1->x_prev;
    float new_vy1 = p1->y - p1->y_prev;
    float new_vz1 = p1->z - p1->z_prev;

    float new_vx2 = p2->x - p2->x_prev;
    float new_vy2 = p2->y - p2->y_prev;
    float new_vz2 = p2->z - p2->z_prev;

    p1->ax = (new_vx1 - vx1) / (time_step * time_step);
    p1->ay = (new_vy1 - vy1) / (time_step * time_step) + GRAVITY;
    p1->az = (new_vz1 - vz1) / (time_step * time_step);

    p2->ax = (new_vx2 - vx2) / (time_step * time_step);
    p2->ay = (new_vy2 - vy2) / (time_step * time_step) + GRAVITY;
    p2->az = (new_vz2 - vz2) / (time_step * time_step);

    p1->collision_count++;
    p2->collision_count++;
}

void update_particles(Particle* particles, int num_particles, float dt) {
    for (int i = 0; i < num_particles; i++) {
        particles[i].ay = GRAVITY;

        float temp_x = particles[i].x;
        float temp_y = particles[i].y;
        float temp_z = particles[i].z;

        // update with verlet integration
        particles[i].x += (particles[i].x - particles[i].x_prev) + particles[i].ax * dt * dt;
        particles[i].y += (particles[i].y - particles[i].y_prev) + particles[i].ay * dt * dt;
        particles[i].z += (particles[i].z - particles[i].z_prev) + particles[i].az * dt * dt;

        // update prev with temp
        particles[i].x_prev = temp_x;
        particles[i].y_prev = temp_y;
        particles[i].z_prev = temp_z;


        // collision detection but future work use other algorithm to detect closest particles
        for (int j = i+1; j < num_particles; j++) {
            if (detect_collision(&particles[i], &particles[j])) {
                fix_position(&particles[i], &particles[j]);
            }
        }

        if (boolsphere) {
            float dist_from_center = sqrt(particles[i].x * particles[i].x + 
                                  particles[i].y * particles[i].y + 
                                  particles[i].z * particles[i].z);
    
            if (dist_from_center > sphere_radius - particles[i].radius) {
                float penetration = dist_from_center - (sphere_radius - particles[i].radius);
                float nx = particles[i].x / dist_from_center;
                float ny = particles[i].y / dist_from_center;
                float nz = particles[i].z / dist_from_center;

                particles[i].x -= penetration * nx;
                particles[i].y -= penetration * ny;
                particles[i].z -= penetration * nz;

                particles[i].x_prev = particles[i].x + random_perturbation() * perturbation_factor;
                particles[i].y_prev = particles[i].y + random_perturbation() * perturbation_factor;
                particles[i].z_prev = particles[i].z + random_perturbation() * perturbation_factor;

                particles[i].ax *= slowdown_factor;
                particles[i].ay *= slowdown_factor;
                particles[i].az *= slowdown_factor;
            }
        } else {
            // boundary conditions for the walls of the simulation tool
            float boundary_cushion = 0.1f; 
            float perturbation_factor = 0.01f; 

            if (particles[i].x < -1.0f + particles[i].radius) {
                particles[i].x = -1.0f + particles[i].radius;
                particles[i].x_prev = particles[i].x + random_perturbation() * perturbation_factor;
            } else if (particles[i].x > 1.0f - particles[i].radius) {
                particles[i].x = 1.0f - particles[i].radius;
                particles[i].x_prev = particles[i].x + random_perturbation() * perturbation_factor;
            }

            if (particles[i].y < -1.0f + particles[i].radius) {
                particles[i].y = -1.0f + particles[i].radius;
                particles[i].y_prev = particles[i].y + random_perturbation() * perturbation_factor;
            } else if (particles[i].y > 1.0f - particles[i].radius) {
                particles[i].y = 1.0f - particles[i].radius;
                particles[i].y_prev = particles[i].y + random_perturbation() * perturbation_factor;
            }

            if (particles[i].z < -1.0f + particles[i].radius) {
                particles[i].z = -1.0f + particles[i].radius;
                particles[i].z_prev = particles[i].z + random_perturbation() * perturbation_factor;
            } else if (particles[i].z > 1.0f - particles[i].radius) {
                particles[i].z = 1.0f - particles[i].radius;
                particles[i].z_prev = particles[i].z + random_perturbation() * perturbation_factor;
            }
        }
        
    }
}

void draw_particles(Particle* particles, int num_particles) {
    GLUquadricObj* quadric = gluNewQuadric();
    for (int i = 0; i < num_particles; i++) {
        glPushMatrix();
        glTranslatef(particles[i].x, particles[i].y, particles[i].z);
        
        // interpolate color
        float acc_magnitude = sqrt(particles[i].ax * particles[i].ax +
                                   particles[i].ay * particles[i].ay +
                                   particles[i].az * particles[i].az);

        float t = (acc_magnitude - 0.01f) / (0.05f - 0.01f); 
        if (t > 1.0f) t = 1.0f;
        if (t < 0.0f) t = 0.0f;

        float r, g, b;
        interpolate_color(t, &r, &g, &b);

        glColor4f(r, g, b, 0.5f);

        gluSphere(quadric, particles[i].radius, 16, 16);
        glPopMatrix();
    }
    gluDeleteQuadric(quadric);
}

void draw_boundary(){
    if (boolsphere) {
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
    gluPerspective(45.0, 800.0 / 600.0, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if (boolsphere) {
        gluLookAt(2.0, 2.0, 2.0,  // Camera position
          0.0, 0.0, 0.0,  // Look-at point
          0.0, 0.0, 1.0); // Up direction
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
