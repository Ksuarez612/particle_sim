#include <GLFW/glfw3.h>
#include <windows.h>
#include <GL/glu.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

typedef struct {
    float x, y, z;       // curr position
    float x_prev, y_prev, z_prev; // previous position
    float ax, ay, az;    // acceleration
    float radius; // radius of particle -> collision detection
} Particle;

void initialize_particles(Particle* particles, int num_particles);
void update_particles(Particle* particles, int num_particles, float dt);
void draw_particles(Particle* particles, int num_particles);
void draw_boundary();
void initOpenGL();
void draw();

int num_particles = 100;
Particle* particles;
const float GRAVITY = -0.04f;

void interpolate_color(float t, float* r, float* g, float* b) {
    float r_low = 0.678f, g_low = 0.847f, b_low = 1.0f;
    float r_high = 1.0f, g_high = 0.0f, b_high = 0.0f;

    *r = (1 - t) * r_low + t * r_high;
    *g = (1 - t) * g_low + t * g_high;
    *b = (1 - t) * b_low + t * b_high;
}

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
        update_particles(particles, num_particles, 0.01f); // assuming fixed time, move particles randomly
        draw();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    free(particles); // remove particles from mem 
    glfwTerminate();
    return 0;
}

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

void update_particles(Particle* particles, int num_particles, float dt) {
    for (int i = 0; i < num_particles; i++) {
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
            float dx = particles[i].x - particles[j].x;
            float dy = particles[i].y - particles[j].y;
            float dz = particles[i].z - particles[j].z;

            float distance = sqrt(dx * dx + dy * dy + dz * dz);
            float min_distance = particles[i].radius + particles[j].radius;
            
            // linear algebra to calculate the collisions 
            if (distance < min_distance) { // collision here 
                float nx = dx / distance; // normal vectors
                float ny = dy / distance; 
                float nz = dz / distance; 

                float vx1 = (particles[i].x - particles[i].x_prev) / dt;
                float vy1 = (particles[i].y - particles[i].y_prev) / dt;
                float vz1 = (particles[i].z - particles[i].z_prev) / dt;

                float vx2 = (particles[j].x - particles[j].x_prev) / dt;
                float vy2 = (particles[j].y - particles[j].y_prev) / dt;
                float vz2 = (particles[j].z - particles[j].z_prev) / dt;

                // Calculate relative velocity in terms of the normal direction
                float rel_vel = vx1 * nx + vy1 * ny + vz1 * nz - (vx2 * nx + vy2 * ny + vz2 * nz);

                // change velocities
                particles[i].x_prev = particles[i].x - (vx1 - rel_vel * nx) * dt;
                particles[i].y_prev = particles[i].y - (vy1 - rel_vel * ny) * dt;
                particles[i].z_prev = particles[i].z - (vz1 - rel_vel * nz) * dt;

                particles[j].x_prev = particles[j].x - (vx2 + rel_vel * nx) * dt;
                particles[j].y_prev = particles[j].y - (vy2 + rel_vel * ny) * dt;
                particles[j].z_prev = particles[j].z - (vz2 + rel_vel * nz) * dt;

                // Correct positions to prevent overlap
                float overlap = 0.5f * (min_distance - distance);
                particles[i].x += nx * overlap;
                particles[i].y += ny * overlap;
                particles[i].z += nz * overlap;

                particles[j].x -= nx * overlap;
                particles[j].y -= ny * overlap;
                particles[j].z -= nz * overlap;
            }
        }

        // boundary conditions for the walls of the simulation tool
        if (particles[i].x < -1.0f + particles[i].radius) {
            particles[i].x = -1.0f + particles[i].radius;
            particles[i].ax *= -0.3;
        } else if (particles[i].x > 1.0f - particles[i].radius) {
            particles[i].x = 1.0f - particles[i].radius;
            particles[i].ax *= -0.3;
        }

        if (particles[i].y < -1.0f + particles[i].radius) {
            particles[i].y = -1.0f + particles[i].radius;
            particles[i].ay *= -0.3;
        } else if (particles[i].y > 1.0f - particles[i].radius) {
            particles[i].y = 1.0f - particles[i].radius;
            particles[i].ay *= -0.3;
        }

        if (particles[i].z < -1.0f + particles[i].radius) {
            particles[i].z = -1.0f + particles[i].radius;
            particles[i].az *= -0.3;
        } else if (particles[i].z > 1.0f - particles[i].radius) {
            particles[i].z = 1.0f - particles[i].radius;
            particles[i].az *= -0.3;
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
    glColor3f(1.0f, 1.0f, 1.0f); // Set boundary color to white
    glBegin(GL_LINES);

    // Draw lines for the boundary box
    // Bottom face
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);

    glVertex3f(1.0f, -1.0f, -1.0f);
    glVertex3f(1.0f, -1.0f, 1.0f);

    glVertex3f(1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);

    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);

    // Top face
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, -1.0f);

    glVertex3f(1.0f, 1.0f, -1.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);

    glVertex3f(1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, 1.0f);

    glVertex3f(-1.0f, 1.0f, 1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);

    // Vertical lines
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

void initOpenGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND); // Enable blending
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.0, 0.0, 0.0, 1.0);
}

void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Setup camera
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 800.0 / 600.0, 1.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(3.0, 3.0, 3.0,  // Camera position
          0.0, 0.0, 0.0,  // Look-at point
          0.0, 1.0, 0.0); // Up direction
    draw_boundary();

    // Draw particles
    draw_particles(particles, num_particles);

    glFlush();
}
