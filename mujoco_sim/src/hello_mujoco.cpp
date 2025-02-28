#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// Path to your converted MJCF model
const char* model_path = "/mujoco/src/description/perfbalanced.xml";

// Simulation objects
mjModel* model = nullptr;
mjData* data = nullptr;
mjvCamera camera;
mjvOption opt;
mjvScene scene;
mjrContext context;

// GLFW window
GLFWwindow* window = nullptr;

// Keyboard callback for closing simulation
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

int main() {
    std::cout << "Loading MuJoCo model: " << model_path << std::endl;

    // Load the model
    char error[1000] = "Could not load model";
    model = mj_loadXML(model_path, nullptr, error, 1000);
    if (!model) {
        std::cerr << "Error loading model: " << error << std::endl;
        return 1;
    }

    // Create simulation data
    data = mj_makeData(model);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return 1;
    }

    // Create a window
    window = glfwCreateWindow(800, 600, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Could not create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, keyboard);

    // Initialize visualization data
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    mjv_makeScene(model, &scene, 2000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    // Run the simulation loop
    while (!glfwWindowShouldClose(window)) {
        // Step the simulation
        mj_step(model, data);

        // Get framebuffer size
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        // Update camera and render
        mjrRect viewport = {0, 0, width, height};
        mjv_updateScene(model, data, &opt, NULL, &camera, mjCAT_ALL, &scene);
        mjr_render(viewport, &scene, &context);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    mj_deleteData(data);
    mj_deleteModel(model);
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}