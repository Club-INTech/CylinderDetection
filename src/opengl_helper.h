//
// Created by jglrxavpok on 29/01/2020.
//

#ifndef CYLINDERDETECTION_OPENGL_HELPER_H
#define CYLINDERDETECTION_OPENGL_HELPER_H

#include <GLFW/glfw3.h>
#include <librealsense2/rs.hpp>
#include <GL/glu.h>

namespace vis {
    typedef GLFWwindow window;

    static GLuint _colorTexture;

    /**
     * Creates a window used for visualisation through GLFW
     * @param name window title
     * @param width window width in pixels
     * @param height window height in pixels
     * @return pointer to the window, or nullptr if an error occured
     */
    window* createWindow(const char* name, int width, int height);

    /**
     * Should this window be kept open?
     * @param window the window
     * @return true iff the window has not been closed by the user
     */
    bool keepOpen(window* window);

    /**
     * Renders a frame and prepares the window for the next frame
     * @param window
     */
    void endFrame(window* window);

    /**
     * Frees resources related to the given window
     * @param window
     */
    void cleanup(window* window);

    void renderPointCloud(rs2::points points);

    void createColorFrame();

    void uploadColorFrame(rs2::video_frame frame);
}

#endif //CYLINDERDETECTION_OPENGL_HELPER_H