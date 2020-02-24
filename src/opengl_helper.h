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
    static GLUquadric* _quadric;

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

    /**
     * Renders a cylinder in immediate mode
     * @param centerX cylinder center X
     * @param centerY cylinder center Y
     * @param centerZ cylinder center Z
     * @param directionX cylinder main axis direction X
     * @param directionY cylinder main axis direction Y
     * @param directionZ cylinder main axis direction Z
     * @param height cylinder height
     * @param radius cylinder height
     * @param r red component of color
     * @param g green component of color
     * @param b blue component of color
     */
    void renderCylinder(float centerX, float centerY, float centerZ, float directionX, float directionY, float directionZ, float height, float radius, float r, float g, float b);

    /**
     * Prepares for an OpenGL render inside world space
     */
    void setupMatrices();
}

#endif //CYLINDERDETECTION_OPENGL_HELPER_H