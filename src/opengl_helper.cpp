//
// Created by jglrxavpok on 29/01/2020.
//

#include "opengl_helper.h"

void checkGLError(const char* where) {
    GLuint glError = glGetError();
    if(glError) {
        fprintf(stderr, "GL Error in [%s] %d = %s\n", where, glError, gluErrorString(glError));
    }
}

vis::window *vis::createWindow(const char* name, int width, int height) {
    if(!glfwInit()) {
        return nullptr;
    }

    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    window* window = glfwCreateWindow(width, height, name, nullptr, nullptr);
    if(window) {
        glfwMakeContextCurrent(window);

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glViewport(0, 0, width, height);

        glMatrixMode(GL_PROJECTION);

        createColorTexture();
        createDepthTexture();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    return window;
}

bool vis::keepOpen(vis::window *window) {
    return !glfwWindowShouldClose(window);
}

void vis::endFrame(vis::window *window) {
    glPopMatrix();
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void vis::cleanup(vis::window *window) {
    glfwDestroyWindow(window);
    glfwTerminate();
}

void vis::setupMatrices() {
    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    // prepare OpenGL context
    glPushMatrix();
    glLoadIdentity();
}

void vis::createDepthTexture() {
    GLuint texture;
    glGenTextures(1, &texture);

    _depthTexture = texture;
}

void vis::createColorTexture() {
    GLuint texture;
    glGenTextures(1, &texture);

    _colorTexture = texture;
}

void vis::uploadDepthFrame(rs2::video_frame& falseColors) {
    uploadToTexture(_depthTexture, falseColors);
    _hasDepth = true;
}

void vis::uploadVideoFrame(rs2::video_frame& frame) {
    uploadToTexture(_colorTexture, frame);
    _hasVideo = true;
}

void vis::uploadToTexture(GLuint texture, rs2::video_frame& frame) {
    // from https://github.com/IntelRealSense/librealsense/blob/master/examples/example.hpp
    if (!frame) return;

/*    GLenum err = glGetError();
    if(err) {
        printf("GL error: %d\n", err);
    }*/

    auto format = frame.get_profile().format();
    auto width = frame.get_width();
    auto height = frame.get_height();

    glBindTexture(GL_TEXTURE_2D, texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    switch (format)
    {
        case RS2_FORMAT_RGB8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.get_data());
            break;
        case RS2_FORMAT_RGBA8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, frame.get_data());
            break;
        case RS2_FORMAT_Y8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, frame.get_data());
            break;
        case RS2_FORMAT_Y10BPACK:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_SHORT, frame.get_data());
            break;
        default:
            throw std::runtime_error("The requested format is not supported by this demo!");
    }

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    //glBindTexture(GL_TEXTURE_2D, 0);
}

void drawTexture(GLuint texture, float minX, float minY, float maxX, float maxY) {
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    glBindTexture(GL_TEXTURE_2D, texture);
    glEnable(GL_TEXTURE_2D);

    glBegin(GL_QUADS);
    {
        glTexCoord2f(0.0f, 1.0f);
        glVertex2f(minX, minY);

        glTexCoord2f(0.0f, 0.0f);
        glVertex2f(minX, maxY);

        glTexCoord2f(1.0f, 0.0f);
        glVertex2f(maxX, maxY);

        glTexCoord2f(1.0f, 1.0f);
        glVertex2f(maxX, minY);
    }
    glEnd();
}

void vis::render(window* window) {
    setupMatrices();
    if(_hasVideo) {
        drawTexture(_colorTexture, -1.0f, -1.0f, 0.0f, 1.0f);
    }
    if(_hasDepth) {
        drawTexture(_depthTexture, 0.0f, -1.0f, 1.0f, 1.0f);
    }
    endFrame(window);

    checkGLError("frame end");
}