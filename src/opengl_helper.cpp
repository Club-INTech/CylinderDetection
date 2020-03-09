//
// Created by jglrxavpok on 29/01/2020.
//

#include "opengl_helper.h"

vis::window *vis::createWindow(const char* name, int width, int height) {
    if(!glfwInit()) {
        return nullptr;
    }

    window* window = glfwCreateWindow(width, height, name, nullptr, nullptr);
    if(window) {
        glfwMakeContextCurrent(window);

        glViewport(0, 0, width, height);
        glOrtho(0, width, height, 0, -1, +1);
        createColorTexture();
        createDepthTexture();

        _quadric = gluNewQuadric();
        gluQuadricDrawStyle(_quadric, GLU_FILL);
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
    glPopAttrib();

    glfwSwapBuffers(window);
    glfwPollEvents();
}

void vis::cleanup(vis::window *window) {
    glfwDestroyWindow(window);
    glfwTerminate();
}

void vis::setupMatrices() {
    // prepare OpenGL context
    glLoadIdentity();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT);
}

void vis::createDepthTexture() {
    GLuint texture;
    glGenTextures(1, &texture);

    glBindTexture(GL_TEXTURE_2D, texture);
    _depthTexture = texture;
}

void vis::createColorTexture() {
    GLuint texture;
    glGenTextures(1, &texture);

    glBindTexture(GL_TEXTURE_2D, texture);
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

    GLenum err = glGetError();
    if(err) {
        printf("GL error: %d\n", err);
    }

    auto format = frame.get_profile().format();
    auto width = frame.get_width();
    auto height = frame.get_height();

    glBindTexture(GL_TEXTURE_2D, texture);

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
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void drawTexture(GLuint texture, float minX, float minY, float maxX, float maxY) {
    glBindTexture(GL_TEXTURE_2D, texture);

    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    {
        glTexCoord2f(0.0f, 0.0f);
        glVertex2f(minX, minY);

        glTexCoord2f(1.0f, 0.0f);
        glVertex2f(maxX, minY);

        glTexCoord2f(1.0f, 1.0f);
        glVertex2f(maxX, maxY);

        glTexCoord2f(0.0f, 1.0f);
        glVertex2f(minX, maxY);
    }
    glEnd();
}

void vis::render(window* window) {
    setupMatrices();
    if(_hasVideo) {
        drawTexture(_colorTexture, 0.0f, 0.0f, 0.5f, 1.0f);
    }
    if(_hasDepth) {
        drawTexture(_depthTexture, 0.0f, 0.0f, 0.5f, 1.0f);
    }
    endFrame(window);
}