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
        createColorFrame();

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
    glMatrixMode(GL_PROJECTION);
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
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, 16.0/9.0, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);


    glEnable(GL_DEPTH_TEST);
}

void vis::renderPointCloud(rs2::points points) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, _colorTexture);

    glPointSize(2.0);
    glBegin(GL_POINTS);
    /* this segment actually prints the pointcloud */
    auto vertices = points.get_vertices();              // get vertices
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates

    for (int i = 0; i < points.size(); i++)
    {
        if(i % UNDERSAMPLING != 0)
            continue;
        //if (vertices[i].z)                // upload the point and texture coordinates only for points we have depth data for

        {
            //if(ABS(vertices[i].z) < 0.5f) {
                glVertex3fv(vertices[i]);
                glTexCoord2fv(tex_coords[i]);
            //}
        }
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void vis::createColorFrame() {
    GLuint texture;
    glGenTextures(1, &texture);

    glBindTexture(GL_TEXTURE_2D, texture);
    _colorTexture = texture;
}

void vis::uploadColorFrame(rs2::video_frame frame) {
    // from https://github.com/IntelRealSense/librealsense/blob/master/examples/example.hpp
    if (!frame) return;

    GLenum err = glGetError();
    if(err) {
        printf("GL error: %d\n", err);
    }

    auto format = frame.get_profile().format();
    auto width = frame.get_width();
    auto height = frame.get_height();

    glBindTexture(GL_TEXTURE_2D, _colorTexture);

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

void vis::renderCylinder(float centerX, float centerY, float centerZ, float directionX, float directionY, float directionZ, float height, float radius, float r, float g, float b) {
    glPushMatrix();
    glColor4f(r, g, b, 1.0f);
    glTranslatef(centerX, centerY, centerZ);
    glTranslatef(0, 0, -height/2);
    glRotated(-90.0, 1.0, 0.0, 0.0);
    gluCylinder(_quadric, radius, radius, height, 64, 64);
    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}