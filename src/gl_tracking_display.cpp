#include <cmath>
#include <iostream>

#include "gl_tracking_display.h"

//===========================================================================
GLTrackingDisplay::GLTrackingDisplay(QWidget *parent) : QOpenGLWidget(parent)
{
    x_human_ = 0.0; y_human_ = 0.0; th_human_ = 0.0;
    x_robot_ = 1.0; y_robot_ = 0.0; th_robot_ = 0.0;
}

//===========================================================================
GLTrackingDisplay::~GLTrackingDisplay()
{
}

//===========================================================================
void GLTrackingDisplay::initializeGL()
{
    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
}

//===========================================================================
void GLTrackingDisplay::resizeGL(int w, int h)
{
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (float)w/h, 0.01, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0,3.5,10,0.0,3.5,0,0,1,0);
}

//===========================================================================
void GLTrackingDisplay::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    draw_human();
    draw_robot();
}

//===========================================================================
void GLTrackingDisplay::set_robot_pose(double x_new, double y_new, double th_new)
{
    x_robot_ = x_new;
    y_robot_ = y_new;
    th_robot_ = th_new;
}

//===========================================================================
void GLTrackingDisplay::set_human_pose(double x_new, double y_new, double th_new)
{
    x_human_ = x_new;
    y_human_ = y_new;
    th_human_ = th_new;
}

//===========================================================================
void GLTrackingDisplay::draw_human()
{
    // calculate the vertices
    std::vector<std::pair<double, double>> vertices;
    this->calc_vertices_triangle(x_human_, y_human_, th_human_, 0.5, 0.3, vertices);

    glBegin(GL_TRIANGLES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(vertices[0].first, vertices[0].second, 0);
        glVertex3d(vertices[1].first, vertices[1].second, 0);
        glVertex3d(vertices[2].first, vertices[2].second, 0);
    glEnd();
}

//===========================================================================
void GLTrackingDisplay::draw_robot()
{
    // calculate the vertices
    std::vector<std::pair<double, double>> vertices;
    this->calc_vertices_triangle(x_robot_, y_robot_, th_robot_, 1.0, 0.2, vertices);

    glBegin(GL_TRIANGLES);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(vertices[0].first, vertices[0].second, 0);
        glVertex3d(vertices[1].first, vertices[1].second, 0);
        glVertex3d(vertices[2].first, vertices[2].second, 0);
    glEnd();
}

//===========================================================================
void GLTrackingDisplay::calc_vertices_triangle(double x, double y, double th, double alpha,
                                               double len, std::vector<std::pair<double, double> > &vertices)
{
    double xv, yv;
    vertices.clear();

    // calcualte the first point
    xv = x;
    yv = y;
    vertices.push_back({xv, yv});

    // calculate the second and third point
    xv = x + len * std::cos(th + alpha * 0.5);
    yv = y + len * std::sin(th + alpha * 0.5);
    vertices.push_back({xv, yv});

    xv = x + len * std::cos(th - alpha * 0.5);
    yv = y + len * std::sin(th - alpha * 0.5);
    vertices.push_back({xv, yv});
}
