#ifndef GL_TRACKING_DISPLAY_H
#define GL_TRACKING_DISPLAY_H

#include <QWidget>
#include <QOpenGLWidget>
#include "GL/glu.h"
#include "GL/gl.h"

#include <vector>
#include <utility>

class GLTrackingDisplay : public QOpenGLWidget
{
    Q_OBJECT
public:
    GLTrackingDisplay(QWidget *parent = 0);
    ~GLTrackingDisplay();

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

public slots:
    void set_robot_pose(double x_new, double y_new, double th_new);
    void set_human_pose(double x_new, double y_new, double th_new);

private:
    double x_robot_;
    double y_robot_;
    double th_robot_;
    double x_human_;
    double y_human_;
    double th_human_;

    void calc_vertices_triangle(double x, double y, double th, double alpha, double len,
                                std::vector<std::pair<double, double>> &vertices);
    void draw_robot();
    void draw_human();
};

#endif // GL_TRACKING_DISPLAY_H
