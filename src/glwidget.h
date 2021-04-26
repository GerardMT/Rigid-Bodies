#ifndef GLWIDGET_H_
#define GLWIDGET_H_

#include "camera.h"
#include "force_field_drag.h"
#include "force_field_gravity.h"
#include "object.h"
#include "rigid_bodies_system.h"

#include <GL/glew.h>
#include <QGLWidget>
#include <QOpenGLShaderProgram>

using namespace std;

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    explicit GLWidget(QWidget *parent = nullptr);

    ~GLWidget();

protected:
    void mousePressEvent(QMouseEvent *event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;

    void keyReleaseEvent(QKeyEvent *event) override;

    void initializeGL() override;

    void resizeGL(int w, int h) override;

    void paintGL() override;

    void clear();

private:
    Camera camera_;

    vector<Object *> objects_;

    vector<PaintGL *> paint_gl_;

    RigidBodiesSystem *rigid_bodies_system = nullptr;

    float dt_;

    chrono::steady_clock::time_point  time_last_;

    bool rotate_ = false;

    int rotate_x_ = 0;
    int rotate_y_ = 0;

    int rotate_last_x_;
    int rotate_last_y_;

    bool forward_ = false;
    bool backwards_ = false;
    bool left_ = false;
    bool right_ = false;

    bool first_paint_ = true;
    float target_frame_time_;

    unsigned int scene_;

private slots:
    void uiScene1();

    void uiScene2();

    void uiScene3();

    void uiScene4();

    void uiScene5();

    void uiScene6();

    void uiCoefficientOfRestitution(double v);

    void uiFixOnResting(bool v);
};

#endif  //  GLWIDGET_H_
