#include "glwidget.h"

#include "collider_box.h"
#include "collider_sphere.h"

#include <QTimer>
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <QMouseEvent>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <glm/gtc/type_ptr.hpp>

using namespace std;

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
}

GLWidget::~GLWidget()
{
    for (auto o : objects_) {
        delete o;
    }

    delete rigid_bodies_system;
}


void GLWidget::initializeGL()
{    
    camera_.fov_y_ = glm::radians(80.0f);

    rigid_bodies_system = new RigidBodiesSystem();
    rigid_bodies_system->addForceField(*new ForceFieldGravity());
    rigid_bodies_system->addForceField(*new ForceFieldDrag(0.1));

    glewInit();

    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    scene_ = -1;
    uiScene1();

    rigid_bodies_system->coefficient_of_restitution_ = parent()->findChild<QDoubleSpinBox *>("coefficientOfRestitution")->value();
    rigid_bodies_system->fix_on_resting_ = parent()->findChild<QCheckBox *>("fixOnResting")->isChecked();

    target_frame_time_ = 1.0f / 60.0f;

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(target_frame_time_ * 1000.0f);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch"
void GLWidget::mousePressEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    rotate_ = true;
    rotate_last_x_ = event->x();
    rotate_last_y_ = event->y();
    break;
  }
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch"
void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  if (rotate_) {
    rotate_x_ = event->x() - rotate_last_x_;
    rotate_y_ = event->y() - rotate_last_y_;

    rotate_last_x_ = event->x();
    rotate_last_y_ = event->y();
  }
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch"
void GLWidget::mouseReleaseEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    rotate_ = false;
    break;
  }
}
#pragma GCC diagnostic pop

void GLWidget::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
  case Qt::Key_W:
    forward_ = true;
    break;
  case Qt::Key_S:
    backwards_ = true;
    break;
  case Qt::Key_A:
    left_ = true;
    break;
  case Qt::Key_D:
    right_ = true;
    break;
  }
}

void GLWidget::keyReleaseEvent(QKeyEvent *event) {
  switch (event->key()) {
  case Qt::Key_W:
    forward_ = false;
    break;
  case Qt::Key_S:
    backwards_ = false;
    break;
  case Qt::Key_A:
    left_ = false;
    break;
  case Qt::Key_D:
    right_ = false;
    break;
  }
}

void GLWidget::resizeGL(int w, int h)
{
    camera_.width_ = w;
    camera_.height_ = h;

    camera_.viewport();
}

void GLWidget::paintGL() {
    chrono::steady_clock::time_point time_now = chrono::steady_clock::now();
    if (first_paint_) {
        first_paint_ = false;
        dt_ = target_frame_time_;
    }else {
        dt_ = chrono::duration_cast<chrono::nanoseconds>(time_now - time_last_).count() * 1e-9;
    }

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (forward_) {
      camera_.forward(dt_);
    }
    if (backwards_) {
      camera_.backwards(dt_);
    }
    if (left_) {
      camera_.left(dt_);
    }
    if (right_) {
      camera_.right(dt_);
    }

    if (rotate_) {
        camera_.rotate(rotate_x_, rotate_y_, dt_);
        rotate_x_ = 0;
        rotate_y_ = 0;
    }

    camera_.compute_view_projection();

    dt_ = target_frame_time_; // TODO DEBUG

    rigid_bodies_system->update(dt_);

    for (auto p : paint_gl_) {
        p->paintGL(dt_, camera_);
    }

    time_last_ = time_now;
}

void GLWidget::clear()
{
    for (auto o : objects_) {
        delete o;
    }
    objects_.clear();
    paint_gl_.clear();
    rigid_bodies_system->clear();
}

void GLWidget::uiScene1()
{
    clear();

    if (scene_ != 1) {
        camera_.pos_ = glm::vec3(-3.0, 3.0, -3.0);
        camera_.lookAt(glm::vec3(0.0, 2.0, 0.0));
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/sphere.ply", *m);

        Object *sphere = new Object(glm::vec3(0.0f, 4.0, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(0.0, 0.0, 1.0, 1.0));
        objects_.push_back(sphere);
        paint_gl_.push_back(sphere);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderSphere(0.5f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        sphere->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/cube.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(1.0, 0.0, 0.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(1.0f, 1.0f, 1.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        rigid_body->fixed_ = true;
        box->setRigibody(*rigid_body);
    }

    for (auto p : paint_gl_) {
        p->initialieGL();
    }

    scene_ = 1;
}

void GLWidget::uiScene2()
{
    clear();

    if (scene_ != 2) {
        camera_.pos_ = glm::vec3(-3.0, 1.0, -3.0);
        camera_.lookAt(glm::vec3(0.0, 2.0, 0.0));
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/sphere.ply", *m);

        Object *sphere = new Object(glm::vec3(0.0f, 4.0, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(0.0, 0.0, 1.0, 1.0));
        objects_.push_back(sphere);
        paint_gl_.push_back(sphere);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderSphere(0.5f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        sphere->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/sphere.ply", *m);

        Object *sphere = new Object(glm::vec3(0.0f, 8.0, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(0.0, 1.0, 1.0, 1.0));
        objects_.push_back(sphere);
        paint_gl_.push_back(sphere);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderSphere(0.5f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        sphere->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/cube.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(1.0, 0.0, 0.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(1.0f, 1.0f, 1.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        rigid_body->fixed_ = true;
        box->setRigibody(*rigid_body);
    }

    for (auto p : paint_gl_) {
        p->initialieGL();
    }

    scene_ = 2;
}

void GLWidget::uiScene3()
{
    clear();

    if (scene_ != 3) {
        camera_.pos_ = glm::vec3(-3.0, 1.0, 0.0);
        camera_.lookAt(glm::vec3(0.0, 1.0, 0.0));
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/cube.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 5.0f, 0.41f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(1.0, 0.0, 0.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(1.0f, 1.0f, 1.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        box->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/sphere.ply", *m);

        Object *sphere = new Object(glm::vec3(0.0f, 0.0, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(0.0, 0.0, 1.0, 1.0));
        objects_.push_back(sphere);
        paint_gl_.push_back(sphere);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderSphere(0.5f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        rigid_body->fixed_ = true;
        sphere->setRigibody(*rigid_body);
    }

    for (auto p : paint_gl_) {
        p->initialieGL();
    }

    scene_ = 3;
}

glm::quat eulerToQuat(const glm::vec3 &e)
{
    float sx = sin(e.x/2);
    float sy = sin(e.y/2);
    float sz = sin(e.z/2);
    float cx = cos(e.x/2);
    float cy = cos(e.y/2);
    float cz = cos(e.z/2);

    return glm::quat(cx * cy * cz + sx * sy * sz,
                     sx * cy * cz - cx * sy * sz,
                     cx * sy * cz + sx * cy * sz,
                     cx * cy * sz - sx * sy * cz);
}

void GLWidget::uiScene4()
{
    clear();

    if (scene_ != 4) {
        camera_.pos_ = glm::vec3(-4.0, 2, -1.0);
        camera_.lookAt(glm::vec3(0.0, 0.5, 0.0));
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/cube.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 3.0f, 0.25 + 0.707106781f),  eulerToQuat(glm::radians(glm::vec3(0.0, 45.0, 0.0))), *m, glm::vec4(1.0, 0.0, 1.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(1.0f, 1.0f, 1.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        box->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/cube.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(1.0, 0.0, 0.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(1.0f, 1.0f, 1.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        rigid_body->fixed_ = true;
        box->setRigibody(*rigid_body);
    }

    for (auto p : paint_gl_) {
        p->initialieGL();
    }

    scene_ = 4;
}

void GLWidget::uiScene5()
{
    clear();

    if (scene_ != 5) {
        camera_.pos_ = glm::vec3(-7.0, 4.0, -6.0);
        camera_.lookAt(glm::vec3(0.0, 0.0, 0.0));
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/sphere.ply", *m);

        Object *sphere = new Object(glm::vec3(0.0f, 16.0, 0.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(0.0, 1.0, 1.0, 1.0));
        objects_.push_back(sphere);
        paint_gl_.push_back(sphere);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderSphere(0.5f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        sphere->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/cube.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 8.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(0.0, 1.0, 0.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(1.0f, 1.0f, 1.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        box->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/sphere.ply", *m);

        Object *sphere = new Object(glm::vec3(0.0f, 6.0, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(0.0, 0.0, 1.0, 1.0));
        objects_.push_back(sphere);
        paint_gl_.push_back(sphere);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderSphere(0.5f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        sphere->setRigibody(*rigid_body);
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/box_10_1_10.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 1.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(1.0, 0.0, 0.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(10.0f, 1.0f, 10.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        rigid_body->fixed_ = true;
        box->setRigibody(*rigid_body);
    }

    for (auto p : paint_gl_) {
        p->initialieGL();
    }

    scene_ = 5;
}

float random(float l, float h)
{
    return l + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX / (h - l)));
}

void GLWidget::uiScene6()
{
    clear();

    if (scene_ != 6) {
        camera_.pos_ = glm::vec3(-7.0, 4.0, -6.0);
        camera_.lookAt(glm::vec3(0.0, 0.0, 0.0));
    }

    const unsigned int num_balls = 20;

    vector<glm::vec3> balls_pos;
    balls_pos.reserve(num_balls);

    while (balls_pos.size() < num_balls) {
        glm::vec3 pos = glm::vec3(random(-3, 3), random(3, 15), random(-3, 3));

        bool save = true;
        for (auto &b : balls_pos) {
            if (glm::length(b - pos) < 2.0) {
                save = false;
                break;
            }
        }

        if (save) {
            {
                Mesh *m = new Mesh();
                Mesh::ReadFromPly("../../res/model/sphere.ply", *m);

                Object *sphere = new Object(pos, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(random(0.0, 1.0), random(0.0, 1.0), random(0.0, 1.0), 1.0));
                objects_.push_back(sphere);
                paint_gl_.push_back(sphere);

                RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderSphere(0.5f));
                rigid_bodies_system->addRigidBody(*rigid_body);
                sphere->setRigibody(*rigid_body);
            }

            balls_pos.push_back(pos);
        }
    }

    {
        Mesh *m = new Mesh();
        Mesh::ReadFromPly("../../res/model/box_10_1_10.ply", *m);

        Object *box = new Object(glm::vec3(0.0f, 1.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), *m, glm::vec4(1.0, 0.0, 0.0, 1.0));
        objects_.push_back(box);
        paint_gl_.push_back(box);

        RigidBody *rigid_body = new RigidBody(1.0f, *new ColliderBox(10.0f, 1.0f, 10.0f));
        rigid_bodies_system->addRigidBody(*rigid_body);
        rigid_body->fixed_ = true;
        box->setRigibody(*rigid_body);
    }

    for (auto p : paint_gl_) {
        p->initialieGL();
    }

    scene_ = 6;
}

void GLWidget::uiCoefficientOfRestitution(double v)
{
    rigid_bodies_system->coefficient_of_restitution_ = v;
}

void GLWidget::uiFixOnResting(bool v)
{
    rigid_bodies_system->fix_on_resting_ = v;
}
