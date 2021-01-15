#ifndef OBJECT_H
#define OBJECT_H

#include "collider.h"
#include "mesh.h"
#include "paint_gl.h"
#include "rigid_body.h"

#include <QOpenGLShaderProgram>

class Object : public PaintGL
{
public:
    Object(glm::vec3 pos, glm::quat rot, Mesh &m, glm::vec4 color);

    ~Object();

    void setRigibody(RigidBody &r);

    void initialieGL() override;

    void paintGL(float dt, const Camera &camera) override;

    const glm::vec3 &pos() const;

    const glm::quat &rot() const;

private:
    Mesh *mesh_;

    RigidBody *rigid_body_;

    QOpenGLShaderProgram program_;

    glm::vec4 color_;

    GLuint vao_;
    GLuint vbo_;
    GLuint nbo_;
    GLuint fbo_;

    glm::vec3 pos_;
    glm::quat rot_;
};

#endif // OBJECT_H
