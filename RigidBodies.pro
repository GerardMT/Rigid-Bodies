QT += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RigidBodies
TEMPLATE = app

CONFIG += c++14
CONFIG(release, release|debug):QMAKE_CXXFLAGS += -Wall -O2

CONFIG(release, release|debug):DESTDIR = release/
CONFIG(release, release|debug):OBJECTS_DIR = release/
CONFIG(release, release|debug):MOC_DIR = release/
CONFIG(release, release|debug):UI_DIR = release/

CONFIG(debug, release|debug):DESTDIR = debug/
CONFIG(debug, release|debug):OBJECTS_DIR = debug/
CONFIG(debug, release|debug):MOC_DIR = debug/
CONFIG(debug, release|debug):UI_DIR = debug/

INCLUDEPATH += /usr/include/eigen3/

LIBS += -lGLEW

SOURCES += \
    camera.cpp \
    collider_box.cpp \
    collider_sphere.cpp \
    force_field_drag.cpp \
    force_field_gravity.cpp \
    main.cc \
    main_window.cc \
    glwidget.cc \
    mesh.cpp \
    object.cpp \
    rigid_bodies_system.cpp \
    rigid_body.cpp \
    util.cpp

HEADERS  += \
    camera.h \
    collider.h \
    collider_box.h \
    collider_sphere.h \
    force_field.h \
    force_field_gravity.h \
    force_field_drag.h \
    force_field_gravity.h \
    main_window.h \
    glwidget.h \
    mesh.h \
    object.h \
    paint_gl.h \
    rigid_bodies_system.h \
    rigid_body.h \
    util.h

FORMS    += \
    main_window.ui

RESOURCES += \
    shader/object.frag \
    shader/object.vert

