#-------------------------------------------------
#
# Project created by QtCreator 2016-03-10T18:26:59
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QMAKE_CXXFLAGS += -std=gnu++0x

TARGET = my_BRRTstar
TEMPLATE = app
INCLUDEPATH += /home/gary/programmer/googletest/googletest-master/googletest/include/
INCLUDEPATH += /home/gary/programmer/googletest/googletest-master/googletest/src
INCLUDEPATH += /home/gary/Desktop/eigen-eigen-07105f7124f9
INCLUDEPATH += /home/gary/programmer/project/qtproject/my_BRRTstar

SOURCES +=\
    2dplane/2dplane.cpp \
    2dplane/obstaclegrid.cpp \
    2dplane/GridStateSpace.cpp \
    2dplane/PlaneStateSpace.cpp \
    rrt-viewer/main.cpp \
    rrt-viewer/MainWindow.cpp \
    rrt-viewer/RRTWidget.cpp \
    3dplane/GridStateSpace3d.cpp \
    3dplane/ObstacleGrid3d.cpp \
    3dplane/PlaneStateSpace3d.cpp \
    3dplane/3dplane.cpp

HEADERS  += \
    statespace.h \
    tree.h \
    rrtstartree.h \
    birrt.h \
    2dplane/2dplane.h \
    2dplane/obstaclegrid.h \
    2dplane/planestatespace.h \
    2dplane/GridStateSpace.hpp \
    planning/Path.hpp \
    rrt-viewer/MainWindow.hpp \
    rrt-viewer/RRTWidget.hpp \
    util.h \
    birrtstar.h \
    node.h \
    3dplane/GridStateSpace3d.h \
    3dplane/ObstacleGrid3d.h \
    3dplane/PlaneStateSpace3d.h \
    3dplane/3dplane.h \
    bezier2d.h

FORMS    += mainwindow.ui


unix:!macx: LIBS += -L$$PWD/../../../googletest/googletest-master/googletest/mybuild/ -lgtest

INCLUDEPATH += $$PWD/../../../googletest/googletest-master/googletest/mybuild
DEPENDPATH += $$PWD/../../../googletest/googletest-master/googletest/mybuild

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../googletest/googletest-master/googletest/mybuild/libgtest.a

unix:!macx: LIBS += -L$$PWD/../../../googletest/googletest-master/googletest/mybuild/ -lgtest_main

INCLUDEPATH += $$PWD/../../../googletest/googletest-master/googletest/mybuild
DEPENDPATH += $$PWD/../../../googletest/googletest-master/googletest/mybuild

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../googletest/googletest-master/googletest/mybuild/libgtest_main.a
