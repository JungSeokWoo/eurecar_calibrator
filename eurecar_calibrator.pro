#-------------------------------------------------
#
# Project created by QtCreator 2017-09-19T19:19:35
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = eurecar_calibrator
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        g_main_window.cpp \
    imgproc/c_camcalib.cpp \
    imgproc/util_funcs.cpp \
    3d_view/c_3d_viewer.cpp \
    custom_qt/c_custom_scene.cpp \
    thread/c_sceneupdate.cpp \
    algorithm/c_fitting.cpp

HEADERS += \
        g_main_window.h \
    imgproc/c_camcalib.h \
    imgproc/util_funcs.hpp \
    3d_view/c_3d_viewer.h \
    custom_qt/c_custom_scene.h \
    thread/c_sceneupdate.h \
    algorithm/c_fitting.h

FORMS += \
        g_main_window.ui



# Opencv libs --------------------------------------

INCLUDEPATH += /usr/local/include

LIBS += -L/usr/local/lib    \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_imgcodecs \
    -lopencv_video \
    -lopencv_videoio \
    -lopencv_calib3d

# etc ----------------------------------------------
LIBS += -L/usr/lib/x86_64-linux-gnu \
    -lboost_system  \
    -lboost_filesystem \
    -lboost_thread
# --------------------------------------------------

# vtk && pcl --------------------------------------
INCLUDEPATH += /usr/local/include/vtk-8.0  \
    /usr/local/include/pcl-1.8 \
    /usr/include/eigen3


LIBS += -L/usr/local/lib \
    -lvtkpng-8.0 \
    -lvtksys-8.0 \
    -lvtkglew-8.0 \
    -lvtkhdf5-8.0 \
    -lvtkjpeg-8.0 \
    -lvtktiff-8.0 \
    -lvtkzlib-8.0 \
    -lvtkexpat-8.0 \
    -lvtkexpat-8.0 \
    -lvtkgl2ps-8.0 \
    -lvtkIOAMR-8.0 \
    -lvtkIOPLY-8.0 \
    -lvtkIOSQL-8.0 \
    -lvtkIOXML-8.0 \
    -lvtkproj4-8.0 \
    -lvtkalglib-8.0 \
    -lvtkIOCore-8.0 \
    -lvtkIOMINC-8.0 \
    -lvtkmetaio-8.0 \
    -lvtkNetCDF-8.0 \
    -lvtksqlite-8.0 \
    -lvtkhdf5_hl-8.0 \
    -lvtkIOImage-8.0 \
    -lvtkIOMovie-8.0 \
    -lvtkIOVideo-8.0 \
    -lvtkIOXML-8.0 \
    -lvtkjsoncpp-8.0 \
    -lvtkCommonSystem-8.0 \
    -lvtkCommonCore-8.0 \
    -lvtkCommonDataModel-8.0 \
    -lvtkCommonMath-8.0 \
    -lvtkGUISupportQt-8.0 \
    -lvtkRenderingQt-8.0 \
    -lvtkGUISupportQtSQL-8.0 \
    -lvtkRenderingCore-8.0 \
    -lvtkRenderingOpenGL2-8.0 \
    -lvtkIOParallel-8.0 \
    -lvtkParallelCore-8.0 \
    -lvtkCommonExecutionModel-8.0 \
    -lvtkIOGeometry-8.0 \
    -lvtkRenderingAnnotation-8.0 \
    -lvtkFiltersGeometry-8.0 \
    -lvtkFiltersCore-8.0 \
    -lpcl_io \
    -lpcl_ml \
    -lpcl_common \
    -lpcl_io_ply \
    -lpcl_kdtree \
    -lpcl_ml \
    -lpcl_octree \
    -lpcl_people \
    -lpcl_search \
    -lpcl_stereo \
    -lpcl_filters \
    -lpcl_surface \
    -lpcl_features \
    -lpcl_tracking \
    -lpcl_keypoints \
    -lpcl_outofcore \
    -lpcl_recognition \
    -lpcl_registration \
    -lpcl_segmentation \
    -lpcl_visualization \
    -lpcl_sample_consensus \
    -lgomp
# -------------------------------------------------

# armadillo ----------------------------------------
INCLUDEPATH += /usr/include

LIBS += -L/usr/lib/x86_64-linux-gnu \
    -larmadillo
# --------------------------------------------------


