#ifndef CSCENEUPDATE_H
#define CSCENEUPDATE_H

// Qt header
#include <QDialog>
#include <QFileDialog>
#include <QImage>
#include <QtGui>
#include <QThread>
#include <QMetaType>
#include <QStandardItemModel>
#include <QProcess>
// opencv header
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// stl
#include <iostream>

using namespace std;

class C_T_SCENEUPDATE : public QThread {
    Q_OBJECT
public:
    C_T_SCENEUPDATE(){}
    ~C_T_SCENEUPDATE(){}

private:
    void run();

private:
    bool pause_status = false;

signals:
    void SIG_C_T_SCENEUPDATE_2_MAIN();
};

#endif // CSCENEUPDATE_H
