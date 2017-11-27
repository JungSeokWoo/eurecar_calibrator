#ifndef CUSTOM_SCENE_IPM_H
#define CUSTOM_SCENE_IPM_H

#include <QWidget>
#include <QDialog>
#include <QFileDialog>
#include <QImage>
#include <QtGui>
#include <QThread>
#include <QPixmap>

#include <QMetaType>

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneMoveEvent>
#include <QGraphicsSceneWheelEvent>
#include <QGraphicsLineItem>

#include <iostream>
#include <vector>

#include "thread/c_sceneupdate.h"


#ifndef SCENE_PT_STRUCT_DEF
#define SCENE_PT_STRUCT_DEF
struct scene_pt_struct{
    int x;
    int y;
};
#endif

using namespace std;


class C_CUSTOM_SCENE_IPM : public QGraphicsScene
{

public:
    C_CUSTOM_SCENE_IPM(C_T_SCENEUPDATE *_c_t_scene_update)
    {
        c_t_scene_update = _c_t_scene_update;
    }

    C_T_SCENEUPDATE* c_t_scene_update;

    int mouse_click_x_loc = 0;
    int mouse_click_y_loc = 0;

    int mouse_start_x_loc = 0;
    int mouse_start_y_loc = 0;
    int mouse_finish_x_loc = 0;
    int mouse_finish_y_loc = 0;

    int mouse_x_loc;
    int mouse_y_loc;

    int scroll_val = 0;

    int current_ind = 0;

    vector<scene_pt_struct> pt_list;


protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseWheelEvent(QGraphicsSceneWheelEvent *event);
private:
    QPointF origPoint;
    QGraphicsLineItem* itemToDraw;
    void makeItemsControllable(bool areControllable);

};



#endif // CUSTOM_SCENE_IPM_H
