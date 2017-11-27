#include "c_custom_scene_ipm.h".h"

void C_CUSTOM_SCENE_IPM::mousePressEvent(QGraphicsSceneMouseEvent *event)
{

    uint coord_x = event->scenePos().x();
    uint coord_y = event->scenePos().y();

    scene_pt_struct pt_data;
    pt_data.x = coord_x;
    pt_data.y = coord_y;

    if(pt_list.size() >= 4)
    {
        pt_list.clear();
    }
    else
    {
        pt_list.push_back(pt_data);
    }

    QGraphicsScene::mousePressEvent(event);

    emit c_t_scene_update->SIG_C_T_SCENEUPDATE_2_MAIN_IPM();
}

void C_CUSTOM_SCENE_IPM::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mouseMoveEvent(event);
}

void C_CUSTOM_SCENE_IPM::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mouseReleaseEvent(event);
}

void C_CUSTOM_SCENE_IPM::mouseWheelEvent(QGraphicsSceneWheelEvent *event)
{
    QGraphicsScene::wheelEvent(event);
}

