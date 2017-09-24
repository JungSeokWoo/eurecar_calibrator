#include "c_custom_scene.h"

void C_CUSTOM_SCENE::mousePressEvent(QGraphicsSceneMouseEvent *event)
{

    uint coord_x = event->scenePos().x();
    uint coord_y = event->scenePos().y();

    scene_pt_struct pt_data;
    pt_data.x = coord_x;
    pt_data.y = coord_y;

    if(pt_list.size() >= 3)
    {
        pt_list.clear();
    }
    else
    {
        pt_list.push_back(pt_data);
    }

    QGraphicsScene::mousePressEvent(event);

    c_t_scene_update->start();

}

void C_CUSTOM_SCENE::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mouseMoveEvent(event);
}

void C_CUSTOM_SCENE::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mouseReleaseEvent(event);
}

void C_CUSTOM_SCENE::mouseWheelEvent(QGraphicsSceneWheelEvent *event)
{
    QGraphicsScene::wheelEvent(event);
}

