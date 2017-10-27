#include "../include/map_marker/ClickableLabel.hpp"
#include <QDebug>

using namespace Qt;

ClickableLabel::ClickableLabel(QWidget *parent) : QLabel(parent)
{
}

void ClickableLabel::mousePressEvent(QMouseEvent *eve )
{
    //qInfo("Clicked!!");

    const QPoint p = eve->pos();
    qDebug() << "The point: " << p;

}