
#include "clickableLabel.h"
#include <QDebug>

ClickableLabel::ClickableLabel(QWidget *parent) : QLabel(parent)
{
}

void ClickableLabel::mousePressEvent(QMouseEvent *eve )
{
    qInfo("Clicked!!");

    const QPoint p = eve->pos();
    qDebug() << "The point: " << p;

}
