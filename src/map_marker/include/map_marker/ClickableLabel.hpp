
#ifndef CLICKABLELABEL_H_
#define CLICKABLELABEL_H_
#include <QLabel>
#include <QMouseEvent>

using namespace Qt;

class ClickableLabel : public QLabel
{
    Q_OBJECT
public:
    ClickableLabel(QWidget *parent = 0);
    void mousePressEvent(QMouseEvent *eve );
};

#endif /*CLICKABLELABEL_H_*/
