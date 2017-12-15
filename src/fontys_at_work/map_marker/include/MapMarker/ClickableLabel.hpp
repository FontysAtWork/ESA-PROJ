
#ifndef CLICKABLELABEL_H
#define CLICKABLELABEL_H

#include <QLabel>
#include <QWidget>
#include <Qt>
#include <QObject>
#include <QMouseEvent>

class ClickableLabel : public QLabel { 
    Q_OBJECT 

public:
    explicit ClickableLabel(QWidget* parent = 0, Qt::WindowFlags f = Qt::WindowFlags());
    ~ClickableLabel();

Q_SIGNALS:
	void clicked(QPoint);

protected:
    void mousePressEvent(QMouseEvent *ev);

};

#endif // CLICKABLELABEL_H
