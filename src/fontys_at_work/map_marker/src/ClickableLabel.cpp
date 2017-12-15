#include "ClickableLabel.hpp"

ClickableLabel::ClickableLabel(QWidget* parent, Qt::WindowFlags f)
    : QLabel(parent) {
    
}

ClickableLabel::~ClickableLabel() {}

void ClickableLabel::mousePressEvent(QMouseEvent* event) {

	const QPoint p = event->pos();
	//p = event->pos();
    Q_EMIT ClickableLabel::clicked(p);
}
