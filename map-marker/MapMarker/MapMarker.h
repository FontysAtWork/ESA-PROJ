#ifndef MAPMARKER_H
#define MAPMARKER_H

#include <QMainWindow>
#include <QPoint>

namespace Ui {
class MapMarker;
}

class MapMarker : public QMainWindow
{
    Q_OBJECT

public:
    explicit MapMarker(QWidget *parent = 0);
    ~MapMarker();

private:
    Ui::MapMarker *ui;
    QPoint GetCurrentMousePos();
};

#endif // MAPMARKER_H
