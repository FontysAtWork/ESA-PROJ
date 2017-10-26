#ifndef MAPMARKER_H
#define MAPMARKER_H

#include <QMainWindow>

namespace Ui {
class MapMarker;
}

class MapMarker : public QMainWindow
{
    Q_OBJECT

public:
    explicit MapMarker(QWidget *parent = 0);
    ~MapMarker();

private slots:
    void on_btnLoadYaml_clicked();

private:
    Ui::MapMarker *ui;
};

#endif // MAPMARKER_H
