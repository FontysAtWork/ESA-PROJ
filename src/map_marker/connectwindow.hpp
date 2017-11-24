#ifndef CONNECTWINDOW_HPP
#define CONNECTWINDOW_HPP

#include <QMainWindow>

namespace Ui {
class ConnectWindow;
}

class ConnectWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ConnectWindow(QWidget *parent = 0);
    ~ConnectWindow();

private:
    Ui::ConnectWindow *ui;
};

#endif // CONNECTWINDOW_HPP
