#ifndef map_marker_MAIN_WINDOW_H
#define map_marker_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace map_marker {
	class MainWindow : public QMainWindow {
		Q_OBJECT

	public Q_SLOTS:
		void on_btnLoadYaml_clicked();
		void on_btnLoadMap_clicked();
		void on_btnWriteYaml_clicked();
		void on_btnClearYaml_clicked();
		void on_btnAddCurrentPose_clicked();
		void on_btnAddCustomPose_clicked();

	public:
		MainWindow(int argc, char** argv, QWidget *parent = 0);
		~MainWindow();
		//void closeEvent(Qt::QCloseEvent *event); // Overloaded function

	private:
		Ui::MapMarker ui;
		QNode qnode;
	};
}

#endif // map_marker_MAIN_WINDOW_H
