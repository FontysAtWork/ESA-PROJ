#ifndef map_marker_MAIN_WINDOW_H
#define map_marker_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <vector>
#include "Marker.hpp"
#include "ClickableLabel.hpp"


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
		void on_btnMoveRobot_clicked();
		void on_btnRemoveMarker_clicked();
		void lblMapImage_clicked(QPoint);
		int GetSelectedMarker();
		void AddMarker(Marker marker);
		void UpdateTable();

	public:
		MainWindow(int argc, char** argv, QWidget *parent = 0);
		~MainWindow();

		//void closeEvent(Qt::QCloseEvent *event); // Overloaded function

	private:
		Ui::MapMarker ui;
		QNode qnode;
		std::vector<Marker> markers;
		ClickableLabel *lblMapImage;
	};
}

#endif // map_marker_MAIN_WINDOW_H
