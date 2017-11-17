#ifndef map_marker_MAIN_WINDOW_H
#define map_marker_MAIN_WINDOW_H

#include "ui_main_window.h"
#include "qnode.hpp"
#include <vector>
#include "Marker.hpp"
#include "ClickableLabel.hpp"
#include "YamlParser.hpp"
#include "MapConfig.hpp"

#include "geometry_msgs/Pose.h"

namespace map_marker {
	class MainWindow : public QMainWindow {
		Q_OBJECT

	public Q_SLOTS:
		void lblMapImage_clicked(QPoint);
		void on_btnLoadYaml_clicked();
		void on_btnLoadMap_clicked();
		void on_btnWriteYaml_clicked();
		void on_btnClearYaml_clicked();
		void on_btnAddCurrentPose_clicked();
		void on_btnAddCustomPose_clicked();
		void on_btnMoveRobot_clicked();
		void on_btnRemoveMarker_clicked();
		void on_btnPanic_clicked();
		void on_btnMoveMarkerDown_clicked();
		void on_btnMoveMarkerUp_clicked();
		void on_btnClearAllMarkers_clicked();
		void UpdateRobotPose(geometry_msgs::Pose p);
		

	public:
		MainWindow(int argc, char** argv, QWidget *parent = 0);
		~MainWindow();

		//void closeEvent(Qt::QCloseEvent *event); // Overloaded function

	private:
		void AddMarker(Marker marker);
		void MoveMarkerUp(int selectedMarker);
		void MoveMarkerDown(int selectedMarker);
		void UpdateTable();
		void UpdateWindow();
		int GetSelectedMarker();
		int ConvertRobotToPixel(double a);
		double ConvertPixelToRobot(int a);
		Ui::MapMarker ui;
		QNode qnode;
		std::vector<Marker> markers;
		ClickableLabel *lblMapImage;
		YamlParser yaml;
		MapConfig mapConfig;
		QPixmap *map;
		geometry_msgs::Pose robotPose;

	protected:
    	void paintEvent(QPaintEvent *event);
    	void drawMarkers(QPainter *qp);
	};
}

#endif // map_marker_MAIN_WINDOW_H
