#ifndef map_marker_MAIN_WINDOW_H
#define map_marker_MAIN_WINDOW_H

#include "ui_MainWindow.h"
#include "Qnode.hpp"
#include <vector>
#include "Marker.hpp"
#include "ClickableLabel.hpp"
#include "YamlParser.hpp"
#include "MapConfig.hpp"
#include "YamlWriter.hpp"
#include "NoGoLine.hpp"

#include "geometry_msgs/Pose.h"

namespace map_marker {
	class MainWindow : public QMainWindow {
		Q_OBJECT

	public Q_SLOTS:
		void lblMapImage_clicked(QPoint);
		void on_btnLoadYaml_clicked();
		void on_btnLoadMap_clicked();
		void on_btnWriteYaml_clicked();
		void on_btnClearAllMarkers_clicked();
		void on_btnAddCurrentPose_clicked();
		void on_btnAddCustomPose_clicked();
		void on_btnMoveRobot_clicked();
		void on_btnRemoveMarker_clicked();
		void on_btnPanic_clicked();
		void on_btnMoveMarkerDown_clicked();
		void on_btnMoveMarkerUp_clicked();
		void on_btnConnect_clicked();
		void on_btnLoadMarkersYaml_clicked();
		void on_btnUpdateMarker_clicked();
		void on_btnNogoLine_clicked();
		void on_radioNav_clicked();
		void on_radioWorkspace_clicked();
		void on_cbxEnvVars_clicked();
		void UpdateRobotPose();
		void SelectionIsChanged();
		void UpdateRobotSize();
		

	public:
		MainWindow(int argc, char** argv, QWidget *parent = 0);
		~MainWindow();

	private:
		void AddMarker(Marker marker);
		void UpdateMarker(int index, Marker marker);
		void MoveMarkerUp(int selectedMarker);
		void MoveMarkerDown(int selectedMarker);
		void UpdateTable();
		void FillMarkerList(std::vector<KeyDataPair> data);
		void ToggleInterface(bool b);
		void UpdateWindow();
		void EnableInterface();
		void ShowNoMasterMessage();

		QPointF RotateDrawPoint(QPoint center, double x, double y, double angle);
		int GetSelectedMarker();
		int ConvertRealSizeToPixel(double a);
		double ConvertPixelToRealSize(int a);
		geometry_msgs::Pose MakePose(double pX, double pY, double pZ, double qX, double qY, double qZ, double qW);

		Ui::MapMarker ui;
		QNode qnode;
		std::vector<Marker> markers;
		ClickableLabel *lblMapImage;
		YamlParser yaml;
		YamlWriter yamlWriter;
		MapConfig mapConfig;
		//QPixmap *map;
		QImage *map;
		geometry_msgs::Pose robotPose;
		QSize robotSize;
		NoGoLine line;

		double map_min;
		double map_max;
		double map_pix;

		bool YamlLoaded;
		bool ImageLoaded;
		bool NodeStarted;

	protected:
    	void paintEvent(QPaintEvent *event);
    	void drawMarkers(QPainter *qp);
	};
}

#endif // map_marker_MAIN_WINDOW_H
