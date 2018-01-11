#ifndef map_marker_MAIN_WINDOW_H
#define map_marker_MAIN_WINDOW_H

#include "ui_MainWindow.h"
#include "Qnode.hpp"
#include <vector>
#include "nav_lib/Marker.hpp"
#include "ClickableLabel.hpp"
#include "nav_lib/YamlParser.hpp"
#include "nav_lib/MapConfig.hpp"
#include "nav_lib/YamlWriter.hpp"
#include "nav_lib/NoGoLine.hpp"
#include "MapRenderer.hpp"
#include <QTimer>
#include "nav_lib/Nav.hpp"

#include "geometry_msgs/Pose.h"

namespace map_marker {
	class MainWindow : public QMainWindow {
		Q_OBJECT

	public Q_SLOTS:
		void lblMapImage_clicked(QPoint);
		void on_btnLoadYaml_clicked();
		void on_btnLoadMap_clicked();
		void on_btnWriteMarkersYaml_clicked();
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
		void on_btnRemoveLine_clicked();
		void on_btnUpdateLine_clicked();
		void on_btnMoveLineUp_clicked();
		void on_btnMoveLineDown_clicked();
		void on_btnLoadLinesYaml_clicked();
		void on_btnWriteLinesYaml_clicked();
		void on_btnNogoLine_clicked();
		void on_btnClearLine_clicked();
		void on_radioShelf_clicked();
		void on_radioWorkstation_clicked();
		void on_radioConveyor_clicked();
		void on_radioWaypoint_clicked();
		void on_radioPrecision_clicked();
		void on_radioNoGoOne_clicked();
		void on_radioNoGoTwo_clicked();
		void on_cbxDynamicImage_clicked();
		void on_cbxEnvVars_clicked();
		void UpdateRobotPose();
		void SelectionIsChanged();
		void UpdateRobotSize();
		void UpdateMap();
		

	public:
		MainWindow(int argc, char** argv, QWidget *parent = 0);
		~MainWindow();

	private:
		void AddMarker(Marker marker);
		void UpdateMarker(int index, Marker marker);
		void MoveMarkerUp(int selectedMarker);
		void MoveMarkerDown(int selectedMarker);
		void UpdateMarkerTable();
		void AddLine(NoGoLine line);
		void UpdateLine(int index, NoGoLine line);
		void MoveLineUp(int selectedLine);
		void MoveLineDown(int selectedLine);
		void UpdateLineTable();
		void FillMarkerList(std::vector<KeyDataPair> data);
		void FillLineList(std::vector<KeyDataPair> data);
		void ToggleInterface(bool b);
		void UpdateWindow();
		void EnableInterface();
		void ShowNoMasterMessage();

		QPointF RotateDrawPoint(QPoint center, double x, double y, double angle);
		int GetSelectedMarker();
		int GetSelectedLine();
		void ResendAllLines();
		int ConvertRealSizeToPixel(double a);
		double ConvertPixelToRealSize(int a);
		geometry_msgs::Pose MakePose(double pX, double pY, double pZ, double qX, double qY, double qZ, double qW);
		void DrawLine(NoGoLine line);
		QPoint rotatePixel(int x, int y);

		Ui::MapMarker ui;
		QNode qnode;
		std::vector<Marker> markers;
		std::vector<NoGoLine> lines;
		ClickableLabel *lblMapImage;
		YamlParser yaml;
		YamlWriter yamlWriter;
		MapConfig mapConfig;
		QImage *map;
		geometry_msgs::Pose robotPose;
		QSize robotSize;
		MapRenderer mapRenderer;
		QTimer timerForMap;

		double map_min;
		double map_max;
		double map_pix;

		bool YamlLoaded;
		bool ImageLoaded;
		bool NodeStarted;
		bool DynamicImage;
		bool NoGoOneSelected;

	protected:
    	void paintEvent(QPaintEvent *event);
    	void drawMarkers(QPainter *qp);
	};
}

#endif // map_marker_MAIN_WINDOW_H
