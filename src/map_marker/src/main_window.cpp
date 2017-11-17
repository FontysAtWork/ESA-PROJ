#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <QGraphicsScene>
#include <iostream>
#include <algorithm>
#include "main_window.hpp"

#include <QDebug>

#include "geometry_msgs/Pose.h"

using namespace Qt;

namespace map_marker {

	const double map_min = -5;
	const double map_max = 5;
	const double map_pix = 992;

	const QColor red = QColor(180, 40, 0);
	const QColor blue = QColor(30, 30, 140);
	const QColor green = QColor(50, 140, 30);
	const QColor orange = QColor(230, 120, 0);

	


	MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc,argv) {
		ui.setupUi(this);
		qnode.init();

		// Connect list update to draw function
		QObject::connect(ui.tableWidget->selectionModel(), SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)), SLOT(DrawOnImage()));

		// Load map image
		QString url = "/home/lars/git/ESA-PROJ/maps/legomap3-cropped.pgm";
		map = new QPixmap(url);

		// Ttable editing
		ui.tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
		ui.tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
		ui.tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);

		// Create map image
		lblMapImage = new ClickableLabel(this);
		lblMapImage->setAlignment(Qt::AlignBottom | Qt::AlignRight);
		lblMapImage->setGeometry(QRect(0, 0, map_pix, map_pix));
		lblMapImage->setPixmap(*map);
		QObject::connect(lblMapImage, SIGNAL(clicked(QPoint)), this, SLOT(lblMapImage_clicked(QPoint)));

		// Set validator for input fields
		ui.inpCustomX->setValidator(new QDoubleValidator(-100, 100, 5, ui.inpCustomX));
		ui.inpCustomY->setValidator(new QDoubleValidator(-100, 100, 5, ui.inpCustomY));
		ui.inpCustomAngle->setValidator(new QDoubleValidator(0, 360, 5, ui.inpCustomAngle));

		// Panic button color
		ui.btnPanic->setStyleSheet("color: rgb(192,0,0);");

		// Add marker for testing
		Marker m(1.0, 2.0, 40.0, Navigation);
		AddMarker(m);

		UpdateTable();
	}

	MainWindow::~MainWindow() {
		delete map;
		delete lblMapImage;

	}

	void MainWindow::lblMapImage_clicked(QPoint a) {
		QString x = QString::number(ConvertPixelToRobot(a.x()));
		QString y = QString::number(ConvertPixelToRobot(a.y()));
		ui.inpCustomX->setText(x);
		ui.inpCustomY->setText(y);
	}

	void MainWindow::on_btnLoadYaml_clicked() {
		QFileDialog dialog(this);
		dialog.setFileMode(QFileDialog::AnyFile);
		dialog.setNameFilter(tr("Map image file (*.yaml)"));

		QStringList fileNames;
		if (dialog.exec())
		fileNames = dialog.selectedFiles();

		yaml.loadYaml(fileNames[0].toUtf8().constData());
	}

	void MainWindow::on_btnLoadMap_clicked() {
		QFileDialog dialog(this);
		dialog.setFileMode(QFileDialog::AnyFile);
		dialog.setNameFilter(tr("Map image file (*.pbm *.pgm *.ppm)"));

		QStringList fileNames;
		if (dialog.exec())
		fileNames = dialog.selectedFiles();

		map = new QPixmap(fileNames[0]);
			lblMapImage->setPixmap(*map);
	}

	void MainWindow::on_btnWriteYaml_clicked() {
		ROS_ERROR("Not implemented yet :(");
	}

	void MainWindow::on_btnClearYaml_clicked() {
		ROS_ERROR("Not implemented yet :(");
	}

	void MainWindow::on_btnAddCurrentPose_clicked() {
		geometry_msgs::Pose pos = qnode.GetRobotPosition();

		MarkerType type;
		if(ui.radioNav->isChecked()) {
			type = Navigation;
		} else if (ui.radioWorkspace->isChecked()) {
			type = Workspace;
		}

		AddMarker(Marker(pos, type));
		UpdateTable();
	}

	void MainWindow::on_btnAddCustomPose_clicked() {
		double x = ui.inpCustomX->text().toDouble();
		double y = ui.inpCustomY->text().toDouble();
		double angle = ui.inpCustomAngle->text().toDouble();
		
		MarkerType type;
		if(ui.radioNav->isChecked()) {
			type = Navigation;
		} else if (ui.radioWorkspace->isChecked()) {
			type = Workspace;
		}
		
		AddMarker(Marker(x, y, angle, type));
		UpdateTable();
	}

	void MainWindow::on_btnMoveRobot_clicked() {
		int index = GetSelectedMarker();
		if(index == -1) {
			ROS_WARN("No marker selected");
			return;
		} 
		Marker * m = &markers[index];
		qnode.MoveRobotToPose(m->GetPose());
	}

	void MainWindow::on_btnRemoveMarker_clicked() {
		int index = GetSelectedMarker();
		if(index == -1) {
			ROS_WARN("No marker selected");
			return;
		} 
		markers.erase(markers.begin()+index);
		UpdateTable();
	}

	void MainWindow::on_btnPanic_clicked() {
		qnode.Panic();
		ROS_INFO("Panic button pressed, stopped robot...");
	}

	void MainWindow::on_btnMoveMarkerUp_clicked() {
		int selectedMarker = GetSelectedMarker();
		MoveMarkerUp(selectedMarker);
		UpdateTable();
	}

	void MainWindow::on_btnMoveMarkerDown_clicked() {
		int selectedMarker = GetSelectedMarker();
		MoveMarkerDown(selectedMarker);
		UpdateTable();
	}

	void MainWindow::on_btnClearAllMarkers_clicked() {
		markers.clear();
		UpdateTable();
	}

	int MainWindow::GetSelectedMarker() {
		int j = -1;
		QModelIndexList indexes = ui.tableWidget->selectionModel()->selectedRows();

		for (int i = 0; i < indexes.count(); ++i) {    
			j = indexes.at(i).row();
		}

		return j;
	}

	void MainWindow::AddMarker(Marker marker) {
		markers.push_back(marker);
	}

	void MainWindow::MoveMarkerUp(int selectedMarker) {
		if(selectedMarker + 1 < markers.size() && selectedMarker >= 0) {
			std::swap(markers.at(selectedMarker), markers.at(selectedMarker + 1));
		} else {
			ROS_WARN("No marker selected");
		}
	}

	void MainWindow::MoveMarkerDown(int selectedMarker) {
		if(selectedMarker > 0) {
			std::swap(markers.at(selectedMarker), markers.at(selectedMarker - 1));
		} else {
			ROS_WARN("No marker selected");
		}
	}

	void MainWindow::UpdateTable() {
		ui.tableWidget->setRowCount(0);
		for(int i=0; i < markers.size(); i++) {
			ui.tableWidget->insertRow ( ui.tableWidget->rowCount() );
			ui.tableWidget->setItem(ui.tableWidget->rowCount()-1, 0, new QTableWidgetItem(QString::fromStdString(markers[i].GetTypeStr())));
			ui.tableWidget->setItem(ui.tableWidget->rowCount()-1, 1, new QTableWidgetItem(QString::number(markers[i].GetX())));
			ui.tableWidget->setItem(ui.tableWidget->rowCount()-1, 2, new QTableWidgetItem(QString::number(markers[i].GetY())));
			ui.tableWidget->setItem(ui.tableWidget->rowCount()-1, 3, new QTableWidgetItem(QString::number(markers[i].GetAngle())));
		}
		DrawOnImage();
		
	}

	void MainWindow::DrawOnImage() {
		lblMapImage->setPixmap(*map);
		DrawRobotOnImage();
		DrawMarkersOnImage();
	}

	void MainWindow::DrawRobotOnImage() {
		geometry_msgs::Pose pos = qnode.GetRobotPosition();

		QPoint p1;
		p1.setX(ConvertRobotToPixel(pos.position.x));
		p1.setY(ConvertRobotToPixel(pos.position.y));

		QImage tmp(lblMapImage->pixmap()->toImage());
		QPainter painter(&tmp);
		QPen paintpen(blue);

		paintpen.setWidth(10);
		painter.setPen(paintpen);
		painter.drawPoint(p1);
		painter.end();

		lblMapImage->setPixmap(QPixmap::fromImage(tmp));
	}

	void MainWindow::DrawMarkersOnImage() {	
		QPoint p1;

		QImage tmp(lblMapImage->pixmap()->toImage());
		QPainter painter(&tmp);
		QPen paintpen(orange);
		
		paintpen.setWidth(10);

		for(int i; i < markers.size(); i++) {
			
			if(i == GetSelectedMarker()) {
				paintpen.setColor(red);
			} else if(markers[i].GetType() == Workspace) {
				paintpen.setColor(green);
			} else {
				paintpen.setColor(orange);
			}
			
			p1.setX(ConvertRobotToPixel(markers[i].GetX()));
			p1.setY(ConvertRobotToPixel(markers[i].GetY()));

			painter.setPen(paintpen);
			painter.drawPoint(p1);
		}

		painter.end();

		lblMapImage->setPixmap(QPixmap::fromImage(tmp));
	}

	int MainWindow::ConvertRobotToPixel(double a) {
		return (a - map_min) * (map_pix - 0) / (map_max - map_min);
	}

	double MainWindow::ConvertPixelToRobot(int a) {
		return (a) * (map_max - map_min) / (map_pix) + map_min;
	}
}
