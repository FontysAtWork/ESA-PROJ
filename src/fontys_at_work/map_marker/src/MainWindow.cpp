#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <QGraphicsScene>
#include <iostream>
#include <algorithm>
#include "MainWindow.hpp"
#include <tf/transform_datatypes.h>

#include <QDebug>

#define DEBUG false

using namespace Qt;

namespace map_marker {
	const QColor red = QColor(180, 40, 0);
	const QColor blue = QColor(30, 30, 140);
	const QColor green = QColor(50, 140, 30);
	const QColor orange = QColor(230, 120, 0);	
	const QColor pink = QColor(255, 105, 180);
	const QColor purple = QColor(188, 66, 244);
	const QColor lightblue = QColor(65, 184, 244);	

	MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc,argv) {
		ui.setupUi(this);
		
		// Make a pose to avoid warnings if to pose is not yet retreived from the robot
		robotPose = MakePose(-4.5, 4.5, 0.0, 0.0, 0.0, 0.0, 1.0);

		// Connect list update to draw function
		QObject::connect(ui.tableMarkers->selectionModel(), SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)), this, SLOT(SelectionIsChanged()));
		QObject::connect(ui.spinRobotWidth, SIGNAL(valueChanged(int)), this, SLOT(UpdateRobotSize()));
		QObject::connect(ui.spinRobotHeight, SIGNAL(valueChanged(int)), this, SLOT(UpdateRobotSize()));
		QObject::connect(&qnode, SIGNAL(RobotPosUpdated()), this, SLOT(UpdateRobotPose()));
		QObject::connect(&qnode, SIGNAL(RosShutdown()), QApplication::instance(), SLOT(quit()));

		// Set robot size
		UpdateRobotSize();

		// Bools to enable gui features
		YamlLoaded = false;
		ImageLoaded = false;
		NodeStarted = false;

		if(DEBUG)
		{
			// This gets overwritten when loading yaml or a map. 
			map_min = -5;
			map_max = 5;
			map_pix = 992;
			// Load map image
			QString url = "/home/lars/git/ESA-PROJ/maps/legomap3-cropped.pgm";
			map = new QPixmap(url);
			lblMapImage->setGeometry(QRect(0, 0, map_pix, map_pix));

			ToggleInterface(true);

		}
		else
		{
			map = new QPixmap();
		}

		ui.cbxImgMapPgm->setAttribute(Qt::WA_TransparentForMouseEvents);
		ui.cbxImgMapPgm->setFocusPolicy(Qt::NoFocus);

		ui.cbxImgMapYaml->setAttribute(Qt::WA_TransparentForMouseEvents);
		ui.cbxImgMapYaml->setFocusPolicy(Qt::NoFocus);

		// Ttable editing
		ui.tableMarkers->setEditTriggers(QAbstractItemView::NoEditTriggers);
		ui.tableMarkers->setSelectionBehavior(QAbstractItemView::SelectRows);
		ui.tableMarkers->setSelectionMode(QAbstractItemView::SingleSelection);

		// Create map image
		lblMapImage = new ClickableLabel(this);
		lblMapImage->setAlignment(Qt::AlignBottom | Qt::AlignRight);
		
		QObject::connect(lblMapImage, SIGNAL(clicked(QPoint)), this, SLOT(lblMapImage_clicked(QPoint)));

		// Set validator for input fields
		ui.inpCustomX->setValidator(new QDoubleValidator(-100, 100, 5, ui.inpCustomX));
		ui.inpCustomY->setValidator(new QDoubleValidator(-100, 100, 5, ui.inpCustomY));
		ui.inpCustomAngle->setValidator(new QDoubleValidator(0, 360, 5, ui.inpCustomAngle));

		// Panic button color
		ui.btnPanic->setStyleSheet("color: rgb(192,0,0);");

		UpdateTable();
	}

	MainWindow::~MainWindow() {
		delete map;
		delete lblMapImage;
	}

	void MainWindow::paintEvent(QPaintEvent *e) {
	  Q_UNUSED(e);
	  
	  QPainter qp(this);

	  qp.drawPixmap(0, 0, map_pix, map_pix, *map);
	  drawMarkers(&qp);
	}

	QPointF MainWindow::RotateDrawPoint(QPoint center, double x, double y, double angle) {
		double tempX = x - center.x();
		double tempY = y - center.y();

		double rotatedX = tempX * cos(angle) - tempY * sin(angle);
		double rotatedY = tempX * sin(angle) + tempY * cos(angle);

		return QPointF(rotatedX + center.x(), rotatedY + center.y());
	}

	void MainWindow::drawMarkers(QPainter *qp) {
		QPen pen(Qt::black, 2, Qt::SolidLine);  
		//geometry_msgs::Pose pos = qnode.GetRobotPosition();
		QPoint p1;
		int selected = GetSelectedMarker();

		for(int i = 0; i < markers.size(); i++) {

			p1.setX(ConvertRealSizeToPixel(markers[i].GetX()));
			p1.setY(ConvertRealSizeToPixel(-markers[i].GetY()));
			
			if(i == selected) {
				pen.setWidth(3);
				pen.setColor(red);
		
				qp->setPen(pen);

				tf::Quaternion q1;	
				tf::quaternionMsgToTF(markers[i].GetQuaternation(), q1);
				q1.normalize();
				double angle1 = tf::getYaw(q1);

				QPointF points[4] = {
					RotateDrawPoint(p1, p1.x() - robotSize.width() / 2, p1.y() + robotSize.height() / 2, -angle1),
					RotateDrawPoint(p1, p1.x() + robotSize.width() / 2, p1.y() + robotSize.height() / 2, -angle1),
					RotateDrawPoint(p1, p1.x() + robotSize.width() / 2, p1.y() - robotSize.height() / 2, -angle1),			
					RotateDrawPoint(p1, p1.x() - robotSize.width() / 2, p1.y() - robotSize.height() / 2, -angle1)
				};

				qp->drawPolygon(points, 4);	

				QPointF centerFront = RotateDrawPoint(p1, p1.x() + robotSize.width() / 2, p1.y(), -angle1);
				QPointF centerRight = RotateDrawPoint(p1, p1.x() + robotSize.width() / 4, p1.y() - robotSize.height() / 2, -angle1);
				QPointF centerLeft = RotateDrawPoint(p1, p1.x() + robotSize.width() / 4, p1.y() + robotSize.height() / 2, -angle1);

				pen.setWidth(2);
				qp->setPen(pen);

				qp->drawLine(centerFront, centerRight);
				qp->drawLine(centerFront, centerLeft);
				
			} else if(markers[i].GetType() == Shelf) {
				pen.setColor(green);
			} else if(markers[i].GetType() == Workstation) {
				pen.setColor(lightblue);
			} else if(markers[i].GetType() == Conveyor) {
				pen.setColor(pink);
			} else if(markers[i].GetType() == Waypoint) {
				pen.setColor(orange);
			} else if(markers[i].GetType() == Precision) {
				pen.setColor(purple);
			} else {
				pen.setColor(blue);
			}

			pen.setWidth(10);

			qp->setPen(pen);
			qp->drawPoint(p1);

			
		}

		p1.setX(ConvertRealSizeToPixel(robotPose.position.x));
		p1.setY(ConvertRealSizeToPixel(-robotPose.position.y));

		pen.setWidth(10);
		pen.setColor(blue);
		qp->setPen(pen);
		qp->drawPoint(p1);

		pen.setWidth(3);
		qp->setPen(pen);
		//qp->fillRect(p1.x() - robotSize.width() / 2, p1.y() - robotSize.height() / 2, robotSize.width(), robotSize.height(), blue);

		tf::Quaternion q;	
		tf::quaternionMsgToTF(robotPose.orientation, q);
		q.normalize();
		double angle = tf::getYaw(q);

		QPointF points[4] = {
			RotateDrawPoint(p1, p1.x() - robotSize.width() / 2, p1.y() + robotSize.height() / 2, -angle),
			RotateDrawPoint(p1, p1.x() + robotSize.width() / 2, p1.y() + robotSize.height() / 2, -angle),
			RotateDrawPoint(p1, p1.x() + robotSize.width() / 2, p1.y() - robotSize.height() / 2, -angle),			
			RotateDrawPoint(p1, p1.x() - robotSize.width() / 2, p1.y() - robotSize.height() / 2, -angle)
		};

		qp->drawPolygon(points, 4);	

		QPointF centerFront = RotateDrawPoint(p1, p1.x() + robotSize.width() / 2, p1.y(), -angle);
		QPointF centerRight = RotateDrawPoint(p1, p1.x() + robotSize.width() / 4, p1.y() - robotSize.height() / 2, -angle);
		QPointF centerLeft = RotateDrawPoint(p1, p1.x() + robotSize.width() / 4, p1.y() + robotSize.height() / 2, -angle);

		pen.setWidth(2);
		qp->setPen(pen);

		qp->drawLine(centerFront, centerRight);
		qp->drawLine(centerFront, centerLeft);
	}

	void MainWindow::lblMapImage_clicked(QPoint a) {
		QString x = QString::number(ConvertPixelToRealSize(a.x()));
		QString y = QString::number(-ConvertPixelToRealSize(a.y()));
		
		line.SetPoint(a.x(), a.y(), qnode);

		ui.inpCustomX->setText(x);
		ui.inpCustomY->setText(y);
	}

	void MainWindow::on_btnLoadYaml_clicked() {

		QFileDialog dialog(this);
		dialog.setFileMode(QFileDialog::ExistingFile);
		dialog.setNameFilter(tr("Map yaml file (*.yaml)"));

		QStringList fileNames;
		if (dialog.exec()) {
			fileNames = dialog.selectedFiles();
			yaml.loadYaml(fileNames[0].toUtf8().constData());
			yaml.printLoadedYaml();
			mapConfig.setFullConfigData(yaml.GetParsedYaml());
			map_min = mapConfig.getOrigin().position.x;
			map_max = fabs(map_min);
			YamlLoaded = true;
			ui.cbxImgMapYaml->setChecked(1);
			EnableInterface();
		} else {
			ROS_ERROR("No file selected, nothing loaded");
		}
	}

	void MainWindow::on_btnLoadMap_clicked() {
		QFileDialog dialog(this);
		dialog.setFileMode(QFileDialog::ExistingFile);
		dialog.setNameFilter(tr("Map image file (*.pbm *.pgm *.ppm)"));

		QStringList fileNames;
		if (dialog.exec()) {
			fileNames = dialog.selectedFiles();
			map = new QPixmap(fileNames[0]);
			QSize a = map->size();
			map_pix = a.height();
			UpdateWindow();
			ImageLoaded = true;
			ui.cbxImgMapPgm->setChecked(1);
			EnableInterface();
		} else {
			ROS_ERROR("No file selected, nothing loaded");
		}
	}

	void MainWindow::on_btnWriteMarkersYaml_clicked() {
		QFileDialog dialog(this);
		dialog.setFileMode(QFileDialog::AnyFile);
		dialog.setNameFilter(tr("Nav marker file (*.yaml)"));
		dialog.setDefaultSuffix(tr("yaml"));

		QStringList fileNames;
		if (dialog.exec()) {
			fileNames = dialog.selectedFiles();
			yamlWriter.writeAllMarkers(markers, fileNames[0].toUtf8().constData());
		} else {
			ROS_ERROR("No file selected, nothing loaded");
		}
		
	}

	void MainWindow::on_btnLoadMarkersYaml_clicked() {
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("Navigation yaml file (*.yaml)"));

	QStringList fileNames;
	if (dialog.exec()) {
		fileNames = dialog.selectedFiles();
		yaml.loadYaml(fileNames[0].toUtf8().constData());
		FillMarkerList(yaml.GetParsedYaml());


	} else {
		ROS_ERROR("No file selected, nothing loaded");
	}
	}

	void MainWindow::on_btnClearAllMarkers_clicked() {
		markers.clear();
		UpdateTable();
	}

	void MainWindow::on_btnAddCurrentPose_clicked() {
		geometry_msgs::Pose pos = qnode.GetRobotPosition();
		std::string name = ui.inpCustomName->text().toUtf8().constData();;

		MarkerType type;
		if(ui.radioShelf->isChecked()) {
			type = Shelf;
		} else if (ui.radioWorkstation->isChecked()) {
			type = Workstation;
		} else if (ui.radioConveyor->isChecked()) {
			type = Conveyor;
		} else if (ui.radioWaypoint->isChecked()) {
			type = Waypoint;
		} else if (ui.radioPrecision->isChecked()) {
			type = Precision;
		}

		AddMarker(Marker(pos, type, name));
		UpdateTable();
	}

	void MainWindow::on_btnAddCustomPose_clicked() {
		double x = ui.inpCustomX->text().toDouble();
		double y = ui.inpCustomY->text().toDouble();
		double angle = ui.inpCustomAngle->text().toDouble();
		std::string name = ui.inpCustomName->text().toUtf8().constData();;
		
		MarkerType type;
		if(ui.radioShelf->isChecked()) {
			type = Shelf;
		} else if (ui.radioWorkstation->isChecked()) {
			type = Workstation;
		} else if (ui.radioConveyor->isChecked()) {
			type = Conveyor;
		} else if (ui.radioWaypoint->isChecked()) {
			type = Waypoint;
		} else if (ui.radioPrecision->isChecked()) {
			type = Precision;
		}
		
		AddMarker(Marker(x, y, angle, type, name));
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

	void MainWindow::on_btnUpdateMarker_clicked() {
		int s = GetSelectedMarker();

		if(s == -1) {
			ROS_WARN("No marker selected");
			return;
		} 

		double x = ui.inpCustomX->text().toDouble();
		double y = ui.inpCustomY->text().toDouble();
		double angle = ui.inpCustomAngle->text().toDouble();
		std::string name = ui.inpCustomName->text().toUtf8().constData();;
		
		MarkerType type;
		if(ui.radioShelf->isChecked()) {
			type = Shelf;
		} else if (ui.radioWorkstation->isChecked()) {
			type = Workstation;
		} else if (ui.radioConveyor->isChecked()) {
			type = Conveyor;
		} else if (ui.radioWaypoint->isChecked()) {
			type = Waypoint;
		} else if (ui.radioPrecision->isChecked()) {
			type = Precision;
		}
		
		UpdateMarker(s, Marker(x, y, angle, type, name));
		UpdateTable();
	}

	void MainWindow::on_btnConnect_clicked() {
		if(ui.cbxEnvVars->isChecked()) {
			if (!qnode.Init()) {
				ShowNoMasterMessage();
			} else {
				ui.btnConnect->setEnabled(false);
				ui.inpMaster->setEnabled(false);
				ui.inpHost->setEnabled(false);
				ui.cbxEnvVars->setEnabled(false);
				NodeStarted = true;
			}
		} else {
			if (!qnode.Init(ui.inpMaster->text().toStdString(), 
					ui.inpHost->text().toStdString())) {
					ShowNoMasterMessage();
			} else {
				ui.btnConnect->setEnabled(false);
				ui.inpMaster->setEnabled(false);
				ui.inpHost->setEnabled(false);
				ui.cbxEnvVars->setEnabled(false);
				NodeStarted = true;
			}
		}
		EnableInterface();
	}

	void MainWindow::on_btnNogoLine_clicked() {

		line.ButtonClicked();
	}


	void MainWindow::on_radioShelf_clicked() {
		ui.inpCustomName->setText("SH");
	}

	void MainWindow::on_radioWorkstation_clicked() {
		ui.inpCustomName->setText("WS");
	}

	void MainWindow::on_radioConveyor_clicked() {
		ui.inpCustomName->setText("CB");
	}

	void MainWindow::on_radioWaypoint_clicked() {
		ui.inpCustomName->setText("WP");
	}

	void MainWindow::on_radioPrecision_clicked() {
		ui.inpCustomName->setText("PP");
	}

	void MainWindow::on_cbxEnvVars_clicked() {
		ui.inpMaster->setEnabled(!ui.cbxEnvVars->isChecked());
		ui.inpHost->setEnabled(!ui.cbxEnvVars->isChecked());
		ui.lblMaster->setEnabled(!ui.cbxEnvVars->isChecked());
		ui.lblHost->setEnabled(!ui.cbxEnvVars->isChecked());
	}

	void MainWindow::UpdateRobotSize() {
		robotSize.setHeight(ui.spinRobotHeight->value());
		robotSize.setWidth(ui.spinRobotWidth->value());
		UpdateWindow();
	}

	void MainWindow::FillMarkerList(std::vector<KeyDataPair> data)
	{
		markers.clear();
		geometry_msgs::Pose p;
		MarkerType t;
		std::string name; 
		for (int i = 0; i < data.size(); i++)
		{
			
			if(data[i].key.compare(0,6,"Marker") == 0)
			{
				if(data[i].data[0].compare(0,3,"She") == 0)
				{
					t = Shelf;
				}
				else if(data[i].data[0].compare(0,3,"Wor") == 0)
				{
					t = Workstation;
				}
				else if(data[i].data[0].compare(0,3,"Con") == 0)
				{
					t = Conveyor;
				}
				else if(data[i].data[0].compare(0,3,"Way") == 0)
				{
					t = Waypoint;
				}
				else if(data[i].data[0].compare(0,3,"Pre") == 0)
				{
					t = Precision;
				}
				else
				{
					std::cout << data[i].data[0] << std::endl;
					t = Robot;
				}
			}
			else if (data[i].key.compare("position_x") == 0)
			{
				p.position.x = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("position_y") == 0)
			{
				p.position.y = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("position_z") == 0)
			{
				p.position.z = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_x") == 0)
			{
				p.orientation.x = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_y") == 0)
			{
				p.orientation.y = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_z") == 0)
			{
				p.orientation.z = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("orientation_w") == 0)
			{
				p.orientation.w = std::atof(data[i].data[0].c_str());
			}
			else if (data[i].key.compare("Name") == 0)
			{
				name = data[i].data[0];
			}
			else
			{
				ROS_ERROR("Unkown data!");
			}

			if(i == data.size() -1 || i < data.size() - 1 && data[i+1].key.compare(0,6,"Marker") == 0)
			{
				Marker m(p, t, name);
				markers.push_back(m);
				t = Robot;
				p.position.x = 0;
				p.position.y = 0;
				p.position.z = 0;
				p.orientation.x = 0;
				p.orientation.y = 0;
				p.orientation.z = 0;
				p.orientation.w = 0;
				name = "";
			}
		}
		UpdateTable();
	}

	void MainWindow::ToggleInterface(bool b) {
			// Panic button
			ui.btnPanic->setEnabled(b);
			
			// Markers
			ui.lblMarkers->setEnabled(b);
			ui.lblMarkertype->setEnabled(b);
			ui.lblXpos->setEnabled(b);
			ui.lblYpos->setEnabled(b);
			ui.lblAnglepos->setEnabled(b);
			ui.lblName->setEnabled(b);
			ui.btnAddCustomPose->setEnabled(b);
			ui.btnUpdateMarker->setEnabled(b);
			ui.btnAddCurrentPose->setEnabled(b);
			ui.btnClearAllMarkers->setEnabled(b);
			ui.btnRemoveMarker->setEnabled(b);
			ui.btnMoveMarkerUp->setEnabled(b);
			ui.btnMoveMarkerDown->setEnabled(b);
			ui.btnLoadMarkersYaml->setEnabled(b);
			ui.btnWriteMarkersYaml->setEnabled(b);
			ui.btnMoveRobot->setEnabled(b);
			ui.inpCustomX->setEnabled(b);
			ui.inpCustomY->setEnabled(b);
			ui.inpCustomAngle->setEnabled(b);
			ui.inpCustomName->setEnabled(b);
			ui.radioShelf->setEnabled(b);
			ui.radioWorkstation->setEnabled(b);
			ui.radioConveyor->setEnabled(b);
			ui.radioWaypoint->setEnabled(b);
			ui.radioPrecision->setEnabled(b);
			ui.tableMarkers->setEnabled(b);

			// No-go lines
			ui.lblNogo->setEnabled(b);
			ui.lblLineX1->setEnabled(b);
			ui.lblLineX2->setEnabled(b);
			ui.lblLineY1->setEnabled(b);
			ui.lblLineY2->setEnabled(b);
			ui.lblLineName->setEnabled(b);
			ui.btnNogoLine->setEnabled(b);
			ui.btnUpdateLine->setEnabled(b);
			ui.btnClearLine->setEnabled(b);
			ui.btnRemoveLine->setEnabled(b);
			ui.btnMoveLineUp->setEnabled(b);
			ui.btnMoveLineDown->setEnabled(b);
			ui.btnLoadLinesYaml->setEnabled(b);
			ui.btnWriteLinesYaml->setEnabled(b);
			ui.inpLineX1->setEnabled(b);
			ui.inpLineX2->setEnabled(b);
			ui.inpLineY1->setEnabled(b);
			ui.inpLineY2->setEnabled(b);
			ui.inpLineName->setEnabled(b);
			ui.tableLines->setEnabled(b);
	}

	int MainWindow::GetSelectedMarker() {
		int j = -1;
		QModelIndexList indexes = ui.tableMarkers->selectionModel()->selectedRows();

		for (int i = 0; i < indexes.count(); ++i) {    
			j = indexes.at(i).row();
		}

		return j;
	}

	void MainWindow::AddMarker(Marker marker) {
		markers.push_back(marker);
	}

	void MainWindow::UpdateMarker(int index, Marker marker) {
		markers.at(index) = marker;
	}

	void MainWindow::MoveMarkerUp(int selectedMarker) {
		if(selectedMarker > 0) {
			std::swap(markers.at(selectedMarker), markers.at(selectedMarker - 1));
		} else {
			ROS_WARN("No marker selected");
		}
	}

	void MainWindow::MoveMarkerDown(int selectedMarker) {
		if(selectedMarker + 1 < markers.size() && selectedMarker >= 0) {
			std::swap(markers.at(selectedMarker), markers.at(selectedMarker + 1));
		} else {
			ROS_WARN("No marker selected");
		}
	}

	void MainWindow::UpdateTable() {
		ui.tableMarkers->setRowCount(0);
		for(int i=0; i < markers.size(); i++) {
			ui.tableMarkers->insertRow ( ui.tableMarkers->rowCount() );
			ui.tableMarkers->setItem(ui.tableMarkers->rowCount()-1, 0, new QTableWidgetItem(QString::fromStdString(markers[i].GetName())));
			ui.tableMarkers->setItem(ui.tableMarkers->rowCount()-1, 1, new QTableWidgetItem(QString::fromStdString(markers[i].GetTypeStr())));
			ui.tableMarkers->setItem(ui.tableMarkers->rowCount()-1, 2, new QTableWidgetItem(QString::number(markers[i].GetX())));
			ui.tableMarkers->setItem(ui.tableMarkers->rowCount()-1, 3, new QTableWidgetItem(QString::number(markers[i].GetY())));
			ui.tableMarkers->setItem(ui.tableMarkers->rowCount()-1, 4, new QTableWidgetItem(QString::number(markers[i].GetAngle())));
		}

		UpdateWindow();
	}

	void MainWindow::SelectionIsChanged() {
		int s = GetSelectedMarker();

		if(s >= 0) {
			if(markers[s].GetType() == Shelf) {
				ui.radioShelf->setChecked(true);
				ui.radioWorkstation->setChecked(false);
				ui.radioConveyor->setChecked(false);
				ui.radioWaypoint->setChecked(false);
				ui.radioPrecision->setChecked(false);
			} else if (markers[s].GetType() == Workstation) {
				ui.radioShelf->setChecked(false);
				ui.radioWorkstation->setChecked(true);
				ui.radioConveyor->setChecked(false);
				ui.radioWaypoint->setChecked(false);
				ui.radioPrecision->setChecked(false);
			} else if (markers[s].GetType() == Conveyor) {
				ui.radioShelf->setChecked(false);
				ui.radioWorkstation->setChecked(false);
				ui.radioConveyor->setChecked(true);
				ui.radioWaypoint->setChecked(false);
				ui.radioPrecision->setChecked(false);
			} else if (markers[s].GetType() == Waypoint) {
				ui.radioShelf->setChecked(false);
				ui.radioWorkstation->setChecked(false);
				ui.radioConveyor->setChecked(false);
				ui.radioWaypoint->setChecked(true);
				ui.radioPrecision->setChecked(false);
			} else if (markers[s].GetType() == Precision) {
				ui.radioShelf->setChecked(false);
				ui.radioWorkstation->setChecked(false);
				ui.radioConveyor->setChecked(false);
				ui.radioWaypoint->setChecked(false);
				ui.radioPrecision->setChecked(true);
			}
			ui.inpCustomX->setText(QString::number(markers[s].GetX()));
			ui.inpCustomY->setText(QString::number(markers[s].GetY()));
			ui.inpCustomAngle->setText(QString::number(markers[s].GetAngle()));
			ui.inpCustomName->setText(QString::fromStdString(markers[s].GetName()));
		}

		UpdateWindow();
	}

	void MainWindow::UpdateWindow() {
		// Update window - draw map and points again
		this->update();
	}

	void MainWindow::UpdateRobotPose() {
		robotPose = qnode.GetRobotPosition();
		UpdateWindow();
	}

	geometry_msgs::Pose MainWindow::MakePose(double pX, double pY, double pZ, double qX, double qY, double qZ, double qW) {
		geometry_msgs::Point p;
		geometry_msgs::Quaternion q;
		geometry_msgs::Pose pos;

		p.x = pX;
		p.y = pY;
		p.z = pZ;

		q.x = qX;
		q.y = qY;
		q.z = qZ;
		q.w = qW;

		pos.position = p;
		pos.orientation = q;

		return pos;
	}

	void MainWindow::EnableInterface()
	{
		if(YamlLoaded && ImageLoaded && NodeStarted)
		{
			lblMapImage->setGeometry(QRect(0, 0, map_pix, map_pix));
			ToggleInterface(true);
		}
	}

	void MainWindow::ShowNoMasterMessage() {
		QMessageBox msgBox;
		msgBox.setText("Couldn't find the ros master...");
		msgBox.exec();
	    close();
	}


	int MainWindow::ConvertRealSizeToPixel(double a) {
		return (a - map_min) * (map_pix) / (map_max - map_min);
	}

	double MainWindow::ConvertPixelToRealSize(int a) {
		return (a) * (map_max - map_min) / (map_pix) + map_min;
	}
}
