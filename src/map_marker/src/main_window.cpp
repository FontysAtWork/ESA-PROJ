#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <QGraphicsScene>
#include <iostream>
#include <algorithm>
#include "main_window.hpp"

#include <QDebug>

#include "geometry_msgs/Pose.h"

extern "C" {
#include "yaml.h"
}

using namespace Qt;

namespace map_marker {

	MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc,argv) {
		ui.setupUi(this);
		qnode.init();

		/* HOEFT NIET
		QObject::connect(ui.btnLoadYaml, SIGNAL(clicked(bool)), this, SLOT(on_btnLoadYaml_clicked()));
		QObject::connect(ui.btnLoadMap, SIGNAL(clicked(bool)), this, SLOT(on_btnLoadMap_clicked()));
		QObject::connect(ui.btnWriteYaml, SIGNAL(clicked(bool)), this, SLOT(on_btnWriteYaml_clicked()));
		QObject::connect(ui.btnClearYaml, SIGNAL(clicked(bool)), this, SLOT(on_btnClearYaml_clicked()));
		QObject::connect(ui.btnAddCurrentPose, SIGNAL(clicked(bool)), this, SLOT(on_btnAddCurrentPose_clicked()));
		QObject::connect(ui.btnAddCustomPose, SIGNAL(clicked(bool)), this, SLOT(on_btnAddCustomPose_clicked()));
		*/

		QString url = "/home/youbot/git/gui/ESA-Proj/maps/legomap3-cropped.pgm";
	    QPixmap mapImg(url);

	    // Ttable editing
	    ui.tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	    ui.tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	    ui.tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	    
	    // Create map image
	    lblMapImage = new ClickableLabel(this);
	    lblMapImage->setAlignment(Qt::AlignBottom | Qt::AlignRight);
	    lblMapImage->setGeometry(QRect(0,0,992,992));
	    lblMapImage->setPixmap(mapImg);
	    QObject::connect(lblMapImage, SIGNAL(clicked(QPoint)), this, SLOT(lblMapImage_clicked(QPoint)));

	    // Set validator for input fields
	    ui.inpCustomX->setValidator(new QDoubleValidator(-100, 100, 5, ui.inpCustomX));
	    ui.inpCustomY->setValidator(new QDoubleValidator(-100, 100, 5, ui.inpCustomY));
	    ui.inpCustomAngle->setValidator(new QDoubleValidator(0, 360, 5, ui.inpCustomAngle));

	    // Panic button color
	    ui.btnPanic->setStyleSheet("color: rgb(192,0,0);");

	    Marker m(1.0, 2.0, 40.0, Navigation);
		AddMarker(m);

	    UpdateTable();
	}

	MainWindow::~MainWindow() {

	}

	void MainWindow::lblMapImage_clicked(QPoint a) {
		QString x = QString::number(a.x());
		QString y = QString::number(a.y());
		ui.inpCustomX->setText(x);
		ui.inpCustomY->setText(y);
	}

	void MainWindow::on_btnLoadYaml_clicked() {

		FILE *fh = fopen("/home/youbot/git/gui/ESA-Proj/maps/legomap-cropped.yaml", "r");
		yaml_parser_t parser;
		yaml_token_t  token;   // new variable 
		// Initialize parser
		if(!yaml_parser_initialize(&parser))
		{
			fputs("Failed to initialize parser!\n", stderr);
		}
		if(fh == NULL)
		{
			fputs("Failed to open file!\n", stderr);
		}

		// Set input file
		yaml_parser_set_input_file(&parser, fh);

		// CODE HERE
		do {
			yaml_parser_scan(&parser, &token);

			switch(token.type)
			{
				// Stream start/end 
				case YAML_STREAM_START_TOKEN: 
				std::cout << "STREAM START" << std::endl; break;//puts("STREAM START"); break;
				case YAML_STREAM_END_TOKEN:   
				std::cout << "STREAM END" << std::endl; break;//puts("STREAM END");   break;
				// Token types (read before actual token) 
				case YAML_KEY_TOKEN:   
				std::cout << "(Key token)   "; break;//   printf("(Key token)   "); break;
				case YAML_VALUE_TOKEN:   
				std::cout << "(Data token)  "; break;// printf("(Value token) "); break;
				// Block delimeters 
				case YAML_BLOCK_SEQUENCE_START_TOKEN:   
				std::cout << "<b>Start Block (Sequence)</b>" << std::endl; break;// puts("<b>Start Block (Sequence)</b>"); break;
				case YAML_BLOCK_ENTRY_TOKEN:   
				std::cout << "<b>Start Block (Entry)</b>" << std::endl; break;//          puts("<b>Start Block (Entry)</b>");    break;
				case YAML_BLOCK_END_TOKEN:   
				std::cout << "<b>End block</b>" << std::endl; break;//            puts("<b>End block</b>");              break;
				// Data 
				case YAML_BLOCK_MAPPING_START_TOKEN:   
				std::cout << "[Block mapping]" << std::endl; break;//  puts("[Block mapping]");            break;
				case YAML_SCALAR_TOKEN:   
				std::cout << "value   " << token.data.scalar.value << std::endl; break;//  printf("scalar %s \n", token.data.scalar.value); break;
				// Array
				case YAML_FLOW_SEQUENCE_START_TOKEN:   
				std::cout << "Start array!" << std::endl; break;// puts("Start array!"); 	break;
				case YAML_FLOW_SEQUENCE_END_TOKEN:   
				std::cout << "End array!" << std::endl; break;// puts("End array!"); break;
				case YAML_FLOW_ENTRY_TOKEN:   
				std::cout << "New array item: " << std::endl; break;// puts("New array item: "); break;
				// Others 
				default:
				std::cout << "Got token of type: " << token.type << std::endl; break;//printf("Got token of type %d\n", token.type);
			}
			if(token.type != YAML_STREAM_END_TOKEN)
			yaml_token_delete(&token);
		} while(token.type != YAML_STREAM_END_TOKEN);
		yaml_token_delete(&token);
		// Cleanup
		yaml_parser_delete(&parser);
		fclose(fh);
	}

	void MainWindow::on_btnLoadMap_clicked() {
		QFileDialog dialog(this);
		dialog.setFileMode(QFileDialog::AnyFile);
		dialog.setNameFilter(tr("Map image file (*.pbm *.pgm *.ppm)"));

		QStringList fileNames;
		if (dialog.exec())
    	fileNames = dialog.selectedFiles();

    	QPixmap mapImg(fileNames[0]);
    	lblMapImage->setPixmap(mapImg);
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

	int MainWindow::GetSelectedMarker() {
		int j = -1;
	    QModelIndexList indexes = ui.tableWidget->selectionModel()->selectedRows();

	 	for (int i = 0; i < indexes.count(); ++i)
	 	{    
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
	}
}
