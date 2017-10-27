#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/map_marker/main_window.hpp"
#include "../include/map_marker/ClickableLabel.hpp"

//extern "C" {
//#include "yaml.h"
//}

using namespace Qt;

namespace map_marker {

	MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc,argv) {
		ui.setupUi(this);
		qnode.init();
		QObject::connect(ui.btnLoadYaml, SIGNAL(clicked(bool)), this, SLOT(on_btnLoadYaml_clicked()));
		QObject::connect(ui.btnLoadMap, SIGNAL(clicked(bool)), this, SLOT(on_btnLoadMap_clicked()));
		QObject::connect(ui.btnWriteYaml, SIGNAL(clicked(bool)), this, SLOT(on_btnWriteYaml_clicked()));
		QObject::connect(ui.btnClearYaml, SIGNAL(clicked(bool)), this, SLOT(on_btnClearYaml_clicked()));
		QObject::connect(ui.btnAddCurrentPose, SIGNAL(clicked(bool)), this, SLOT(on_btnAddCurrentPose_clicked()));
		QObject::connect(ui.btnAddCustomPose, SIGNAL(clicked(bool)), this, SLOT(on_btnAddCustomPose_clicked()));

		QString url = "/home/viki/git/ESA-PROJ/maps/legomap3-cropped.pgm";
	    QPixmap img(url);
	    ClickableLabel *label = new ClickableLabel(this);
	    QPoint p(0,0);
	    label->setAlignment(Qt::AlignBottom | Qt::AlignRight);
	    label->setGeometry(QRect(0,0,992,992));
	    label->setPixmap(img);
	}

	MainWindow::~MainWindow() {

	}

	void MainWindow::on_btnLoadYaml_clicked() {
		
	}

	void MainWindow::on_btnLoadMap_clicked() {

	}

	void MainWindow::on_btnWriteYaml_clicked() {

	}

	void MainWindow::on_btnClearYaml_clicked() {

	}

	void MainWindow::on_btnAddCurrentPose_clicked() {

	}

	void MainWindow::on_btnAddCustomPose_clicked() {

	}


//void MapMarker::on_btnLoadYaml_clicked()
//{
//    FILE *fh = fopen("../maps/legomap-cropped.yaml", "r");
//    yaml_parser_t parser;
//yaml_token_t  token;   /* new variable */
//    // Initialize parser
//    if(!yaml_parser_initialize(&parser))
//    {
//    fputs("Failed to initialize parser!\n", stderr);
//    }
//    if(fh == NULL)
//    {
//    fputs("Failed to open file!\n", stderr);
//    }

//    // Set input file
//    yaml_parser_set_input_file(&parser, fh);

//    // CODE HERE
//    do {
//        yaml_parser_scan(&parser, &token);
//        switch(token.type)
//        {
//        /* Stream start/end */
//        case YAML_STREAM_START_TOKEN: puts("STREAM START"); break;
//        case YAML_STREAM_END_TOKEN:   puts("STREAM END");   break;
//        /* Token types (read before actual token) */
//        case YAML_KEY_TOKEN:   printf("(Key token)   "); break;
//        case YAML_VALUE_TOKEN: printf("(Value token) "); break;
//        /* Block delimeters */
//        case YAML_BLOCK_SEQUENCE_START_TOKEN: puts("<b>Start Block (Sequence)</b>"); break;
//        case YAML_BLOCK_ENTRY_TOKEN:          puts("<b>Start Block (Entry)</b>");    break;
//        case YAML_BLOCK_END_TOKEN:            puts("<b>End block</b>");              break;
//        /* Data */
//        case YAML_BLOCK_MAPPING_START_TOKEN:  puts("[Block mapping]");            break;
//        case YAML_SCALAR_TOKEN:  printf("scalar %s \n", token.data.scalar.value); break;
//        /* Others */
//        default:
//          printf("Got token of type %d\n", token.type);
//        }
//        if(token.type != YAML_STREAM_END_TOKEN)
//          yaml_token_delete(&token);
//      } while(token.type != YAML_STREAM_END_TOKEN);
//      yaml_token_delete(&token);
//    // Cleanup
//    yaml_parser_delete(&parser);
//    fclose(fh);
//}

}
