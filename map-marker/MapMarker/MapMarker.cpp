#include "MapMarker.h"
#include "ui_MapMarker.h"
#include "ClickableLabel.h"
//extern "C" {
//#include "yaml.h"
//}

MapMarker::MapMarker(QWidget *parent) : QMainWindow(parent), ui(new Ui::MapMarker) {
    ui->setupUi(this);

    QString url = R"(/home/viki/git/ESA-PROJ/map-marker/maps/legomap3-cropped.pgm)";
    QPixmap img(url);
    ClickableLabel *label = new ClickableLabel(this);
    QPoint p(0,0);
    label->setAlignment(Qt::AlignBottom | Qt::AlignRight);
    label->setGeometry(QRect(0,0,992,992));
    label->setPixmap(img);
}

MapMarker::~MapMarker() {
    delete ui;
}

void MapMarker::on_btnLoadYaml_clicked()
{
    /*FILE *fh = fopen("config/public.yaml", "r");
    yaml_parser_t parser;

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

    // Cleanup
    yaml_parser_delete(&parser);
    fclose(fh);*/
}
