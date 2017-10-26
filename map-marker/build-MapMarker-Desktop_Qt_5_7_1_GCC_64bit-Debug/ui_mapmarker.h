/********************************************************************************
** Form generated from reading UI file 'mapmarker.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAPMARKER_H
#define UI_MAPMARKER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MapMarker
{
public:
    QWidget *centralWidget;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;

    void setupUi(QMainWindow *MapMarker)
    {
        if (MapMarker->objectName().isEmpty())
            MapMarker->setObjectName(QStringLiteral("MapMarker"));
        MapMarker->resize(1200, 992);
        centralWidget = new QWidget(MapMarker);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(1000, 20, 121, 16));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(1000, 40, 59, 15));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(1000, 110, 91, 16));
        MapMarker->setCentralWidget(centralWidget);

        retranslateUi(MapMarker);

        QMetaObject::connectSlotsByName(MapMarker);
    } // setupUi

    void retranslateUi(QMainWindow *MapMarker)
    {
        MapMarker->setWindowTitle(QApplication::translate("MapMarker", "MapMarker", Q_NULLPTR));
        label->setText(QApplication::translate("MapMarker", "Lees yaml", Q_NULLPTR));
        label_2->setText(QApplication::translate("MapMarker", "Lees pgm", Q_NULLPTR));
        label_3->setText(QApplication::translate("MapMarker", "Schrijf yaml", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MapMarker: public Ui_MapMarker {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAPMARKER_H
