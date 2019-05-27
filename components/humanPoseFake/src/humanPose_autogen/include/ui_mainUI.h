/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpinBox *cameraID_sb;
    QSpacerItem *horizontalSpacer;
    QLabel *label_2;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout;
    QLineEdit *name_le;
    QTextEdit *person_te;
    QVBoxLayout *verticalLayout_3;
    QSpacerItem *verticalSpacer;
    QPushButton *add_pb;
    QPushButton *remove_bt;
    QSpacerItem *verticalSpacer_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_4;
    QListWidget *frames_list;
    QPushButton *clear_pb;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *load_pb;
    QPushButton *save_pb;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *publish_pb;
    QLabel *time_l;
    QDoubleSpinBox *timer_sb;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QStringLiteral("guiDlg"));
        guiDlg->resize(587, 391);
        verticalLayout_4 = new QVBoxLayout(guiDlg);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(guiDlg);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        cameraID_sb = new QSpinBox(guiDlg);
        cameraID_sb->setObjectName(QStringLiteral("cameraID_sb"));

        horizontalLayout->addWidget(cameraID_sb);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout_4->addLayout(horizontalLayout);

        label_2 = new QLabel(guiDlg);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_4->addWidget(label_2);

        label_3 = new QLabel(guiDlg);
        label_3->setObjectName(QStringLiteral("label_3"));

        verticalLayout_4->addWidget(label_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        name_le = new QLineEdit(guiDlg);
        name_le->setObjectName(QStringLiteral("name_le"));

        verticalLayout->addWidget(name_le);

        person_te = new QTextEdit(guiDlg);
        person_te->setObjectName(QStringLiteral("person_te"));

        verticalLayout->addWidget(person_te);


        horizontalLayout_3->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        add_pb = new QPushButton(guiDlg);
        add_pb->setObjectName(QStringLiteral("add_pb"));
        add_pb->setMaximumSize(QSize(30, 16777215));

        verticalLayout_3->addWidget(add_pb);

        remove_bt = new QPushButton(guiDlg);
        remove_bt->setObjectName(QStringLiteral("remove_bt"));
        remove_bt->setMaximumSize(QSize(30, 16777215));

        verticalLayout_3->addWidget(remove_bt);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_2);


        horizontalLayout_3->addLayout(verticalLayout_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        label_4 = new QLabel(guiDlg);
        label_4->setObjectName(QStringLiteral("label_4"));

        verticalLayout_2->addWidget(label_4);

        frames_list = new QListWidget(guiDlg);
        frames_list->setObjectName(QStringLiteral("frames_list"));
        frames_list->setSelectionMode(QAbstractItemView::ExtendedSelection);

        verticalLayout_2->addWidget(frames_list);

        clear_pb = new QPushButton(guiDlg);
        clear_pb->setObjectName(QStringLiteral("clear_pb"));

        verticalLayout_2->addWidget(clear_pb);


        horizontalLayout_3->addLayout(verticalLayout_2);

        horizontalLayout_3->setStretch(0, 100);
        horizontalLayout_3->setStretch(2, 100);

        verticalLayout_4->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        load_pb = new QPushButton(guiDlg);
        load_pb->setObjectName(QStringLiteral("load_pb"));

        horizontalLayout_2->addWidget(load_pb);

        save_pb = new QPushButton(guiDlg);
        save_pb->setObjectName(QStringLiteral("save_pb"));

        horizontalLayout_2->addWidget(save_pb);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        publish_pb = new QPushButton(guiDlg);
        publish_pb->setObjectName(QStringLiteral("publish_pb"));

        horizontalLayout_2->addWidget(publish_pb);

        time_l = new QLabel(guiDlg);
        time_l->setObjectName(QStringLiteral("time_l"));

        horizontalLayout_2->addWidget(time_l);

        timer_sb = new QDoubleSpinBox(guiDlg);
        timer_sb->setObjectName(QStringLiteral("timer_sb"));
        timer_sb->setDecimals(2);
        timer_sb->setMaximum(30);
        timer_sb->setValue(1);

        horizontalLayout_2->addWidget(timer_sb);


        verticalLayout_4->addLayout(horizontalLayout_2);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "humanPose", Q_NULLPTR));
        label->setText(QApplication::translate("guiDlg", "CameraID", Q_NULLPTR));
        label_2->setText(QApplication::translate("guiDlg", "ID,  Pose3D( X, Y, ROT, POS_GOOD, ROT_GOOD, confidence)", Q_NULLPTR));
        label_3->setText(QApplication::translate("guiDlg", "int, Pose3D( float, float, float, bool, bool, int)", Q_NULLPTR));
        name_le->setPlaceholderText(QApplication::translate("guiDlg", "Name for the poses frame", Q_NULLPTR));
        person_te->setHtml(QApplication::translate("guiDlg", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Ubuntu'; font-size:11pt;\">1, Pose(1000,1000,1.43,true, true, 90)</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Ubuntu'; font-size:11pt;\">2, Pose(1000,3000,-1.43,true, true, 65)</span></p></body></html>", Q_NULLPTR));
        add_pb->setText(QApplication::translate("guiDlg", "+", Q_NULLPTR));
        remove_bt->setText(QApplication::translate("guiDlg", "-", Q_NULLPTR));
        label_4->setText(QApplication::translate("guiDlg", "Frames to publish:", Q_NULLPTR));
        clear_pb->setText(QApplication::translate("guiDlg", "Clear", Q_NULLPTR));
        load_pb->setText(QApplication::translate("guiDlg", "load", Q_NULLPTR));
        save_pb->setText(QApplication::translate("guiDlg", "save", Q_NULLPTR));
        publish_pb->setText(QApplication::translate("guiDlg", "Publish", Q_NULLPTR));
        time_l->setText(QApplication::translate("guiDlg", "Sleep time:", Q_NULLPTR));
        timer_sb->setSuffix(QApplication::translate("guiDlg", "sec", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
