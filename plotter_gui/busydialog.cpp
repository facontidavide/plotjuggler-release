#include "busydialog.h"
#include "ui_busydialog.h"
#include <QMovie>

BusyDialog::BusyDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::BusyDialog)
{
    ui->setupUi(this);

    setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);

    this->move( (parent->width() )/2,
                (parent->height()/2 - height())   );

    QMovie *movie = new QMovie(":/icons/resources/loader.gif");
    QLabel *processLabel = new QLabel(this);
    processLabel->setMovie(movie);

    ui->verticalLayout->addWidget(processLabel);
    movie->start();
    this->show();

    this->setAttribute( Qt::WA_DeleteOnClose, true );
}

BusyDialog::~BusyDialog()
{
    delete ui;
}
