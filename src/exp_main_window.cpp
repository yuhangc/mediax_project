#include "exp_main_window.h"
#include "ui_exp_main_window.h"

ExpMainWindow::ExpMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ExpMainWindow)
{
    ui->setupUi(this);
}

ExpMainWindow::~ExpMainWindow()
{
    delete ui;
}

void ExpMainWindow::Init()
{

}
