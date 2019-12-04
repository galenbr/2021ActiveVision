#include "mainWindow.hpp"

MainWindow::MainWindow() :
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
