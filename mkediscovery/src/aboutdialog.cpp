#include "aboutdialog.h"
#include "ui_aboutdialog.h"

#include <QFile>
#include <QMessageBox>

// ----------------------------------------------------------------------------

AboutDialog::AboutDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::AboutDialog)
{
  ui->setupUi(this);

  setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);

  QFile file(":/html/about"); 
  file.open(QIODevice::ReadOnly | QIODevice::Text);  
  QString about_text = file.readAll();
  
  about_text.replace("@APP_NAME@", "MkEDiscovery");
  about_text.replace("@VERSION@", MKEDISCOVERY_VERSION);

  ui->html_label->setHtml(about_text);
}

// ----------------------------------------------------------------------------

AboutDialog::~AboutDialog()
{
  delete ui;
}

// ----------------------------------------------------------------------------

void AboutDialog::aboutQt()
{
  QMessageBox::aboutQt(this, "About Qt");
}
