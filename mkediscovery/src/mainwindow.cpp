#include <QUrl>
#include <QDesktopServices>
#include <QSettings>
#include <QDesktopWidget>
#include <QStyle>

#include "aboutdialog.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "mke/net/ssdp/discovery.h"

// ----------------------------------------------------------------------------

MainWindow::MainWindow(QWidget* parent)
: QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  
  // Version
  auto info_text = ui->info_label->text();
  info_text.replace("@VERSION@", MKEDISCOVERY_VERSION);
  ui->info_label->setText(info_text);

  // MkE logos
  QColor color = this->palette().color(QWidget::backgroundRole());

  if (color.lightness() < 128)
    {
      ui->logo_label->setPixmap(QPixmap(QString::fromUtf8(":/img/logos_inv")));
    }

  // Device table
  ui->device_table->setModel(&devicelist_);
  ui->device_table->horizontalHeader()->setSectionResizeMode(
    QHeaderView::Stretch);
  
  // Initialize the window position to the center of the screen
  this->setGeometry(
    QStyle::alignedRect(
        Qt::LeftToRight,
        Qt::AlignCenter,
        this->size(),
        qApp->desktop()->availableGeometry()
    )
  );

  // Override the window position using a saved geometry, if availbe
  QSettings settings("MagikEye", "MkEDiscovery");
  restoreGeometry(settings.value("geometry").toByteArray());

  try
    {
      // Start SSDP daemon
      ssdpd_.setPatternURN("urn:schemas-upnp-org:device:Basic:1");
      ssdpd_.setPatternProtocol("MkE");

      ssdpd_.setDeviceAddedCallback(
        [this](const std::string& usn, const std::string& location,
               const std::string& xml_location, const std::string& protocol,
               const std::string& device_name, const std::string& unit_id,
               const std::string& urn) {
          QString q_usn(QLatin1String(usn.c_str()));
          QString q_location(QLatin1String(location.c_str()));
          QString q_xml_location(QLatin1String(xml_location.c_str()));
          QString q_apiType(QLatin1String(protocol.c_str()));
          QString q_device_name(QLatin1String(device_name.c_str()));
          QString q_unit_id(QLatin1String(unit_id.c_str()));

          QMetaObject::invokeMethod(
            this, "onSsdpDeviceAdded", Qt::QueuedConnection,
            Q_ARG(QString, q_usn), Q_ARG(QString, q_location),
            Q_ARG(QString, q_xml_location), Q_ARG(QString, q_apiType),
            Q_ARG(QString, q_device_name), Q_ARG(QString, q_unit_id));
        });

      ssdpd_.setDeviceRemovedCallback(
        [this](const std::string& usn, const std::string& location,
               const std::string& xml_location, const std::string& protocol,
               const std::string& device_name, const std::string& unit_id,
               const std::string& urn) {
          QString q_usn(QLatin1String(usn.c_str()));
          QString q_location(QLatin1String(location.c_str()));
          QString q_xml_location(QLatin1String(xml_location.c_str()));
          QString q_apiType(QLatin1String(protocol.c_str()));
          QString q_device_name(QLatin1String(device_name.c_str()));
          QString q_unit_id(QLatin1String(unit_id.c_str()));

          QMetaObject::invokeMethod(
            this, "onSsdpDeviceRemoved", Qt::QueuedConnection,
            Q_ARG(QString, q_usn), Q_ARG(QString, q_location),
            Q_ARG(QString, q_xml_location), Q_ARG(QString, q_apiType),
            Q_ARG(QString, q_device_name), Q_ARG(QString, q_unit_id));
        });

      ssdpd_.start();
    }
  catch (std::exception& e)
    {
    }
}

// ----------------------------------------------------------------------------


void MainWindow::onSsdpDeviceAdded(QString usn,
                                   QString location,
                                   QString xml_location,
                                   QString apiType,
                                   QString device_name,
                                   QString unit_id)
{
  try
    {
      QString name = device_name + "-" + unit_id;
      devicelist_.addDevice(name, location);
    }
  catch (std::exception& e)
    {
    }
}

// ----------------------------------------------------------------------------

void MainWindow::onSsdpDeviceRemoved(QString usn,
                                     QString location,
                                     QString xml_location,
                                     QString apiType,
                                     QString device_name,
                                     QString unit_id)
{
  try
    {
      QString name = device_name + "-" + unit_id;
      devicelist_.removeDevice(name);
    }
  catch (std::exception& e)
    {
    }
}

// ----------------------------------------------------------------------------

void MainWindow::deviceSelected(QModelIndex index)
{
  auto location = devicelist_.getDeviceLocation(index);

  if (location.toBool())
    QDesktopServices::openUrl(
      QUrl(QString("http://") + location.toString(), QUrl::TolerantMode));
}

// ----------------------------------------------------------------------------

void MainWindow::showAboutDialog()
{
  AboutDialog about(this);
  about.exec();
}

// ----------------------------------------------------------------------------

MainWindow::~MainWindow()
{
  ssdpd_.stop();

  QSettings settings("MagikEye", "MkEDiscovery");
  settings.setValue("geometry", saveGeometry());

  delete ui;
}
