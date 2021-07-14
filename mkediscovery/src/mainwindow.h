#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "mke/net/ssdp/discovery.h"
#include "devicelist.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
  Q_OBJECT

 private slots:
  // void onShowDeviceTableContextMenu(const QPoint &pos);
  // void onDeviceTabledoubleClicked(const QModelIndex &index);

  void onSsdpDeviceAdded(QString usn,
                         QString location,
                         QString xml_location,
                         QString apiType,
                         QString device_name,
                         QString unit_id);
  void onSsdpDeviceRemoved(QString usn,
                           QString location,
                           QString xml_location,
                           QString apiType,
                           QString device_name,
                           QString unit_id);

  void deviceSelected(QModelIndex);
  void showAboutDialog();

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 private:
  Ui::MainWindow* ui;
  mke::net::ssdp::Discovery ssdpd_;
  DeviceList devicelist_;

};
#endif // MAINWINDOW_H
