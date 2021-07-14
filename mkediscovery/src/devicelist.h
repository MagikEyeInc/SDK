#pragma once

#include <map>

#include <QAbstractTableModel>
#include <QStringList>
#include <QIcon>


class DeviceList : public QAbstractTableModel
{
  Q_OBJECT

 public:
  DeviceList();

  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant headerData(int section,
                      Qt::Orientation orientation,
                      int role) const override;
  QVariant data(const QModelIndex& index,
                int role = Qt::DisplayRole) const override;

  void addDevice(const QString &name, const QString &location);
  void removeDevice(const QString &name);
  QVariant getDeviceLocation(const QModelIndex& index);

 public slots:
 // void onDeviceAdded(device::Device* device_ptr, int index);
 // void onDeviceRemoved(QString name, int index);

private:
  QStringList headers_;
  std::map<QString, QString> devices_;
};
