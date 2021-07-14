#include "devicelist.h"


#include <QPoint>
#include <QMenu>


DeviceList::DeviceList()
{
  headers_ << "Name" << "Location";
}

// ----------------------------------------------------------------------------

int DeviceList::rowCount(const QModelIndex &parent) const
{
  return devices_.size();
}

// ----------------------------------------------------------------------------

int DeviceList::columnCount(const QModelIndex &parent) const
{
  return 2;
}

// ----------------------------------------------------------------------------

QVariant DeviceList::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (orientation == Qt::Horizontal)
    {
      if (role == Qt::DisplayRole)
        {
          return headers_.at(section);
        }
    }
  else
    {
      if (role == Qt::DisplayRole)
        {
          return section + 1;
        }
    }

  return QVariant();
}

// ----------------------------------------------------------------------------

QVariant DeviceList::data(const QModelIndex &index, int role) const
{
  if (role == Qt::DisplayRole)
    {
      int i = 0;
      auto it = devices_.begin();

      for (; it != devices_.end(); it++, i++)
        {
          if (i == index.row())
            break;
        }

      if (it == devices_.end())
        return QVariant();

      if (index.column() == 0)
        {
          return it->first;
        }
      else if (index.column() == 1)
        {
          return it->second;
         }
    }

  return QVariant();
}

// ----------------------------------------------------------------------------

void DeviceList::addDevice(const QString& name, const QString& location)
{
  emit beginResetModel();

  devices_[name] = location;

  emit endResetModel();
}

// ----------------------------------------------------------------------------

void DeviceList::removeDevice(const QString& name)
{
  emit beginResetModel();

  auto it = devices_.find(name);
  
  if (it != devices_.end())
    devices_.erase(it);

  emit endResetModel();
}

// ----------------------------------------------------------------------------

QVariant DeviceList::getDeviceLocation(const QModelIndex& index)
{
    int i = 0;
    auto it = devices_.begin();

    for (; it != devices_.end(); it++, i++)
    {
        if (i == index.row())
        break;
    }

    if (it == devices_.end())
      return QVariant();

    return it->second;
}