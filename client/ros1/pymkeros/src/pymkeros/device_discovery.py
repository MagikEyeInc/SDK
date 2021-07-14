'''
* device_discovery
*
* Copyright (c) 2020-2021, Magik-Eye Inc.
* author: Jigar Patel, jigar@magik-eye.com
'''

import re
import time
import threading
import pymkeapi
from pymkeros.device_info import DeviceInfo

# =============================================================================


class DeviceDiscovery:
    IPADDR_REGEXP = "(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)"

    # =========================================================================

    def __init__(self, timeout=1):
        self.device_list_ = {}
        self.timeout_ = timeout

    # =========================================================================

    @staticmethod
    def isIpAddress(val):
        ipaddr_regexp = re.compile("^"+DeviceDiscovery.IPADDR_REGEXP+"$")
        if ipaddr_regexp.match(val):
            return True
        else:
            return False

    # =========================================================================

    def getDeviceList(self):
        return self.device_list_

    # =========================================================================

    def setTimeout(self, timeout):
        self.timeout_ = timeout

    # =========================================================================

    def updateDeviceList(self, timeout=0):
        self.device_list_.clear()

        if timeout == 0:
            timeout = self.timeout_

        self.device_list_ = pymkeapi.discover_devices(timeout)

    # =========================================================================

    def validateDevice(self, device_name, device_info=None):
        # Iterate to find in device_list
        for uid, ip in self.device_list_.items():
            if((uid == device_name) or (ip == device_name)):
                if device_info:
                    device_info.setIpAddr(ip)
                    device_info.setUnitId(uid)
                return True
        else:
            return False

# =============================================================================
# =============================================================================


if __name__ == "__main__":
    # Test DeviceDiscovery
    discovery_1 = DeviceDiscovery(2)
    discovery_2 = DeviceDiscovery()

    # Test isIpAddress
    print(discovery_1.isIpAddress("000.12.12.034"))  # True
    print(discovery_1.isIpAddress("000.12.234.23.23"))  # False

    # Test updateDeviceList
    discovery_1.updateDeviceList(1)

    # Test getDeviceList
    print(discovery_1.getDeviceList())

    # Test validateDevice
    device = DeviceInfo()
    print(discovery_1.validateDevice("192.168.43.67"))
    print(discovery_1.validateDevice("192.168.43.67", device))
    print(discovery_1.validateDevice("MagikEyeOne-34cff660"))
    print(discovery_1.validateDevice("MagikEyeOne-34cff660", device))
    print(device)
