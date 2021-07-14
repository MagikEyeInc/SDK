#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <cstdio>
#include <utility>

#include "mke/net/ssdp/discovery.h"

// ============================================================================

int main(int argc, char* argv[])
{
  try
    {

      mke::net::ssdp::Discovery discovery;

      discovery.setPatternPrefix("uuid:UPnP");
      discovery.setPatternProtocol("MkE");
      discovery.setPatternDeviceName("");
      discovery.setPatternUnitID("");
      discovery.setPatternURN("urn:schemas-upnp-org:device:Basic:1");

      discovery.setDeviceAddedCallback([](const std::string& usn,
                                          const std::string& location,
                                          const std::string& xml_location,
                                          const std::string& protocol,
                                          const std::string& device_name,
                                          const std::string& unit_id,
                                          const std::string& urn) {
        std::cout << "=== Device added ===" << std::endl;
        std::cout << "USN: " << usn << std::endl; // e.g.: uuid:UPnP-MkE-MagikEyeOne-00e04c68025b::urn:schemas-upnp-org:device:Basic:1
        std::cout << "Location: " << location << std::endl; // e.g.: 192.168.0.121
        std::cout << "XML location: " << xml_location << std::endl; // e.g.: http://192.168.0.121:49152/description.xml
        std::cout << "Protocol: " << protocol << std::endl; // e.g.: MkE
        std::cout << "Device name: " << device_name << std::endl; // e.g.: MagikEyeOne
        std::cout << "Device sn: " << unit_id << std::endl; // e.g.: 00e04c68025b
        std::cout << "URN: " << urn << std::endl; // e.g.: urn:schemas-upnp-org:device:Basic:1
        std::cout << "===" << std::endl;
      });
      discovery.setDeviceRemovedCallback(
        [](const std::string& usn, const std::string& location,
           const std::string& xml_location, const std::string& protocol,
           const std::string& device_name, const std::string& unit_id,
           const std::string& urn) {
          std::cout << "Device removed: " << usn << " : " << location << " : "
                    << xml_location << std::endl;
        });

      discovery.start();

      while (1)
        {
          std::this_thread::sleep_for(std::chrono::seconds(10));
        }

      discovery.stop();
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}
