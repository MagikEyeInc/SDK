'''
* device_info
*
* Copyright (c) 2020-2021, Magik-Eye Inc.
* author: Jigar Patel, jigar@magik-eye.com
'''

# ============================================================================
# DeviceInfo

class DeviceInfo:
	# Constants
	START_SERVICE_PREFIX = "pymkeros2_startpublish_"
	STOP_SERVICE_PREFIX = "pymkeros2_stoppublish_"
	NODE_NAME_PREFIX = "pymkeros2_node_"
	TOPIC_NAME_PREFIX = "/pymkeros2_node_pcd_"
	START_SERVICE_NODE_PREFIX = "pymkeros2_start_service_node_pcd_"
	STOP_SERVICE_NODE_PREFIX = "pymkeros2_stop_service_node_pcd_"

	def __init__(self,id="",ip=""):
		self.ip_    = ip
		self.id_    = id
		self.alias_ = ""

	@staticmethod
	def removeSpecialChars(name):
		name = name.replace('.','_')
		name = name.replace('-','_')
		return name
	
	def getName(self):
		if not self.alias_:
			return self.id_
		else:
			return self.alias_

	def setAlias(self,alias = ""):
		if alias:
			self.alias_ = alias
		elif self.id_:
			self.alias_ = self.id_
		else:
			self.alias_ = self.ip_
	
	def getAlias(self):
		return self.alias_
	
	def setIpAddr(self,ip):
		self.ip_ = ip
	
	def getIpAddr(self):
		return self.ip_
	
	def setUnitId(self,id):
		self.id_ = id
	
	def getUnitId(self):
		return self.id_

	def getStartServiceName(self):
		name = DeviceInfo.START_SERVICE_PREFIX + self.getName()
		name = DeviceInfo.removeSpecialChars(name)
		return name

	def getStopServiceName(self):
		name = DeviceInfo.STOP_SERVICE_PREFIX + self.getName()
		name = DeviceInfo.removeSpecialChars(name)
		return name

	def getNodeName(self):
		name = DeviceInfo.NODE_NAME_PREFIX + self.getName()
		name = DeviceInfo.removeSpecialChars(name)
		return name

	def getStartServiceNodeName(self):
		name = DeviceInfo.START_SERVICE_NODE_PREFIX + self.getName()
		name = DeviceInfo.removeSpecialChars(name)
		return name
  	
	def getStopServiceNodeName(self):
		name = DeviceInfo.STOP_SERVICE_NODE_PREFIX + self.getName()
		name = self.removeSpecialChars(name)
		return name

	def getTopicName(self):
		name = DeviceInfo.TOPIC_NAME_PREFIX + self.getName()
		name = DeviceInfo.removeSpecialChars(name)
		return name
	
	def __str__(self):
		return "[UID: ]"+str(self.id_)+", IP:"+str(self.ip_)+", A: "\
			+str(self.alias_)

def main():
	# Multiple constructors
	device1 = DeviceInfo()
	device2 = DeviceInfo("192.168.43.67")

	# Test set/get IP and Unit ID
	device2.setUnitId("MagikEyeOne")
	print(device2.getUnitId())
	device2.setIpAddr("192.168.43.67")
	print(device2.getIpAddr())

	# Test Node name modifications
	print(device2)
	print(device2.getNodeName())
	print(device2.getTopicName())
	print(device2.getStartServiceName())
	print(device2.getStopServiceName())

if __name__ == '__main__':
    main()