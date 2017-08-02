import json
import urllib.request
import urllib.error
from os import system
'''
url_building = "Building=DemoBuilding"
url_level = "Level=1"
url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?" + url_building + "&" + url_level
html = urllib2.urlopen(url)
map = json.loads(html.read())

print map
print map["info"]["northAt"]
'''

class grab_map(object):

	def __init__(self, buil, lel):
		self.building = buil
		self.level = lel
		self.internet_err = 0
		self.info = ""

	# Ask for the building and level
	def get_building_level(self):
		#espeak.synth('name')
		system("echo please input the name of the building | festival --tts")
		self.building = input("Please input the name of building: ")
		system("echo the number of level| festival --tts")
		#espeak.synth('building')
		self.level = input("Please input the number of level: ")

	# Get the building detail from the specific building and level.
	# And deal with some errors because of internet
	def grab_info(self):
		url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php?Building=" + self.building + "&Level=" + self.level
		
		
		try:
			
			html = urllib.request.urlopen(url)
			# print (html.read().decode('utf-8'))
			tmp = html.read().decode()
			# print(tmp)
			# self.info = json.loads(str(html.read().decode('utf8')))
			self.info = json.loads(tmp)
			# print self.info["info"]
			if (self.info["info"] == None):
				# print "wrong_input"
				return "wrong_input" 
			else:
				return "success"
		except urllib.error.HTTPError as e:
			
			#("HTTPError:")
			#if hasattr(e,"code"):
			#	print e.code
			#if hasattr(e,"reason"):
			#	print e.reason
			
			return "http_error"
		

		except urllib.error.URLError as e:
			
			#if hasattr(e,"code"):
			#	print e.code
			#if hasattr(e,"reason"):
			#	print e.reason
			#print("URLError")
			
			return "url_error"
		


	# Write the info to the file
	def write_file(self):
		filename = "./map/" + self.building + "-" + self.level + ".3h"
		# print filename
		fo = open(filename, "w")
		fo.write(str(self.info))
		fo.close()

	# When it can't access the internet, we will find map in the cache.
	def visit_cache(self):
		# print self.building
		filename = "./map/" + self.building + "-" + self.level + ".3h"
		# print filename
		try:
			# print "666" + self.info
			fo = open(filename, "r")
			self.info = eval(fo.read())
			fo.close()
			#print self.info
			print("Read from cache successfully!")
			return "success"
		except IOError:
			print("There is no " + str(self.building) + "-" + str(self.level) + ".3h" + " such file")
			return "error"

	def execute(self):
		self.get_building_level()
		ret = self.grab_info()
		if (ret == "success"):
			# print "success------"
			self.write_file()
			return "success"
		elif (ret == "url_error"):
			print ("Can't access the internet.")
			result = self.visit_cache()
			# print "map_666"
			if result == "success":
				return "success"
			else:
				return "error"
		else:
			print (ret)
			return "error"



if __name__ == '__main__':
	myJson = grab_map("DemoBuilding", "1")
	myJson.execute()
