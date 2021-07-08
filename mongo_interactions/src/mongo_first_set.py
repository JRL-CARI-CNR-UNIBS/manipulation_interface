#!/usr/bin/env python3

import pymongo
import pprint
import rospy

from std_msgs.msg import String
from pymongo import MongoClient

client = MongoClient('localhost', 27017)

db = client.interface_data

go_to_collection     = db.go_to_locations
boxes_collection     = db.boxes
objects_collection   = db.objects_types
picks_collection     = db.picks
places_collection    = db.places
slots_collection     = db.slots
groups_collection    = db.groups
obj_names_collection = db.objects_names
robots_collection    = db.robots

if __name__ == '__main__':
	rospy.init_node('prova', anonymous=True)

	go_to_collection.delete_many({})
	go_to_param = rospy.get_param("/go_to_location")
	go_to_collection.insert(go_to_param)

	boxes_collection.delete_many({})
	boxes_param = rospy.get_param("/inbound/boxes")
	boxes_collection.insert(boxes_param)

	slots_collection.delete_many({})
	slots_param = rospy.get_param("/outbound/slots")
	slots_collection.insert(slots_param)

	groups_collection.delete_many({})
	groups_param = rospy.get_param("/outbound/slots_group")
	groups_collection.insert(groups_param)

	obj_names_collection.delete_many({})
	obj_name_param = rospy.get_param("/objects_names")
	obj_names_collection.insert(obj_name_param)

	try:
		param_name = rospy.get_param_names()
	except ROSException:
		print("could not get param name")

	objects_collection.delete_many({})
	for param in param_name:
		if ( param.find( '/manipulation_objects/' ) != -1 ):
			if ( param.find( 'grasp_poses' ) != -1 ):
				object_param = rospy.get_param(param.replace('/grasp_poses',''))
				objects_collection.insert(object_param)

	robots_collection.delete_many({})
	robots_param = rospy.get_param("/go_to_location_server/groups")
	robots_collection.insert(robots_param)

#	picks_collection.delete_many({})
#	picks_param = rospy.get_param("/picks_param")
#	picks_collection.insert(picks_param)

#	places_collection.delete_many({})
#	places_param = rospy.get_param("/places_param")
#	places_collection.insert(places_param)




