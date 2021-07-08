#!/usr/bin/env python3

import pymongo
import pprint
import rospy

from std_msgs.msg import String
from pymongo import MongoClient

client = MongoClient('localhost', 27017)

db = client.interface_data

go_to_server_collection    = db.go_to_location_server
outbound_server_collection = db.outbound_place_server
inbound_server_collection  = db.inbound_pick_server
multi_skills_collection    = db.multi_skills
objects_collection         = db.objects

if __name__ == '__main__':
	rospy.init_node('prova', anonymous=True)

	go_to_server_collection.delete_many({})
	go_to_param = rospy.get_param("/go_to_location_server")
	go_to_server_collection.insert(go_to_param)

	outbound_server_collection.delete_many({})
	outbound_param = rospy.get_param("/outbound_place_server")
	outbound_server_collection.insert(outbound_param)

	inbound_server_collection.delete_many({})
	inbound_param = rospy.get_param("/inbound_pick_server")
	inbound_server_collection.insert(inbound_param)

	multi_skills_collection.delete_many({})
	multi_skills_param = rospy.get_param("/multi_skills")
	multi_skills_collection.insert(multi_skills_param)

	objects_collection.delete_many({})
	objects_param = rospy.get_param("/manipulation_objects")
	objects_collection.insert(objects_param)
