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

if __name__ == '__main__':
  rospy.init_node('prova', anonymous=True)

#	The idea is to save the params in mongo using an initial yaml file to generate ros-params

  go_to_server_collection.delete_many({})
  go_to_param = rospy.get_param("/go_to_location_server")
  go_to_server_collection.insert_one(go_to_param)

  outbound_server_collection.delete_many({})
  outbound_param = rospy.get_param("/outbound_place_server")
  outbound_server_collection.insert_one(outbound_param)

  inbound_server_collection.delete_many({})
  inbound_param = rospy.get_param("/inbound_pick_server")
  inbound_server_collection.insert_one(inbound_param)

  multi_skills_collection.delete_many({})
  multi_skills_param = rospy.get_param("/multi_skills")
  multi_skills_collection.insert_one(multi_skills_param)

