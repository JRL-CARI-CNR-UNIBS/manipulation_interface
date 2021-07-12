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
picks_collection     = db.picks
places_collection    = db.places
slots_collection     = db.slots
groups_collection    = db.groups
objects_collection   = db.objects

if __name__ == '__main__':
  rospy.init_node('prova', anonymous=True)

  go_to_collection.delete_many({})
  go_to_param = rospy.get_param("/go_to_location")
  go_to_collection.insert_many(go_to_param)

  boxes_collection.delete_many({})
  boxes_param = rospy.get_param("/inbound/boxes")
  boxes_collection.insert_many(boxes_param)

  slots_collection.delete_many({})
  slots_param = rospy.get_param("/outbound/slots")
  slots_collection.insert_many(slots_param)

  groups_collection.delete_many({})
  groups_param = rospy.get_param("/outbound/slots_group")
  groups_collection.insert_many(groups_param)

  try:
    param_name = rospy.get_param_names()
  except ROSException:
    print("could not get param name")

  objects_collection.delete_many({})
  objects_param = rospy.get_param("/manipulation_objects")
  objects_collection.insert_one(objects_param)

#  you can save picks and places param only if you already
#  runned initial_interface and saved

  picks_collection.delete_many({})
  picks_param = rospy.get_param("/picks_param")
  picks_collection.insert_many(picks_param)

  places_collection.delete_many({})
  places_param = rospy.get_param("/places_param")
  places_collection.insert_many(places_param)




