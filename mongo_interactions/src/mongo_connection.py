#!/usr/bin/env python3

import pymongo
import pprint
import rospy

from std_msgs.msg import String
from pymongo import MongoClient
from mongo_interactions.srv import SaveParam

def callback(data):
  return data.data

def save_initial_param(srv):
  go_to_collection.delete_many({})
  go_to_param = rospy.get_param("/go_to_location")
  go_to_collection.insert_many(go_to_param)

  boxes_collection.delete_many({})
  boxes_param = rospy.get_param("/inbound/boxes")
  boxes_collection.insert_many(boxes_param)

  picks_collection.delete_many({})
  picks_param = rospy.get_param("/picks_param")
  picks_collection.insert_many(picks_param)

  places_collection.delete_many({})
  places_param = rospy.get_param("/places_param")
  places_collection.insert_many(places_param)

  slots_collection.delete_many({})
  slots_param = rospy.get_param("/outbound/slots")
  slots_collection.insert_many(slots_param)

  groups_collection.delete_many({})
  groups_param = rospy.get_param("/outbound/slots_group")
  groups_collection.insert_many(groups_param)

  objects_collection.delete_many({})
  objects_param = rospy.get_param("/manipulation_objects")
  objects_collection.insert_one(objects_param)

  print('initial param are saved')
  return 'true'

def save_recipe_param(srv):
  multi_skills_collection.delete_many({})
  multi_skills_param = rospy.get_param("/multi_skills")
  multi_skills_collection.insert_one(multi_skills_param)

  print('recipe param are saved')
  return 'true'

def save_server():
  s = rospy.Service('/save_initial_param_on_mongo', SaveParam, save_initial_param)
  s = rospy.Service('/save_recipe_param_on_mongo', SaveParam, save_recipe_param)
  print("Ready to save new param.")
  rospy.spin()

client = MongoClient('localhost', 27017)

db = client.interface_data

go_to_collection           = db.go_to_locations
boxes_collection           = db.boxes
picks_collection           = db.picks
places_collection          = db.places
slots_collection           = db.slots
groups_collection          = db.groups
go_to_server_collection    = db.go_to_location_server
outbound_server_collection = db.outbound_place_server
inbound_server_collection  = db.inbound_pick_server
multi_skills_collection    = db.multi_skills
objects_collection         = db.objects

if __name__ == '__main__':
  rospy.init_node('prova', anonymous=True)

  go_to_mongo     = list(go_to_collection.find())
  boxes_mongo     = list(boxes_collection.find())
  picks_mongo     = list(picks_collection.find())
  places_mongo    = list(places_collection.find())
  slots_mongo     = list(slots_collection.find())
  groups_mongo    = list(groups_collection.find())
  go_to_server_mongo    = list(go_to_server_collection.find())
  outbound_server_mongo =	list(outbound_server_collection.find())
  multi_skills_mongo    =	list(multi_skills_collection.find())
  inbound_server_mongo  =	list(inbound_server_collection.find())
  objects_mongo         = list(objects_collection.find())

  go_to_locations = []
  for data in go_to_mongo:
    del data['_id']
    go_to_locations.append( data )
    rospy.set_param("/go_to_location", go_to_locations)

  boxes = []
  for data in boxes_mongo:
    del data['_id']
    boxes.append( data )
    rospy.set_param("/inbound/boxes", boxes)

  picks = []
  for data in picks_mongo:
    del data['_id']
    picks.append( data )
    rospy.set_param("/picks_param", picks)

  places = []
  for data in places_mongo:
    del data['_id']
    places.append( data )
    rospy.set_param("/places_param", places)

  slots = []
  for data in slots_mongo:
    del data['_id']
    slots.append( data )
    rospy.set_param("/outbound/slots", slots)

  groups = []
  for data in groups_mongo:
    del data['_id']
    groups.append( data )
    rospy.set_param("/outbound/slots_group", groups)

  if go_to_server_mongo != []:
    del go_to_server_mongo[0]['_id']
    for key in go_to_server_mongo[0]:
      if type(go_to_server_mongo[0][key]) == 'dict':
        for key_ in go_to_server_mongo[0][key]:
          rospy.set_param("/go_to_location_server/"+key+"/"+key_, go_to_server_mongo[0][key][key_])
      else:
        rospy.set_param("/go_to_location_server/"+key, go_to_server_mongo[0][key])

  if outbound_server_mongo != []:
    del outbound_server_mongo[0]['_id']
    for key in outbound_server_mongo[0]:
      if type(outbound_server_mongo[0][key]) == 'dict':
        for key_ in outbound_server_mongo[0][key]:
          rospy.set_param("/outbound_place_server/"+key+"/"+key_, outbound_server_mongo[0][key][key_])
      else:
        rospy.set_param("/outbound_place_server/"+key, outbound_server_mongo[0][key])

  if inbound_server_mongo != []:
    del inbound_server_mongo[0]['_id']
    for key in inbound_server_mongo[0]:
      if type(inbound_server_mongo[0][key]) == 'dict':
        for key_ in inbound_server_mongo[0][key]:
          rospy.set_param("/inbound_pick_server/"+key+"/"+key_, inbound_server_mongo[0][key][key_])
      else:
        rospy.set_param("/inbound_pick_server/"+key, inbound_server_mongo[0][key])

  if objects_mongo != []:
    del objects_mongo[0]['_id']
    for key in objects_mongo[0]:
      if type(objects_mongo[0][key]) == 'dict':
        for key_ in objects_mongo[0][key]:
          if type(objects_mongo[0][key][key_]) == 'dict':
            for key__ in objects_mongo[0][key][key_]:
              rospy.set_param("/manipulation_objects/"+key+"/"+key_+"/"+key__, objects_mongo[0][key][key_][key__])
          else:
            rospy.set_param("/manipulation_objects/"+key+"/"+key_, objects_mongo[0][key][key_])
      else:
        rospy.set_param("/manipulation_objects/"+key, objects_mongo[0][key])

  if multi_skills_mongo != []:
    del multi_skills_mongo[0]['_id']
    for key in multi_skills_mongo[0]:
      if type(multi_skills_mongo[0][key]) == 'dict':
        for key_ in multi_skills_mongo[0][key]:
          if type(multi_skills_mongo[0][key][key_]) == 'dict':
            for key__ in multi_skills_mongo[0][key][key_]:
              rospy.set_param("/multi_skills/"+key+"/"+key_+"/"+key__, multi_skills_mongo[0][key][key_][key__])
          else:
            rospy.set_param("/multi_skills/"+key+"/"+key_, multi_skills_mongo[0][key][key_])
      else:
        rospy.set_param("/multi_skills/"+key, multi_skills_mongo[0][key])

        save_server()
