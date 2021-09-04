#!/usr/bin/env python2

import pymongo
import pprint
import rospy

from std_msgs.msg import String
from pymongo import MongoClient
from manipulation_interface_mongo.srv import SaveParam

def callback(data):
	return data.data

def save_components_params(srv):
	go_to_param = rospy.get_param("/go_to_location")
	if go_to_param:
		go_to_collection.delete_many({})
		go_to_collection.insert_many(go_to_param)

	boxes_param = rospy.get_param("/inbound/boxes")
	if boxes_param:
		boxes_collection.delete_many({})
		boxes_collection.insert_many(boxes_param)

	go_to_param = rospy.get_param("/go_to_location")
	if go_to_param:
		go_to_collection.delete_many({})
		go_to_collection.insert_many(go_to_param)

	groups_param = rospy.get_param("/outbound/slots_group")
	if groups_param:
		groups_collection.delete_many({})
		groups_collection.insert_many(groups_param)

	objects_param = rospy.get_param("/manipulation/objects")
	if objects_param:
		objects_collection.delete_many({})
		objects_collection.insert_many(objects_param)

	slots_param = rospy.get_param("/outbound/slots")
	if slots_param:
		slots_collection.delete_many({})
		slots_collection.insert_many(slots_param)

	print('components params are saved')
	return 'true'

def save_actions_params(srv):
	tasks_param = rospy.get_param("/multi_skills/tasks")
	if tasks_param:
		tasks_collection.delete_many({})
		tasks_collection.insert_many(tasks_param)

	print('actions params are saved')
	return 'true'

def save_recipe_param(srv):
	recipes_param = rospy.get_param("/multi_skills/recipes")
	if recipes_param:
		recipes_collection.delete_many({})
		recipes_collection.insert_many(recipes_param)

	print('recipe param is saved')
	return 'true'

def save_server():
	s = rospy.Service('/save_components_params_on_mongo', SaveParam, save_components_params)
	s = rospy.Service('/save_actions_param_on_mongo', SaveParam, save_actions_params)
	s = rospy.Service('/save_recipe_param_on_mongo', SaveParam, save_recipe_param)
	print("Ready to save new param.")
	rospy.spin()

client = MongoClient('localhost', 27017)

db = client.interface_data

boxes_collection           = db.boxes
go_to_collection           = db.go_to_locations
groups_collection          = db.groups
objects_collection         = db.objects
recipes_collection         = db.recipes
slots_collection           = db.slots
tasks_collection           = db.tasks_properties

if __name__ == '__main__':
	rospy.init_node('prova', anonymous=True)

	boxes_mongo   = list(boxes_collection.find())
	go_to_mongo   = list(go_to_collection.find())
	groups_mongo  = list(groups_collection.find())
	objects_mongo = list(objects_collection.find())
	recipes_mongo = list(recipes_collection.find())
	slots_mongo   = list(slots_collection.find())
	tasks_mongo   = list(tasks_collection.find())

	boxes = []
	for data in boxes_mongo:
		del data['_id']
		boxes.append( data )

	rospy.set_param("/inbound/boxes", boxes)

	go_to_locations = []
	for data in go_to_mongo:
		del data['_id']
		go_to_locations.append( data )

	rospy.set_param("/go_to_location", go_to_locations)

	groups = []
	for data in groups_mongo:
		del data['_id']
		groups.append( data )

	rospy.set_param("/outbound/slots_group", groups)

	objects = []
	for data in objects_mongo:
		del data['_id']
		objects.append( data )

	rospy.set_param("/manipulation_object_types", objects)

#	objects = []
#	for data in objects_mongo:
#		str = '/manipulation_object_types/'
#		str = str + data['name']
#		del data['name']
#		rospy.set_param(str, data)

	recipes = []
	for data in recipes_mongo:
		del data['_id']
		recipes.append( data )

	rospy.set_param("/multi_skills/recipes", recipes)

	slots = []
	for data in slots_mongo:
		del data['_id']
		slots.append( data )

	rospy.set_param("/outbound/slots", slots)

	tasks = []
	for data in tasks_mongo:
		del data['_id']
		tasks.append( data )

	rospy.set_param("/multi_skills/tasks", tasks)

	save_server()
