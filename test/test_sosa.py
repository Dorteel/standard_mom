from rdflib import Graph, Literal, RDF, URIRef, Namespace
from rdflib.namespace import SOSA
import os
# Create an empty Graph
sensorGraph = Graph()

locobot = Namespace("http://example.org/locobot/")

#locobot = URIRef("http://example.org/locobot")
locobotCamera = URIRef("http://example.org/camera")

testDetection = [{'name': 'cluster_1', 'position': [0.3225392010185295, 0.2640586788732463, -0.0406332993591656], 'yaw': 0, 'color': [221.2, 210.60000000000002, 207.4], 'num_points': 435.40000000000003}, {'name': 'cluster_2', 'position': [0.37995929638788767, 0.08580552762115529, -0.0539108581287438], 'yaw': 0, 'color': [55.4, 52.199999999999996, 47.4], 'num_points': 569.6}]
sensorGraph.add
# Add observation
# sensorGraph.add((locobotCamera, RDF.type, SOSA.Actuator))
# sensorGraph.add((locobot, SOSA.madeByActuator, locobotCamera))

camera = URIRef(SOSA.Sensor)
newGraph = Graph().add((locobotCamera, RDF.type, SOSA.Sensor))
newGraph.serialize(destination="test.ttl")
print(locobot.waist)

g = Graph()
g.parse("http://www.w3.org/People/Berners-Lee/card")
g.serialize(destination="/home/user/locobot_ws/src/standard_model_of_mind/standard_mom/test/tbl.ttl")
# print("file saved!")
# print(os.getcwd())
# Print out the entire Graph in the RDF Turtle format
#print(sosaGraph.serialize(format="turtle"))

#print(sensorGraph.serialize(format="turtle"))
oro = Graph().parse("http://kb.openrobots.org/")
print(len(oro))
