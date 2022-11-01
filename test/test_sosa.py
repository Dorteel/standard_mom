from rdflib import Graph, Literal, RDF, URIRef
from rdflib.namespace import SOSA

# Create an empty Graph
sensorGraph = Graph()

locobot = URIRef("http://example.org/locobot")
locobotCamera = URIRef("http://example.org/camera")

# Add observation
sensorGraph.add((locobotCamera, RDF.type, SOSA.Actuator))
sensorGraph.add((locobot, SOSA.madeByActuator, locobotCamera))

camera = URIRef(SOSA.Sensor)
print(camera)
# Print out the entire Graph in the RDF Turtle format
#print(sosaGraph.serialize(format="turtle"))

#print(sensorGraph.serialize(format="turtle"))