#!/usr/bin/env python3
# ======================================================
# This file tests the queries that need to be run
# in order to access the declarative memory of the robot
# ======================================================
from rdflib.plugins.sparql import prepareQuery
from rdflib import Graph, URIRef, Namespace, Literal
from rdflib.namespace import RDFS, XSD
# Create a Graph, add in some test data
g = Graph()
cup = URIRef("http://example.org/cup/")
knowrob = Namespace("http://knowrob.org/kb/knowrob.owl#")

g.parse('memory.ttl')
g.add((cup, RDFS.label, Literal('cup')))
g.add((cup, knowrob.hasAffordance, knowrob.GraspingAffordance))
g.serialize(destination="memory.ttl")


m = """
PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
SELECT ?label WHERE 
{   ?object knowrob:hasAffordance knowrob:GraspingAffordance .
    ?object rdfs:label ?label .
}"""
obj = 'cup'
query = """
            PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
            ASK 
            {   ?object knowrob:hasAffordance knowrob:GraspingAffordance .
                ?object rdfs:label '""" + obj + """' .
            }"""

qres = g.query(query)

# qres = g.query("""
# PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
# SELECT ?label ?object WHERE 
# {   ?object <http://knowrob.org/kb/knowrob.owl#hasAffordance> <http://knowrob.org/kb/knowrob.owl#GraspingAffordance> .
#     ?object rdfs:label ?label
# }""")

# q = prepareQuery(
#     "SELECT ?s WHERE { ?person knowrob:knows ?s .}",
#     initNs = { "knowrob": knowrob }
# )

print(''.join([str(item) for item in qres]))
    #print(f"{row.s}")

# g.parse(
#     data="""
#         <x:> a <c:> .
#         <y:> a <c:> .
#         <z:> a <c:> .
#         <z:> h <c:> .
#     """,
#     format="turtle"
# )

# Select all the things (s) that are of type (rdf:type) c:
# qres = g.query("""SELECT ?object WHERE { ?object h <c:> }""")


# for row in qres:
#     print(f"{row.object}")
#     #print(f"{row.s}")


# for row in qres:
#     print(f"{row.s}")
# # prints:
# # x:
# # y:

# # Add in a new triple using SPATQL UPDATE
# g.update("""INSERT DATA { <z:> a <c:> }""")

# # Select all the things (s) that are of type (rdf:type) c:
# qres = g.query("""SELECT ?s WHERE { ?s a <c:> }""")

# print("After update:")
# for row in qres:
#     print(f"{row.s}")
# # prints:
# # x:
# # y:
# # z:

# # Change type of <y:> from <c:> to <d:>
# g.update("""
#          DELETE { <y:> a <c:> }
#          INSERT { <y:> a <d:> }
#          WHERE { <y:> a <c:> }
#          """)
# print("After second update:")
# qres = g.query("""SELECT ?s ?o WHERE { ?s a ?o }""")
# for row in qres:
#     print(f"{row.s} a {row.o}")
# # prints:
# # x: a c:
# # z: a c:
# # y: a d:

