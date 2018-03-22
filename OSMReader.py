"""
Author: Sofiane

"""
import json
import networkx as nx
from collections import defaultdict, Counter
from methods import methods
import fiona

class OSMReader():
	def __init__(self):
		pass

	@staticmethod
	def build_road_network_from_overpass_output(fname):
		"""
		This function builds a road graph from road geojson file.
		We are excluding some ways that are not roads. See definitions at this link: http://wiki.openstreetmap.org/wiki/Key:highway
		I assume we are interested in a limited set of properties: one_way, maxspeed, lanes.
		# TODO: Infer angle info from previous/next
		:param fname: the road shape file of the city
		:return: a networkx graph, way_to_nodes dict, and way_to_properties
		"""

		sh = json.load(open(fname))
		try:
			features = sh['features']
		except KeyError:
			print 'weired file format. Cannot find features.'
			raise

		g = nx.DiGraph()
		line_points = defaultdict(list)
		line_properties = dict()
		for obj in features:
			properties = obj['properties']
			# Exclude roads that are not for cars.
			if properties['highway'] not in ['motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified',
			                                'residential', 'service', 'motorway_link', 'trunk_link', 'primary_link',
			                                 'secondary_link', 'tertiary_link' ]:
				continue

			# Check the properties we are interested in.
			line_properties[properties['@id']] = {'oneway': properties.get('oneway', None),
			                                      'maxspeed': properties.get('maxspeed', None),
			                                      'lanes': properties.get('lanes', None)}

			# Build the road directed graph.
			path = [tuple(node) for node in obj['geometry']['coordinates']]
			line_points[properties['@id']] = path
			g.add_path(path)
			for i in range(len(path) - 1):
				g[path[i]][path[i+1]]['oneway'] = properties.get('oneway', None)
				g[path[i]][path[i+1]]['maxspeed'] = properties.get('maxspeed', None)
				g[path[i]][path[i+1]]['lanes'] = properties.get('lanes', None)
				# infer angle for middle nodes
				if i > 0:
					g[path[i]]['angle'] = methods.calculate_bearing(latitude_1=path[i-1][1], longitude_1=path[i-1][0],
					                                                latitude_2=path[i+1][1], longitude_2=path[i+1][0])
			if 'oneway' in properties and properties['oneway'] == 0:
				path.reverse()
				g.add_path(path)
				for i in range(len(path) - 1):
					g[path[i]][path[i + 1]]['oneway'] = properties.get('oneway', None)
					g[path[i]][path[i + 1]]['maxspeed'] = properties.get('maxspeed', None)
					g[path[i]][path[i + 1]]['lanes'] = properties.get('lanes', None)
		return g, line_points, line_properties


	@staticmethod
	def build_road_network_from_osm_shapefile(shape_file):
		"""
		This function builds a road graph of a city from its road shapefile.
		The idea is to create an edge for each consecutive nodes in each path.
		 Use fiona to read the shape file.
		:param shape_file: the road shape file of the city
		:return: a graph
		"""
		g = nx.DiGraph()
		sh = fiona.open(shape_file)
		for obj in sh:
			maxspeed = obj['properties'].get('maxspeed', None)
			lanes = obj['properties'].get('lanes', None)
			oneway = obj['properties'].get('oneway', None)
			path = obj['geometry']['coordinates']
			for i in range(1, len(path)):
				g.add_edge(path[i-1], path[i])
				g.node[path[i-1]] = {'lat': path[i-1][1], 'lon': path[i-1][0], 'maxspeed':maxspeed, 'lanes':lanes, 'oneway': oneway}
			g.node[path[-1]] = {'lat': path[-1][1], 'lon': path[-1][0], 'maxspeed': maxspeed, 'lanes': lanes, 'oneway': oneway}
			if oneway:
				path.reverse()
				g.add_path(path)
		return g

