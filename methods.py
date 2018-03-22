"Author: Sofiane"

"""
Class of all static methods needed in the project.
"""

from matplotlib import collections as mc
from scipy.spatial import cKDTree
from collections import defaultdict
import operator
import geopy
import geopy.distance
import json
import time
import datetime
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from geojson import MultiLineString
import geojson
import math
from networkx import betweenness_centrality
import networkx as nx

class GpsPoint:
	def __init__(self, vehicule_id=None, lon=None, lat=None, speed=None, timestamp=None, angle=None, traj_id=None):
			self.vehicule_id = int(vehicule_id) if vehicule_id != None else 0
			self.speed = float(speed) if speed != None else 0.0
			if timestamp != None:
				self.timestamp = datetime.datetime.strptime(timestamp, '%Y-%m-%d %H:%M:%S+03')
			self.lon = float(lon)
			self.lat = float(lat)
			self.angle = float(angle)
			if traj_id != None:
				self.traj_id = traj_id

	def get_coordinates(self):
		"""
		return the lon,lat of a gps point
		:return: tuple (lon, lat)
		"""
		return (self.lat, self.lon)

	def get_lonlat(self):
		return (self.lon, self.lat)

	def set_traj_id(self, traj_id):
		self.traj_id = traj_id

	def __str__(self):
		return "bt_id:%s, speed:%s, timestamp:%s, lon:%s, lat:%s, angle:%s" % \
			   (self.vehicule_id, self.speed, self.timestamp, self.lon, self.lat, self.angle)

	def __repr__(self):
		return "bt_id:%s, speed:%s, timestamp:%s, lon:%s, lat:%s, angle:%s" % \
			   (self.vehicule_id, self.speed, self.timestamp, self.lon, self.lat, self.angle)


class methods:
	def __init__(self,):
		pass

	@staticmethod
	def draw_simple_roadnet(rn, showNodes=False):
		lines = [[s, t] for s, t in rn.edges()]
		lc = mc.LineCollection(lines, colors='black', linewidths=2)
		fig, ax = plt.subplots(facecolor='black', figsize=(14, 10))
		ax.add_collection(lc)
		ax.autoscale()
		if showNodes == True:
			plt.scatter([node[0] for node in rn.nodes()], [node[1] for node in rn.nodes()], s=30)
		plt.show()

	@staticmethod
	def draw_roadnet_with_edge_features(rn, discriminating_feature='maxspeed', showNodes=False):
		lines = [[s, t] for s, t in rn.edges()]
		colors = ['black' if rn[s][t][discriminating_feature] is not None else 'red' for s, t in rn.edges()]
		lc = mc.LineCollection(lines, colors=colors, linewidths=2)
		fig, ax = plt.subplots(facecolor='black', figsize=(14, 10))
		ax.add_collection(lc)
		ax.autoscale()
		if showNodes == True:
			plt.scatter([node[0] for node in rn.nodes()], [node[1] for node in rn.nodes()], s=30)
		plt.show()

	@staticmethod
	def is_point_in_bboxes(point, bboxes):
		"""
		Check if a point (lon, lat) is within a set of bboxes [[max_lon, max_lat, min_lon, min_lat]]
		:param point: lon, lat
		:param bboxes: [[max_lon, max_lat, min_lon, min_lat]]
		:return: Boolean
		"""

		# Case where bboxes are not defines, accept all points. Always return True
		if bboxes == None:
			return True

		lon, lat = point
		for bbox in bboxes:
			if lon <= bbox[0] + 0.002 and lon >= bbox[2] - 0.002 and lat <= bbox[1] + 0.002 and lat >= bbox[3] - 0.02:
				return True
		return False

	@staticmethod
	def load_data(fname='data/gps_data/gps_points.csv', BBOXES=None):
		"""
		Given a file that contains gps points, this method creates different data structures
		:param fname: the name of the input file. format: speed,date_time,angle,lon,lat
		:return: data_points (list of gps positions with their metadata), raw_points (coordinates only),
		points_tree is the KDTree structure to enable searching the points space
		"""
		data_points = list()
		raw_points = list()

		with open(fname, 'r') as f:
			f.readline()
			for line in f:
				if len(line) < 10:
					continue
				vehicule_id, timestamp, lat, lon, speed, angle = line.split(',')
				if not methods.is_point_in_bboxes((float(lon), float(lat)), BBOXES):
					continue
				pt = GpsPoint(vehicule_id=vehicule_id, timestamp=timestamp, lat=lat, lon=lon, speed=speed, angle=angle)
				data_points.append(pt)
				raw_points.append(pt.get_coordinates())
		points_tree = cKDTree(raw_points)
		return np.array(data_points), np.array(raw_points), points_tree


	@staticmethod
	def create_trajectories(input_fname='data/gps_data.csv', waiting_threshold=5, BBOXES=None):
		"""
		return all trajectories.
		The heuristic is simple. Consider each users sorted traces not broken by more than 1 hour as trajectories.
		:param waiting_threshold: threshold for trajectory split expressed in seconds.
		:return: list of lists of trajectories
		"""

		data_points, raw_points, points_tree = methods.load_data(fname=input_fname, BBOXES=BBOXES)

		detections = defaultdict(list)
		for p in data_points:
			detections[p.vehicule_id].append(p)

		# compute trajectories: split detections by waiting_threshold
		trajectories = []
		for btd, ldetections in detections.iteritems():
			points = sorted(ldetections, key=operator.attrgetter('timestamp'))
			source = 0
			prev_point = 0
			i = 1
			while i < len(points):
				delta = points[i].timestamp - points[prev_point].timestamp
				if delta.days * 24 * 3600 + delta.seconds > waiting_threshold:
					trajectories.append(points[source: i])
					source = i
				prev_point = i
				i += 1
			if source < len(points):
				trajectories.append(points[source: -1])
		return trajectories


	@staticmethod
	def create_gps_stream_from_data(gps_data_fname, BBOXES=None):
		"""
		Simple function to prepare the stream of points.
		:param DATA_PATH:
		:param FILE_CODE:
		:return:
		"""
		# Generate Trajectories
		trajectories = methods.create_trajectories(input_fname=gps_data_fname,
												   waiting_threshold=21, BBOXES=BBOXES)
		# Sort trajectories into a stream of points
		building_trajectories = dict()
		gps_point_stream = []
		for i, trajectory in enumerate(trajectories):
			for point in trajectory:
				point.set_traj_id(i)
				gps_point_stream.append(point)
		gps_point_stream = sorted(gps_point_stream, key=operator.attrgetter('timestamp'))
		return gps_point_stream


	@staticmethod
	def assign_gps_points_to_osm_clusters(osm_rn, gps_points, radius_meters=5):
		"""
		Right now, angle is not taken into account.
		:param osm_rn:
		:param gps_points:
		:return:
		"""
		assignment = defaultdict(list)
		kd_index = cKDTree([c for c in osm_rn.nodes()])
		RADIUS_DEGREES = radius_meters * 10e-6
		clusters = osm_rn.nodes()
		for point in gps_points:
			nearest_cluster_indices = [clu_index for clu_index in
									   kd_index.query_ball_point(x=point.get_lonlat(), r=RADIUS_DEGREES, p=2)]
			if len(nearest_cluster_indices) == 0:
				# The gps point doesn't match any of the OSM clusters
				continue
			pt = geopy.Point(point.get_coordinates())
			close_clusters_distances = [geopy.distance.distance(pt, geopy.Point(clusters[clu_index])).meters
										for clu_index in nearest_cluster_indices]
			closest_cluster_indx = nearest_cluster_indices[close_clusters_distances.index(min(close_clusters_distances))]
			assignment[closest_cluster_indx].append(point)

			# TODO: case in which we account for angles.
			# nearest_cluster_indices = [clu_index for clu_index in
			 #                       kd_index.query_ball_point(x=point.get_lonlat(), r=RADIUS_DEGREES, p=2)
			 #                       if math.fabs(
			# 		diffangles(point.angle, kd_index[clu_index].angle)) <= HEADING_ANGLE_TOLERANCE]

		return assignment

	@staticmethod
	def calculate_bearing(latitude_1, longitude_1, latitude_2, longitude_2):
		"""
		Got it from this link: http://pastebin.com/JbhWKJ5m
	   Calculation of direction between two geographical points
	   """
		rlat1 = math.radians(latitude_1)
		rlat2 = math.radians(latitude_2)
		rlon1 = math.radians(longitude_1)
		rlon2 = math.radians(longitude_2)
		drlon = rlon2 - rlon1

		b = math.atan2(math.sin(drlon) * math.cos(rlat2), math.cos(rlat1) * math.sin(rlat2) -
					   math.sin(rlat1) * math.cos(rlat2) * math.cos(drlon))
		return (math.degrees(b) + 360) % 360

	@staticmethod
	def save_edgeFile_into_geojson(edgeFile, outFile):
		geojson = {'type': 'FeatureCollection', 'features': []}
		with open(edgeFile, 'r') as f:
			for incId, line in enumerate(f):
				slon, slat, tlon, tlat, status = line.split(' ')
				feature = {'type': 'Feature', 'properties': {}, 'geometry': {'type': 'LineString', 'coordinates': []}}
				feature['geometry']['coordinates'] = [(float(slon), float(slat)),(float(tlon), float(tlat))]
				feature['properties']['id'] = incId
				feature['properties']['status'] = int(status)
				geojson['features'].append(feature)
		json.dump(geojson, open(outFile, 'w'))

	@staticmethod
	def save_edgeFile_newEdges_into_geojson(edgeFile, outFile):
		"""
		This method only saves newly detected segments.
		:param edgeFile:
		:param outFile:
		:return:
		"""

		geojson_obj = {'type': 'FeatureCollection', 'features': []}
		with open(edgeFile, 'r') as f:
			for incId, line in enumerate(f):
				slon, slat, tlon, tlat, status = line.split(' ')
				if int(status) != 1:
					continue
				print int(status)
				feature = {'type': 'Feature', 'properties': {}, 'geometry': {'type': 'LineString', 'coordinates': []}}
				feature['geometry']['coordinates'] = [(float(slon), float(slat)),(float(tlon), float(tlat))]
				feature['properties']['id'] = incId
				feature['properties']['status'] = int(status)
				geojson_obj['features'].append(feature)
		json.dump(geojson_obj, open(outFile, 'w'))

	@staticmethod
	def draw_fused_map(fname, showNodes=False):
		"""
		This method plots the resulting map of the fusion process.
		:param fname: path to the geojson file generated by QMapFuse.py
		:param showNodes: boolean for whether to show nodes on the map or not.
		:return: nothing.
		"""
		obj = geojson.load(open(fname))
		colors_dict = {-1: 'black', 0: 'green', 1: 'red'}
		colors = []
		lines = []
		nodes = set()
		for e in obj['features']:
			lines.append(e['geometry']['coordinates'])
			colors.append(colors_dict[e['properties']['status']])
			nodes.add(tuple(e['geometry']['coordinates'][0]))
			nodes.add(tuple(e['geometry']['coordinates'][1]))
		# lines = [[s, t] for s, t in rn.edges()]
		lc = mc.LineCollection(lines, colors=colors, linewidths=1)
		fig, ax = plt.subplots(facecolor='black', figsize=(14, 10))
		ax.add_collection(lc)
		ax.autoscale()
		if showNodes == True:
			plt.scatter([node[0] for node in nodes], [node[1] for node in nodes], s=2)
		plt.show()


"""
In the following, we list the methods created by Rade for QMapFusion
"""


theta = 50.0

def geodist(point1, point2):
	return(np.sqrt((111257*(point1[0]-point2[0]))**2+(70197*(point1[1]-point2[1]))**2)) #180dg difference equivalent to 80m difference

def taxidistkm(point1, point2):
	return(1.11257*np.abs(point1[0]-point2[0])+0.70197*np.abs(point1[1]-point2[1])+ 50.0/180/100000*angledist(point2[2],point1[2])) #180dg difference equivalent to 80m difference

def distcolinearity(point1, point2):
	AA = anglebetweentwopoints(point1, point2)
	return((1.11257*np.abs(point1[0]-point2[0])+0.70197*np.abs(point1[1]-point2[1]))*(angledist(point1[2],AA)+angledist(point2[2],AA)))

def taxidistwpenalty(point1, point2,penalty):
	return(0.70197*np.abs(point1[0]-point2[0])+0.70197*np.abs(point1[1]-point2[1])+ penalty/180/100000.0*angledist(point2[2],point1[2])) #180dg difference equivalent to 80m difference

def dist2seed(point1, point2):
	AA = anglebetweentwopoints(point1, point2)
	anglecost = min(angledist(point1[2],AA)+angledist(point2[2],AA), angledist(point1[2],180+AA)+angledist(point2[2],180+AA))
	return(111257*np.abs(point1[0]-point2[0])+70197*np.abs(point1[1]-point2[1])+ 100.0/180*max(0,anglecost-30))

def getdata_all_attributes(nsamples, datestart, datestr, datapath):
	datapointwts = []
	lats = []
	lons = []
	j = 0
	with open(datapath,'rb') as f:
		for line in f:
			j = j+1
			if j > nsamples:
				break
			line = line[:-1].decode('ascii', 'ignore')
			zz = line.split(",")
			if zz[3][:10] < datestr and zz[3][:10] >= datestart:
				ts = time.mktime(datetime.datetime.strptime(zz[3], "%Y-%m-%d %H:%M:%S").timetuple())
				LL = (float(zz[5][:8]), float(zz[6][:8]))
				angle = float(zz[4])-180
				speed = float(zz[2])
				lats.append(LL[0])
				lons.append(LL[1])
				pointwts = (LL[0], LL[1], angle, speed, j, ts)
				datapointwts.append(pointwts)
	return(datapointwts)

def getdata(nsamples, datestart, datestr, datapath):
	datapointwts = []
	lats = []
	lons = []
	j = 0
	with open(datapath,'rb') as f:
		f.readline()
		for line in f:
			j = j+1
			if j > nsamples:
				break
			line = line[:-1].decode('ascii', 'ignore')
			zz = line.split(",")
			if zz[1][:10] < datestr and zz[1][:10] >= datestart:
				ts = time.mktime(datetime.datetime.strptime(zz[1], "%Y-%m-%d %H:%M:%S").timetuple())
				LL = (float(zz[3][:8]), float(zz[4][:8]))
				angle = float(zz[2])-180
				speed = float(zz[0])
				lats.append(LL[0])
				lons.append(LL[1])
				pointwts = (LL[0], LL[1], angle, speed, j, ts)
				datapointwts.append(pointwts)
	return(datapointwts)

def greaterthanangle(alpha,beta):
	if (beta-alpha)%360<180:
		return True
	else:
		return False

def angledist(a1, a2):
	return(min(abs(a1-a2),abs((a1-a2) % 360),abs((a2-a1) % 360),abs(a2-a1)))

def anglebetweentwopoints(LL1, LL2):
	xd = (LL1[0]-LL2[0]); yd =LL1[1]-LL2[1];
#    xd = 70197/111257*(LL1[0]-LL2[0]); yd =LL1[1]-LL2[1];
	return(np.arctan2(xd,yd)*180/np.pi)

def is_power2(num):
	return num != 0 and ((num & (num - 1)) == 0)

def getseeds(datapoint,radius):
	chosen = []; seeds = [];
	random.shuffle(datapoint)
	j = 0;
	for p in datapoint:
		chosen.append(p);
	ok = 1; j = -1;
	for j,p in enumerate(chosen):
		ok = -1;
		if j<1000:
			for q in seeds:
				if taxidistkm(p,q) < radius/100000.0:
					ok = 1
					break;
			if ok <1:
				seeds.append(p)
		else:
			if j%1000 == 0:# and (is_power2(int(j/1000))):
				S = [(xx[0], xx[1]) for xx in seeds];
				nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(S)
				X = np.array([[xx[0],xx[1]] for xx in chosen[j:min(len(chosen),j+1000)]]); #X.reshape(-1, 1)
				distances, indices = nbrs.kneighbors(X)
			if distances[j%1000][0] > radius/100000.0:
				seeds.append(p)
	return (seeds)

def avgpoint(cc):
	hh = np.arctan2(sum([np.sin(xx[2] / 360.0 * 2 * np.pi) for xx in cc]), sum([np.cos(xx[2] / 360.0 * 2 * np.pi) for xx in cc])) * 180.0 / np.pi
	return((np.mean([xx[0] for xx in cc]), np.mean([xx[1] for xx in cc]), hh))

def newmeans(datapointwts, seeds):
	newseeds = []
	cost = 0
	avgspeed = []
	pointsperseed = []
	cluster, p2cluster = point2cluster(datapointwts, seeds)
	for cd in cluster:
		if len(cluster[cd])>0:
			hh = np.arctan2(sum([np.sin(xx[2]/360.0*2*np.pi) for xx in cluster[cd]]),\
							sum([np.cos(xx[2]/360.0*2*np.pi) for xx in cluster[cd]]))*180.0/np.pi
			newseeds.append((np.mean([xx[0] for xx in cluster[cd]]), np.mean([xx[1] for xx in cluster[cd]]), hh))
			hh = [xx[3] for xx in cluster[cd] if xx[3] > 0]
			if len(hh) < 1:
				hh = [0]
			avgspeed.append(np.mean(hh))
			cost = cost+sum([taxidistkm(xx, newseeds[-1]) for xx in cluster[cd]])
		else:
			newseeds.append(seeds[cd])
			avgspeed.append(0)
		pointsperseed.append(len(cluster[cd]))
	return(newseeds,cost,avgspeed,pointsperseed)

def densifykm(datapointwts):
	newpoints = []
	for ii, xx in enumerate(datapointwts):
		if ii>1:
			if datapointwts[ii-1][-1]<datapointwts[ii][-1] and datapointwts[ii-1][-1]>datapointwts[ii][-1]-11 \
					and taxidistkm(datapointwts[ii-1],datapointwts[ii]) < 0.01:
				delta = int(taxidistwpenalty(datapointwts[ii][:3], datapointwts[ii-1][:3],0)*100000.0/20)+1
				x1 = datapointwts[ii-1]
				x2 = datapointwts[ii]
				if np.abs(datapointwts[ii-1][2]-datapointwts[ii][2]) < 5:
					for jj in range(1, delta-1):
						newpoints.append(tuple([float(jj)/delta*x1[sq]+(delta-jj)/float(delta)*x2[sq] \
												for sq in range(len(x1))]))
	result = datapointwts+newpoints
	result.sort(key=lambda x: x[-2],reverse=False)
	return(result)

def densify(datapointwts):
	newpoints = []
	for ii, xx in enumerate(datapointwts):
		if ii>1:
			if datapointwts[ii-1][-1]<datapointwts[ii][-1] and datapointwts[ii-1][-1]>datapointwts[ii][-1]-11 \
					and taxidist(datapointwts[ii-1],datapointwts[ii]) < 200:
				delta = int(taxidist(datapointwts[ii][:3], datapointwts[ii-1][:3])/20.0)+1
				x1 = datapointwts[ii-1]
				x2 = datapointwts[ii]
				if np.abs(datapointwts[ii-1][2]-datapointwts[ii][2]) < 10:
					for jj in range(1, delta-1):
						newpoints.append(tuple([float(jj)/delta*x1[sq]+float(delta-jj)/delta*x2[sq] \
												for sq in range(len(x1))]))
	result = datapointwts+newpoints
	result.sort(key=lambda x: x[-2],reverse=False)
	return (result)


def getpossibleedges(datapointwts,seeds):
	X = [(xx[0], xx[1]) for xx in datapointwts]
	S = [(xx[0], xx[1]) for xx in seeds]
	cluster = {}
	p2cluster = []
	gedges = {}
	gedges1 = {}
	nedges = {}
	nbrs = NearestNeighbors(n_neighbors=5, algorithm='ball_tree').fit(S)
	distances, indices = nbrs.kneighbors(X)
	for cd in range(len(seeds)):
		cluster[cd] = []
	for ii, ll in enumerate(indices):
		dd = [taxidistkm(seeds[xx], datapointwts[ii][:-1]) for xx in ll]
		cd = ll[dd.index(min(dd))]
		cluster[cd].append(datapointwts[ii])
		p2cluster.append(cd)
	for ii, xx in enumerate(datapointwts):
		if ii>1:
			if datapointwts[ii-1][-1]<datapointwts[ii][-1] and datapointwts[ii-1][-1]>datapointwts[ii][-1]-11:
				cd1 = p2cluster[ii-1]; cd2 = p2cluster[ii]
			if not cd1== cd2:
				gedges1[(cd1,cd2)] =  gedges1.get((cd1,cd2),0)+1
	return(gedges1)

def coocurematrix(datapointwts,seeds):
	startcoocurence = time.time();    gedges1 = {}; std = {}
	cluster, p2cluster = point2cluster(datapointwts, seeds)
	seedweight = [len([ yy for yy in cluster[xx] if yy[3]>0]) for xx in cluster]
	for ii in range(len(seeds)):
		mang = seeds[ii][-1]
		if len(cluster[ii])>50:
			std[ii] = np.percentile([angledist(xx[2], mang) for xx in cluster[ii]], 90)
		else:
			std[ii] = np.max([20]+[angledist(xx[2], mang) for xx in cluster[ii]])
	for ii, xx in enumerate(datapointwts):
		if ii>1:
			if datapointwts[ii-1][-1]<=datapointwts[ii][-1] and datapointwts[ii-1][-1]>=datapointwts[ii][-1]-21 and taxidistkm(datapointwts[ii-1],datapointwts[ii])<0.01:
				cd1 = p2cluster[ii-1]; cd2 = p2cluster[ii]
				if (not cd1== cd2) and ((min(datapointwts[ii][3],datapointwts[ii-1][3])>=5)):
					AA = anglebetweentwopoints(seeds[cd1], seeds[cd2])
					if max(angledist(AA,seeds[cd1][2]),angledist(AA,seeds[cd2][2]))< 2+4*(angledist(seeds[cd1][2],seeds[cd2][2])+min(std[cd1],std[cd2])): #and relativeangle1<std[cd1] and relativeangle2<std[cd2] and:
						gedges1[(cd1, cd2)] =  gedges1.get((cd1,cd2),0)+1
	gedges2 = {gg:gedges1[gg] for gg in gedges1}
	for gg in gedges2:
		if gg in gedges1 and (gg[1],gg[0]) in gedges1:
			if gedges1[(gg[1],gg[0])]>=gedges1[gg]:
				del gedges1[gg]
			elif gedges1[(gg[1],gg[0])]==gedges1[gg]:
				gg0 =(gg[1], gg[0])
				cd1 = gg[0]
				cd2 = gg[1]
				AA = anglebetweentwopoints(seeds[cd1], seeds[cd2])
				AArev = anglebetweentwopoints(seeds[cd2], seeds[cd1])
				if (angledist(AA, seeds[cd1][2]) + angledist(AA, seeds[cd2][2]))<(angledist(AArev, seeds[cd1][2]) + angledist(AArev, seeds[cd2][2])):
					del gedges1[gg0]
				else:
					del gedges1[gg]
		cd1 = gg[0]; cd2 = gg[1]; AA = anglebetweentwopoints(seeds[cd1], seeds[cd2])
		if int(angledist(AA, seeds[cd1][2]) + angledist(AA, seeds[cd2][2]))>180 and gg in gedges1:
			del gedges1[gg]
	gedges2 = {gg: gedges1[gg] for gg in gedges1}
	neighbors = {}
	filneigh = {}
	for gg in gedges2:
		neighbors[gg[0]] = []
		cd1 = gg[0]
		cd2 = gg[1]
	for gg in gedges2:
		neighbors[gg[0]].append(gedges1[gg])
	for ss in neighbors:
		neighbors[ss] = sorted(neighbors[ss])
	for gg in gedges2:
		hh = min(sum(neighbors.get(gg[1],[0])),sum(neighbors[gg[0]]))
		if gedges1[gg]<np.log(max(1,hh))-1:
			del gedges1[gg]
	return (gedges1)

def prunegraph(gedges,seeds):
	neighbors = {}
	for gg in gedges:
		if (gg[1], gg[0]) in gedges:
			print(gg)
	for ss in range(len(seeds)):
		neighbors[ss] = []
		if (ss, ss) in gedges:
			del gedges[(ss, ss)]
	gedges1 = dict(gedges)
	for gg in gedges1:
		neighbors[gg[0]].append(gg[1])
	depth = 5
	gedges2 = {gg:geodist(seeds[gg[0]], seeds[gg[1]]) for gg in gedges}
	gedges = {gg:geodist(seeds[gg[0]], seeds[gg[1]]) for gg in gedges}
	hopedges = []
	for dd in range(depth):
		gedges1 = dict(gedges2)
		for gg in gedges1:
			for ss in neighbors[gg[1]]:
				if not ss == gg[0]:
					gedges2[(gg[0], ss)] = min(gedges2[(gg[0],gg[1])] + gedges[(gg[1],ss)],gedges2.get((gg[0], ss),100000))
					hopedges.append((gg[0],ss))
	for gg in hopedges:
		if gg in gedges and gedges[gg]>0.8*gedges2[gg]:
			del gedges[gg]
#    neighbors = [[] for ss in range(len(seeds))];
	return (gedges)

def point2cluster(datapointwts,seeds):
	cluster = {};p2cluster = []; gedges = {}; gedges1 = {}; nedges = {}; std = {}; seeds1 = []; seedweight = [];
	X = [(111257 * xx[0], 70197 * xx[1], 50.0 / 180 * xx[2]) for xx in datapointwts];    S = [(111257 * xx[0], 70197 * xx[1], 50.0 / 180 * xx[2]) for xx in seeds];
	Xrot = [(111257 * xx[0], 70197 * xx[1], 50.0 / 180 * (xx[2]%360)) for xx in datapointwts];    Srot = [(111257 * xx[0], 70197 * xx[1], 50.0 / 180 * (xx[2]%360)) for xx in seeds];
	for cd in range(len(seeds)):
		cluster[cd] = []
	nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(S)
	distances, indices = nbrs.kneighbors(X)
	nbrsrot = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(Srot)
	distancesrot, indicesrot = nbrsrot.kneighbors(Xrot)
	for ii, ll in enumerate(indices):
		cd = indicesrot[ii][0]
		if distances[ii][0] < distancesrot[ii][0]:
			cd = indices[ii][0];
		cluster[cd].append(datapointwts[ii])
		p2cluster.append(cd)
	return(cluster,p2cluster)

def splitclusters(datapointwts,seeds):
	std = {}
	seeds1 = []
	seedweight = []
	cluster, p2cluster = point2cluster(datapointwts, seeds)
	for cl in cluster:
		mang = seeds[cl][-1];1
		if len(cluster[cl]) > 10:
			std[cl] = np.percentile([angledist(xx[2], mang) for xx in cluster[cl]], 90)
			clockwise = [xx for xx in cluster[cl] if greaterthanangle(xx[2], mang)]
			if std[cl]>10 and len(clockwise)>0 and len(clockwise)<len(cluster[cl]):
				seeds1.append(avgpoint(clockwise))
				seeds1.append(avgpoint([xx for xx in cluster[cl] if not greaterthanangle(xx[2], mang)]))
				seedweight.append(len(clockwise))
				seedweight.append(len(cluster[cl]) -len(clockwise))
			else:
				seeds1.append(seeds[cl]); seedweight.append(len(cluster[cl]))
		else:
			seeds1.append(seeds[cl]); seedweight.append(len(cluster[cl]))
	return seeds1, seedweight

def splitclustersparallel(datapointwts,seeds):
	X = [(xx[0], xx[1]) for xx in datapointwts]
	S = [(xx[0], xx[1]) for xx in seeds]
	cluster = {}
	p2cluster = []
	std = {}
	roadwidth = []
	nbrs = NearestNeighbors(n_neighbors=20, algorithm='ball_tree').fit(S)
	distances, indices = nbrs.kneighbors(X)
	for cd in range(len(seeds)):
		cluster[cd] = []; roadwidth.append(0)
	for ii, ll in enumerate(indices):
		dd = [taxidistkm(seeds[xx], datapointwts[ii][:-1]) for xx in ll]
		cd = ll[dd.index(min(dd))]
		cluster[cd].append(datapointwts[ii])
		p2cluster.append(cd)
	for cl in cluster:
		mang = seeds[cl][-1]
		scl = seeds[cl]
		if len(cluster[cl]) > 10:
			std[cl] = np.percentile([angledist(xx[2], mang) for xx in cluster[cl]], 90)
			roadwidth[cl] = 1+5*np.std([geodist(scl,xx)*np.sin(anglebetweentwopoints(scl,xx)-scl[-1])  for xx in cluster[cl]])

def printclusters(seeds):
	with open('data/tmp/clusters07112015.txt', 'w') as g:
		for pp in seeds:
			g.write("%s %s %s\n" %(pp[0],pp[1],pp[2]))

def computeclusters(datapointwts):
	datapoint = [(x[0], x[1], x[2]) for x in datapointwts]
	seeds = getseeds(datapoint, 20)
	oldcost = 100000000
	for ss in range(2):
		nseeds, cost, avgspeed, pointsperseed = newmeans(datapointwts, seeds)
		print(ss, cost)
		if float((oldcost-cost))/cost<0.0001:
			break
		seeds = nseeds
		oldcost = cost
		printclusters(seeds)
	return (seeds)

def printedges(gedges, seeds,datapointwts):
	maxspeed = [0 for xx in range(len(seeds))]
	cluster, p2cluster = point2cluster(datapointwts, seeds);
	for cd in cluster:
		maxspeed[cd] = int(np.percentile([0] + [xx[3] for xx in cluster[cd]], 90))
	with open('data/tmp/edges1130.txt', 'w') as g:
		for gg in gedges:
			g.write("%s %s %s %s %s %s %s %s\n" % (seeds[gg[0]][0], seeds[gg[0]][1], seeds[gg[0]][2], seeds[gg[1]][0],\
												   seeds[gg[1]][1], seeds[gg[1]][2], maxspeed[gg[0]], maxspeed[gg[1]]))

def readseeds():
	seeds= []
	with open('data/tmp/clusters07112015.txt','rb') as f:
		for line in f:
			line = line[:-1].decode('ascii', 'ignore')
			zz = line.split(" ")
			seeds.append((float(zz[0]),float(zz[1]),float(zz[2])))
	return(seeds)

def getedges2(datapointwts,seeds):
	indg = {}
	gedges1 = getpossibleedges(datapointwts, seeds)
	S = [(xx[0], xx[1]) for xx in seeds]
	gedges = {}
	nbrs = NearestNeighbors(n_neighbors=30, algorithm='ball_tree').fit(S)
	distances, indices = nbrs.kneighbors(S)
	for ii, ll in enumerate(indices):
		dd = [(taxidistkm(seeds[ii], seeds[xx]), xx, anglebetweentwopoints(seeds[ii],seeds[xx]),seeds[ii][2],seeds[xx][2],gedges1.get((ii,xx),0)) for xx in ll]
		dd.sort(key=lambda x: x[0]*(angledist(x[2],x[3])+angledist(x[2],x[4])),reverse=False)
		for jj in range(1,min(2,len(dd))):
			if  dd[jj][0]*(angledist(dd[jj][2], dd[jj][3]) + angledist(dd[jj][2], dd[jj][4]))< 0.04 or dd[jj][-1]>0: # 0.03 ~ 30m under 90dg or 300m udner 10dg
				gedges[(ii,dd[jj][1])] = dd[jj][-1]
				indg[dd[jj][1]] = 1
	for ii, ll in enumerate(indices):
		if indg.get(ii,0)<1:
			dd = [(taxidistkm(seeds[ii], seeds[xx]), xx, anglebetweentwopoints(seeds[xx],seeds[ii]),seeds[ii][2],seeds[xx][2],gedges1.get((xx,ii),0)) for xx in ll]
			dd.sort(key=lambda x: x[0]*(angledist(x[2],x[3])+angledist(x[2],x[4])),reverse=False)
			for jj in range(1,min(2,len(dd))):
				if dd[jj][0]*(angledist(dd[jj][2], dd[jj][3]) + angledist(dd[jj][2], dd[jj][4]))< 0.04 or dd[jj][-1]>0:
					gedges[(dd[jj][1],ii)] = dd[jj][-1]
					indg[ii] = 1
	return(gedges)

def getedges3(datapointwts,seeds):
	gedges = getedges2(datapointwts,seeds)
	gedges1 = coocurematrix(datapointwts,seeds)
	for gg in gedges1:
		if gedges1[gg]>10 and not(gg in gedges):
			gedges[gg]=gedges1[gg]
	for gg in gedges:
		gedges[gg] = max(1, gedges[gg])
	return(gedges)

def connectculdesac(gedges,seeds,datapointwts):
	cluster, p2cluster = point2cluster(datapointwts,seeds)
	indg = [0 for xx in seeds]; outdg = [0 for xx in seeds]; seedweight = [len(cluster[xx]) for xx in cluster]
	S = [(xx[0], xx[1]) for xx in seeds];  nbrs = NearestNeighbors(n_neighbors=30, algorithm='ball_tree').fit(S)
	for gg in gedges:
		indg[gg[1]] = indg[gg[1]]+1
	for gg in gedges:
		indg[gg[0]] = indg[gg[0]]+1
	distances, indices = nbrs.kneighbors(S)
	for ii, ll in enumerate(indices):
		if outdg[ii]==0 and seedweight[ii]>0:
			dd = [(taxidistkm(seeds[ii], seeds[xx]), xx, anglebetweentwopoints(seeds[ii], seeds[xx]), seeds[ii][2],   seeds[xx][2]) for xx in ll]
			dd.sort(key=lambda x: x[0] * (angledist(x[2], x[3]) + angledist(x[2], x[4])), reverse=False)
			jj = 1;
			if dd[jj][0] * (angledist(dd[jj][2], dd[jj][3]) + angledist(dd[jj][2], dd[jj][4])) < 0.04:  # 0.03 ~ 30m under 90dg or 300m udner 10dg
				gedges[(ii, dd[jj][1])] = 1
				indg[dd[jj][1]] = indg[dd[jj][1]]+1;
	for ii, ll in enumerate(indices):
		if indg[ii]==0 and seedweight[ii]>0:
			dd = [(taxidistkm(seeds[ii], seeds[xx]), xx, anglebetweentwopoints(seeds[xx], seeds[ii]), seeds[ii][2],   seeds[xx][2]) for xx in ll]
			dd.sort(key=lambda x: x[0] * (angledist(x[2], x[3]) + angledist(x[2], x[4])), reverse=False)
			jj = 1;
			if dd[jj][0] * (angledist(dd[jj][2], dd[jj][3]) + angledist(dd[jj][2], dd[jj][4])) < 0.04:  # 0.03 ~ 30m under 90dg or 300m udner 10dg
				gedges[(dd[jj][1],ii)] = 1
				outdg[dd[jj][1]] = outdg[dd[jj][1]]+1;
	return gedges

def pruneorphans(gedges,depth):
	inout = {}
	for edge in gedges:
		inout[edge[0]] = []
		inout[edge[1]] = []
	for edge in gedges:
		inout[edge[0]].append(edge)
		inout[edge[1]].append(edge)
	ok = 0; j = 0;
	while ok <depth:
		ok = ok+1;
		for node in inout:
			if len(inout[node])==1:
				eedge = inout[node][0]
				othernode = sum(eedge)-node;
				iin = inout[othernode].index(eedge)
				inout[node] = []
				inout[othernode].pop(iin)
				j = j + 1;
		gedges1 = {}
		for node in inout:
			for eedge in inout[node]:
				gedges1[eedge] = 1
	return(gedges1)

def ccw(A,B,C):
	return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D) and A!=C and A!=D and B!=C and B!=D

def prunecrossedges(gedges,seeds,seedweight):
	gedges1 = {x:gedges[x] for x in gedges};
	j = 0;
	for gg in gedges1:
		j = j+1;
		if gedges1[gg]== 0:
			for gg1 in gedges1:
				if np.abs(seeds[gg[0]][0]-seeds[gg1[0]][0])<0.01 and np.abs(seeds[gg[0]][1]-seeds[gg1[0]][1])<0.01:
					if min(seedweight[gg[0]], seedweight[gg[1]]) < 0.1 * min(seedweight[gg1[0]],seedweight[gg1[1]]):
						if intersect(seeds[gg[0]][:2],seeds[gg[1]][:2],seeds[gg1[0]][:2],seeds[gg1[1]][:2]):
							del gedges[gg];
							break
	return(gedges)

def getgeojson(gedges,seeds):
	inp = []
	for xx in gedges:
		ll1 = seeds[xx[0]]
		ll2 = seeds[xx[1]]
		inp.append([(ll1[0], ll1[1]), (ll2[0], ll2[1])])
	geojson.dump(inp, open('data/tmp/map0.geojson', 'w'))

def plotmap(seeds,gedges):
	plt.figure(figsize=(12, 8))  # in inches!
	ax = plt.gca()
	ax.get_xaxis().get_major_formatter().set_useOffset(False)
	ax.get_yaxis().get_major_formatter().set_useOffset(False)
	seedswedge = set([gg[0] for gg in gedges]+[gg[1] for gg in gedges])
	seedslat, seedslon = [seeds[xx][0] for xx in seedswedge], [seeds[xx][1] for xx in seedswedge]
	plt.scatter(seedslat, seedslon, marker='.', color='g', s=20)  # pointsperseed)
	segs = [];colsegs = []
	for gg in gedges:
		(n1,n2) = gg;
		segs.append(((seeds[n1][0], seeds[n1][1]), (seeds[n2][0], seeds[n2][1])))
		colsegs.append((1,0,0))
	for xx in seedswedge:
		gg = seeds[xx]; arl = 0.00001;
		segs.append(((gg[0], gg[1]), (gg[0] - arl * np.sin(np.pi / 180 * gg[2]),gg[1] - 0.9999 * arl * np.cos(np.pi / 180 * gg[2]) )))
		colsegs.append((0, 0, 1))
	ln_coll = matplotlib.collections.LineCollection(segs, colors=colsegs)# (rr, 0, 1 - rr))
	ax = plt.gca()
	ax.add_collection(ln_coll)
	plt.draw()
	plt.xlabel('lon')
	plt.ylabel('lat')

def anomalydetection(datapointwts,seeds):
	clusterload = {};
	X = [(xx[0], xx[1]) for xx in datapointwts];    S = [(xx[0], xx[1]) for xx in seeds];cluster = {};p2cluster = []; gedges2 = {}; gedges1 = {}; nedges = {}; seeddistance = {}; neighbors = {}; std = {};
	nbrs = NearestNeighbors(n_neighbors=20, algorithm='ball_tree').fit(S)
	distances, indices = nbrs.kneighbors(X)
	for cd in range(len(seeds)):
		cluster[cd] = []
	for ii, ll in enumerate(indices):
		dd = [taxidistkm(seeds[xx], datapointwts[ii][:-1]) for xx in ll]
		cd = ll[dd.index(min(dd))];
		cluster[cd].append(datapointwts[ii])
		p2cluster.append(cd)
		hourslot = datapointwts[ii][-1]%3600;
		clusterload[(cd,hourslot)] = clusterload.get((cd,hourslot),0)+1
	clusterloadtotal = [len(cluster[xx]) for xx,yy in enumerate(cluster)]
	plt.figure()
	plt.hist(clusterloadtotal,100)
	plt.show()


# ==============================================
# MapFuse methods
# =============================================
def taxidist(point1, point2):
	return(70197*np.abs(point1[0]-point2[0])+111256*np.abs(point1[1]-point2[1])+ theta/180*angledist(point2[2], point1[2])) #180dg difference equivalent to 80m difference

# def angledist(a1, a2):
#     return(min(abs(a1-a2),abs((a1-a2) % 360),abs((a2-a1) % 360),abs(a2-a1)))

def dist(point1, point2):
	return (np.sqrt((70197 * (point1[0] - point2[0])) ** 2 + (111256 * (point1[1] - point2[1])) ** 2))  # 180dg difference equivalent to 80m difference

def getosmmap(base_map):
	G, nodelatlonlanes, neighbors = read_osm_wtype_sa(base_map)
	seeds = [(nodelatlonlanes[ii][1], nodelatlonlanes[ii][0], nodelatlonlanes[ii][2]) \
			 for ii in range(len(nodelatlonlanes))]
	return (seeds, neighbors)

def getourmap():
	seeddict = {}
	seeds = []
	seed2label = {}
	neighbors = {}
	fedge = 'data/tmp/edges1130.txt'
	with open(fedge, 'rb') as f:
		for line in f:
			line = line[:-1].decode('ascii', 'ignore')
			zz = line.split(" ")
			s1 = (float(zz[0]), float(zz[1]), float(zz[2]))
			s2 = (float(zz[3]), float(zz[4]), float(zz[5]))
			seeddict[s1] = 1
			seeddict[s2] = 1
	j = 0
	for gg in sorted(seeddict):
		seeds.append(gg)
		seed2label[gg] = j
		neighbors[j] = []
		j = j+1
	with open(fedge,'rb') as f:
		for line in f:
			line = line[:-1].decode('ascii', 'ignore')
			zz = line.split(" ")
			s1 = (float(zz[0]), float(zz[1]), float(zz[2]))
			s2 = (float(zz[3]), float(zz[4]), float(zz[5]))
			neighbors[seed2label[s1]].append(seed2label[s2])
	return(seeds, neighbors)

def distpoint2set(p,sset):
	return(np.median([taxidist(p,gg) for gg in sset]))

def densifymap(seeds, neighbors, step):
	newseeds = []
	newneighbors = {}
	new2old = {}
	old2new = {}
	j = -1
	for node in range(len(seeds)):
		j = j + 1
		newseeds.append(seeds[node])
		new2old[j] = node
		old2new[node] = j
		newneighbors[node] = []
	for v in range(len(seeds)):
		for u in neighbors[v]:
			dd = dist(seeds[v], seeds[u])
			nnn = int(float(dd) / step) + 1
			curnode = old2new[v]
			if v in neighbors.get(u, []):
				duplex = 1
			else:
				duplex = 0
			for ss in range(nnn):
				if ss < nnn - 1:
					j = j + 1
					newneighbors[curnode].append(j)
					newneighbors[j] = []
					if duplex > 0:
						newneighbors[j].append(curnode)
					coef = (float(ss) + 1) / float(nnn)
					newseeds.append((seeds[u][0] * coef + seeds[v][0] * (1 - coef), seeds[u][1] * coef + seeds[v][1] * (1 - coef),max(seeds[u][2],seeds[v][2])))
					curnode = j
				else:
					newneighbors[curnode].append(old2new[u])
	return (newseeds, newneighbors)

def prunesparsedeadclusters(clusters,clusterneighbour,deadclusters):
	realdead = set(); setdeadclusters = set(deadclusters)
	for iic,cl in enumerate(clusters):
		alldeadpotential = set()
		bfsfronteer = [iic]; visitednodes = set()
		while len(bfsfronteer)>0 and iic in setdeadclusters:
			alldeadpotential.update(bfsfronteer)
			newbfsfronteer = []
			for jjc in bfsfronteer:
				for kkc in clusterneighbour[jjc]:
					if kkc in setdeadclusters and not kkc in alldeadpotential:
						newbfsfronteer.append(kkc)
			bfsfronteer = newbfsfronteer
		if len(alldeadpotential)>10:
			realdead.update(alldeadpotential)
	return(list(realdead))

def detectdeadclusters(datapointwts,clusters, clusterneighbour):
	oneway = {}; onewayint = {}
	for igg, gg in enumerate(clusters):
		oneway[gg[:2]] = oneway.get(gg[:2], 0)+1
	for igg, gg in enumerate(clusters):
		onewayint[igg] = oneway[gg[:2]]
	cluster, p2cluster = point2cluster(datapointwts,clusters)
	centroids = {}; deadclusters = []
	centralitytrheshold = 1
	for cd in cluster:
		if len(cluster[cd])>0:
			hh = np.arctan2(sum([np.sin(xx[2]/360.0*2*np.pi) for xx in cluster[cd]]),sum([np.cos(xx[2]/360.0*2*np.pi) for xx in cluster[cd]]))*180.0/np.pi
			centroids[cd] = ((np.mean([xx[0] for xx in cluster[cd]]),np.mean([xx[1] for xx in cluster[cd]]),hh))
			if int(1000*clusters[cd][3])>centralitytrheshold*onewayint[cd]**3 and taxidist(clusters[cd],centroids[cd])>10:
#            if int(1000*clusters[cd][3])>centralitytrheshold+0*onewayint[cd]**3 and distpoint2set(clusters[cd],cluster[cd])>25:
				deadclusters.append(cd)
		elif int(1000*clusters[cd][3])>centralitytrheshold*onewayint[cd]**3:#  and onewayint[cd]>2:
			deadclusters.append(cd)
	deadclusters = prunesparsedeadclusters(clusters,clusterneighbour,deadclusters);
	return(deadclusters)


def plotmap_6(seeds, neighbors, datapointwts, passededges, deadclusters, ccolor):
	segs = []
	for n1 in range(len(seeds)):
		for n2 in neighbors.get(n1, []):
			segs.append(((seeds[n1][0], seeds[n1][1]), (seeds[n2][0], seeds[n2][1])))
#    plt.figure()
	ax = plt.gca()
	ln_coll = matplotlib.collections.LineCollection(segs, colors=ccolor)
	ax.add_collection(ln_coll)
	ax.get_xaxis().get_major_formatter().set_useOffset(False)
	ax.get_yaxis().get_major_formatter().set_useOffset(False)
	ax.add_collection(ln_coll)
	segs = []
	for edge in passededges:
			segs.append(((seeds[edge[0]][0], seeds[edge[0]][1]), (seeds[edge[1]][0], seeds[edge[1]][1])))
	ln_coll = matplotlib.collections.LineCollection(segs, colors=(0, 1, 0 ))
	ax.add_collection(ln_coll)
	left = np.min([xx[0] for xx in seeds])
	right = np.max([xx[0] for xx in seeds])
	low = np.min([xx[1] for xx in seeds])
	high = np.max([xx[1] for xx in seeds])
	ax.set_xlim(left, right)
	ax.set_ylim(low, high)
	plt.draw()
	segs = []
	for n1 in range(len(seeds)):
		gg = seeds[n1]; arl = 0.00002;
		segs.append(((gg[0], gg[1]), (gg[0] - arl * np.sin(np.pi / 180 * gg[2]),gg[1] - 0.9999 * arl * np.cos(np.pi / 180 * gg[2]) )))
	ln_coll = matplotlib.collections.LineCollection(segs, colors=(0,0, 1 ))  # (rr, 0, 1 - rr))
	ax.add_collection(ln_coll); segs = []
	for n1 in range(len(datapointwts)):
		gg = datapointwts[n1]; arl = 0.00002
		segs.append(((gg[0], gg[1]), (gg[0] - arl * np.sin(np.pi / 180 * gg[2]),gg[1] - 0.9999 * arl * np.cos(np.pi / 180 * gg[2]))))
#    for igg, gg in enumerate(seeds):
#        seeds[igg] = (gg[0],gg[1],np.log(1+int(1000*gg[3])) )
	maxcent = max([gg[2] for gg in seeds])
	lats, lons = [xx[0] for xx in seeds],[xx[1] for xx in seeds]
	plt.scatter(lats,lons,s =1, c = ccolor)
	lats, lons = [seeds[xx][0] for xx in deadclusters],[seeds[xx][1] for xx in deadclusters]
	plt.scatter(lats,lons,s =20, c = 'k')
	return(ax)

def getclusters(seeds, neighbors):
	clusters = []; neighbors_reverse = {}; clustersid = {}; jj = 0; clusterneighbour = {};
	for n1, seed in enumerate(seeds):
		for n2 in neighbors[n1]:
			seed2 = seeds[n2]
			new_d_lon = seed[0] - seed2[0]
			new_d_lat = seed[1] - seed2[1]
			angle = -math.degrees(math.atan2(new_d_lat, new_d_lon)) + 90
			clusters.append(((seed[0]+seed2[0])/2.0,(seed[1]+seed2[1])/2.0,angle,min(seed[2],seed2[2])))
			clustersid[(n1, n2)] = jj
			clusterneighbour[jj] = []
			jj = jj+1
			if not n2 in neighbors_reverse:
				neighbors_reverse[n2] = [n1]
			else:
				neighbors_reverse[n2].append(n1)
	for n1, seed in enumerate(seeds):
		for n2 in neighbors[n1]:
			for n0 in neighbors_reverse.get(n1,[]):
				c0 = clustersid[(n0, n1)]
				c1 = clustersid[(n1, n2)]
				clusterneighbour[c0].append(c1)
	return(clusters,clusterneighbour)

def deletenodesfromgraph(seeds, neighbors,deleteindex):
	j = 0; deleteset = set(deleteindex); seedsafterdel = []; old2newindex = {}; neighborsafterdel = {};
	for iis,gg in enumerate(seeds):
		if not iis in deleteset:
			seedsafterdel.append(gg)
			old2newindex[iis] = j
			j = j + 1
	for iis, gg in enumerate(seeds):
		if iis in old2newindex:
			j = old2newindex[iis]
			neighborsafterdel[j] = [old2newindex[ll] for ll in neighbors[iis] if not ll in deleteset];
	return(seedsafterdel,neighborsafterdel)

# def point2cluster(datapointwts,seeds):
#     cluster = {};p2cluster = []; gedges = {}; gedges1 = {}; nedges = {}; std = {}; seeds1 = []; seedweight = [];
#     X = [(70197 * xx[0], 111256 * xx[1], theta / 180 * xx[2]) for xx in datapointwts];    S = [(70197 * xx[0], 111256 * xx[1], theta / 180 * xx[2]) for xx in seeds];
#     Xrot = [(70197 * xx[0], 111256 * xx[1], theta / 180 * (xx[2]%360)) for xx in datapointwts];    Srot = [(70197 * xx[0], 111256 * xx[1], theta / 180 * (xx[2]%360)) for xx in seeds];
#     for cd in range(len(seeds)):
#         cluster[cd] = []
#     nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(S)
#     distances, indices = nbrs.kneighbors(X)
#     nbrsrot = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(Srot)
#     distancesrot, indicesrot = nbrsrot.kneighbors(Xrot)
#     for ii, ll in enumerate(indices):
#         cd = indicesrot[ii][0]
#         if distances[ii][0] < distancesrot[ii][0]:
#             cd = indices[ii][0];
#         cluster[cd].append(datapointwts[ii])
#         p2cluster.append(cd)
#     return(cluster,p2cluster)
#
# def anglebetweentwopoints(LL1, LL2):
#     xd = (LL1[0]-LL2[0]); yd =LL1[1]-LL2[1];
#     return(np.arctan2(xd,yd)*180/np.pi)
#
# def distcolinearity(point1, point2):
#     AA = anglebetweentwopoints(point1, point2)
#     return((70197*np.abs(point1[0]-point2[0])+111256*np.abs(point1[1]-point2[1]))*(angledist(point1[2],AA)+angledist(point2[2],AA)))

def set2setdistance(set1,set2,theta):
	cluster = {};p2cluster = []; gedges = {}; gedges1 = {}; nedges = {}; std = {}; seeds1 = []; seedweight = []; distances12 = []; indices12 = []
	X = [(70197 * xx[0], 111256 * xx[1], theta / 180.0 * xx[2]) for xx in set1];    S = [(70197 * xx[0], 111256 * xx[1], theta / 180.0 * xx[2]) for xx in set2]
	Xrot = [(70197 * xx[0], 111256 * xx[1], theta / 180.0 * (xx[2]%360)) for xx in set1];    Srot = [(70197 * xx[0], 111256 * xx[1], theta / 180.0 * (xx[2]%360)) for xx in set2]
	nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(S)
	distances, indices = nbrs.kneighbors(X)
	nbrsrot = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(Srot)
	distancesrot, indicesrot = nbrsrot.kneighbors(Xrot)
	for ii, ll in enumerate(indices):
		if distances[ii][0]<distancesrot[ii][0]:
			distances12.append(distances[ii][0])
			indices12.append(indices[ii][0])
		else:
			distances12.append(distancesrot[ii][0])
			indices12.append(indicesrot[ii][0])
	return(distances12,indices12)

def shortestpath(seedsosm, neighborsosm, vstart,vend):
	fronteer = [vstart]
	depth = 20
	visited = {}
	visited[vstart] = (0,vstart)
	for dd in range(1, depth):
		newfronteer = []
		for v in fronteer:
			for n2 in neighborsosm[v]:
				if not (n2 in visited):
					newfronteer.append(n2)
					visited[n2] = (dd,v)
		fronteer = newfronteer
	path = []
	if (vend in visited) and (not vstart == vend):
		xx = visited[vend][1]
		path.append((xx, vend))
		while visited[xx][0] > 0:
			path.append((visited[xx][1],xx))
			xx = visited[xx][1]
	return(path)

def extendpath(ll,seedskha, neighborskha ,distances,reverseneighborskha,alreadyvisited):
	path = set(); frontier = [ll]; ok = 1; visited = set(); ddcount = 0; lenseeds = len(seedskha);
	while ok > 0:
		ok = -1
		newfrontier = []
		for ff in frontier:
			if distances[ff] > 10:
				for jj in neighborskha[ff]:
					if not (ff,jj) in path and dist(seedskha[ff],seedskha[jj])<200:#  and dist(seedskha[ff], seedskha[jj]) < 100:
						path.add((ff, jj))
						newfrontier.append(jj)
						visited.add(jj)
						ok = 1
		frontier = newfrontier
	frontier = [ll]
	ok = 1
	while ok > 0:
		ok = -1
		newfrontier = []
		for ff in frontier:
			if distances[ff] > 10:
				for jj in reverseneighborskha.get(ff,[]):
					if not (jj,ff) in path and jj<lenseeds and dist(seedskha[ff],seedskha[jj])<200: #  and dist(seedskha[ff], seedskha[jj]) < 100:
						path.add((jj,ff))
						newfrontier.append(jj)
						visited.add(jj)
						ok = 1
		frontier = newfrontier
	alreadyvisited.update(visited)
	return path,alreadyvisited

def stitchtwomaps(seedskha, neighborskha ,clusters,clusterneighbour):
	reverseneighborskha = {}; alreadyvisited = set(); newneighborskha = {}
	for igg,gg in enumerate(seedskha):
		for jgg in neighborskha[igg]:
			if not jgg in reverseneighborskha:
				reverseneighborskha[jgg] = []
			reverseneighborskha[jgg].append(igg)
	distances,indices = set2setdistance(seedskha, clusters,50)
	potentialnewroads = set([ll for ll,gg in enumerate(distances) if distances[ll]>30])
	for ll in potentialnewroads:
		if not ll in alreadyvisited:
			stitchpath,alreadyvisited  = extendpath(ll,seedskha, neighborskha ,distances,reverseneighborskha,alreadyvisited)
		for gg in stitchpath:
			if not gg[0] in newneighborskha:
				newneighborskha[gg[0]] = []
			newneighborskha[gg[0]].append(gg[1])
		for gg in stitchpath:
			if distances[gg[0]]<10:
				seedskha[gg[0]] = clusters[indices[gg[0]]]
			if distances[gg[1]]<10:
				seedskha[gg[1]] = clusters[indices[gg[1]]]
	for gg in newneighborskha:
		newneighborskha[gg] = list(set(newneighborskha[gg]))
	return(seedskha, newneighborskha)

def printedges_4(seedskha, neighborskha, clusters, clusterneighbour):
	with open('data/tmp/edges3011_status.txt', 'w') as fdist:
		for igg, gg in enumerate(seedskha):
			for ihh in neighborskha.get(igg,[]):
				fdist.write("%s %s %s %s %s\n" % (gg[0], gg[1], seedskha[ihh][0], seedskha[ihh][1], 1))
		for igg, gg in enumerate(clusters):
			for ihh in clusterneighbour.get(igg,[]):
				fdist.write("%s %s %s %s %s\n" % (gg[0], gg[1], clusters[ihh][0], clusters[ihh][1], 0))


"""
Methods that were in gistgileshape.py
"""

def read_osm_wtype_sa(G):
	"""
	Sofiane created this method to consume a graph G that we generate on the flight from Shapefiles
	:param G: base_map created from shapefiles
	:return:
	"""
	insidenode = {}
	lanes = {}
	maxspeed = {}
	nodelat = []
	nodelon = []
	nodeid = []
	j = 0
	id2label = {}
	neighbors = {}
	nodelatlonlanes = []
	for kk in [1]:
		bcent =  betweenness_centrality(G, k=kk)
	for n_id in G.nodes_iter():
		lanes[n_id] = G.node[n_id].get('lanes', None)
		maxspeed[n_id] = G.node[n_id].get('maxspeed', None)
		if 1 > 0:
			insidenode[n_id] = 1
			nodelat.append(G.node[n_id]['lat'])
			nodelon.append(G.node[n_id]['lon'])
			nodelatlonlanes.append((n_id[1], n_id[0], 0*bcent.get(n_id, 0), lanes[n_id], maxspeed[n_id]))
			nodeid.append(n_id)
			id2label[n_id] = j
			neighbors[j] = []
			j = j + 1
	segs = []
	for e_id in G.edges_iter():
		n1 = e_id[0]
		n2 = e_id[1]
		if n1 in insidenode and n2 in insidenode:
			segs.append((n1, n2))
			neighbors[id2label[n1]].append(id2label[n2])
	return(G, nodelatlonlanes, neighbors)

def read_osm_wtype(filename_or_stream):
	insidenode = {}
	lanes = {}
	maxspeed = {}
	G = nx.read_gpickle('data/tmp/osm_network.gpickle')
	nodelat = []
	nodelon = []
	nodeid = []
	j = 0
	id2label = {}
	neighbors = {}
	nodelatlonlanes = []
	for kk in [1]:
		bcent =  betweenness_centrality(G, k=kk)
	for n_id in G.nodes_iter():
		lanes[n_id] = G.node[n_id]['lanes']
		maxspeed[n_id] = G.node[n_id]['maxspeed']
		if 1 > 0:
			insidenode[n_id] = 1
			nodelat.append(G.node[n_id]['lat'])
			nodelon.append(G.node[n_id]['lon'])
			nodelatlonlanes.append((n_id[1], n_id[0], 0*bcent.get(n_id, 0), lanes[n_id], maxspeed[n_id]))
			nodeid.append(n_id)
			id2label[n_id] = j
			neighbors[j] = []
			j = j + 1
	segs = []
	for e_id in G.edges_iter():
		n1 = e_id[0]
		n2 = e_id[1]
		if n1 in insidenode and n2 in insidenode:
			segs.append((n1, n2))
			neighbors[id2label[n1]].append(id2label[n2])
	return(G, nodelatlonlanes, neighbors)