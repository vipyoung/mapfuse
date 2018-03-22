import time
import getopt
import sys
from OSMReader import OSMReader
from methods import methods, getourmap, getosmmap, densifymap, getclusters, detectdeadclusters, \
	deletenodesfromgraph, stitchtwomaps, printedges_4, getdata, densifykm, computeclusters, \
	splitclusters, coocurematrix, prunegraph, getgeojson, printedges, printclusters

if __name__ =='__main__':
	# Default parameters
	nsamples = 20000000
	datestr = '2015-11-01'
	data_path = 'data/gps_data.csv'
	basemap_path = 'data/osm/segments.shp'
	plot_map = 0

	(opts, args) = getopt.getopt(sys.argv[1:], "d:b:t:p:h")
	for o, a in opts:
		if o == "-d":
			data_path = str(a)
		if o == "-b":
			basemap_path = str(a)
		if o == "-t":
			datestr = str(a)
		if o == "-p":
			plot_map = int(a)
		if o == "-h":
			print("Usage: python QMapFuse.py [-d <path_to_data_file>] [-b <path_to_shape_file>]"\
				"[-t <starting data: 2015-11-01>] [-p <1:plot fused map, 0: otherwise>] [-h <help>]\n"\
				"e.g. python QMapFuse.py -d data/GPS_Dataprivatebus1130.csv -b data/segments/segments.shp -t 2015-11-01 -p 1\n")
			exit()

	# 1. Run Kharita.
	datestrend = datestr[:-2] + str(31)
	start = time.time()
	data = getdata(nsamples, datestr, datestrend, data_path)
	print('read %s tuples in %s' % (len(data), time.time() - start))
	densified_data = densifykm(data)
	print('data densified iKn %s' % (time.time() - start))
	datapointwts = [xx for xx in densified_data if xx[3] >= 5]
	print('remaining datapoints with speed > 5kmph: ', len(datapointwts))
	print('Clustering\n#iter\tscore')
	seeds = computeclusters(datapointwts)
	print(len(seeds), time.time() - start)
	for ii in range(3):
		seeds, seedweight = splitclusters(datapointwts, seeds)
	print('cluster split: ', time.time() - start)
	gedges = coocurematrix(datapointwts, seeds)
	print('coocurence matrix computed: ', time.time() - start)
	gedges = prunegraph(gedges, seeds)
	print('graph pruning. number of edges = ', len(gedges), time.time()-start)
	getgeojson(gedges, seeds)
	printedges(gedges, seeds, datapointwts)
	printclusters(seeds)

	# 2. Run the map fusion code.
	start = time.time()
	# Work on the static map: OSM, etc.
	G = OSMReader.build_road_network_from_osm_shapefile(basemap_path)
	print('Generated OSM graph from shapefiles in:', time.time() - start)
	seedsosm, neighborsosm = getosmmap(base_map=G)
	seedsosm, neighborsosm = densifymap(seedsosm, neighborsosm, 20)
	print ('osm map read/densified: ', time.time()-start, len(seedsosm), \
	       sum([len(neighborsosm[xx]) for xx in neighborsosm]))
	clusters, clusterneighbour = getclusters(seedsosm, neighborsosm)
	deadclusters = detectdeadclusters(densified_data, clusters, clusterneighbour)
	print('dead clusters found: ', time.time() - start, len(deadclusters))
	clusters1, clusterneighbour1 = deletenodesfromgraph(clusters, clusterneighbour, deadclusters)
	# Work on Kharita
	seedskha, neighborskha = getourmap()
	seedskha, neighborskha = stitchtwomaps(seedskha, neighborskha, clusters1, clusterneighbour1)
	print('kharita new segments added: ', time.time() - start, 'number of new (kharita) nodes: ', \
	      len([item for gg in neighborskha for item in neighborskha[gg]]))

	printedges_4(seedskha, neighborskha, clusters1, clusterneighbour1)
	methods.save_edgeFile_into_geojson(edgeFile='data/tmp/edges3011_status.txt', outFile='data/fused_map.geojson')

	# Plotting the map
	if plot_map:
		print('Plotting the fused map')
		methods.draw_fused_map(fname='data/fused_map.geojson', showNodes=True)


