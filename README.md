# MapFuse
MapFuse solves the problem of fusing two different maps in order to highlight commonalities and differences between them.
A relevant use case of MapFuse would be fuse an existing outdated yet high quality map representing a road network with
a more recent map created from GPS traces in order to update the existing one. 
  
The solution takes as input a shapefile  of a map (say osm) and a csv file of GPS points.
It generates a new shapefile or geojson file of the fused map which highlights new segments and dead segments.

To learn more about the project, check our paper:
_Rade Stanojevic, Sofiane Abbar, Saravanan Thirumuruganathan, Gianmarco De Francisci Morales, Sanjay Chawla, Fethi Filali and Ahid Aleimat_: 
**_Road Network Fusion for Incremental Map Updates._**
In special volume of Springer's Lecture Notes in Cartography and Geoinformation (LBS 2018).

## Running the code:
### Requirements
The easiest way to run the code is to use virtualenv.
- install virtualenv: sudo apt-get install virtualenv
- Head to a directory where you want to create a virtualenv and run:
`virtualenv --no-site-packages qmap_venv`
- Activate the newly created virtualenv as follows: `source qmap_venv/bin/activate`
- Install all packages in requirements.txt as follows: `pip install -r requirements.txt`
Once virtualenv is activated, proceed with the following to see the different options:
`python MapFuse.py -h`

Then run:

`python MapFuse.py -d data/gps_data.csv -b data/osm/segments.shp -t 2015-11-01 -p 1`

* -d: path to the csv data file
* -b: path to the shape file of the base map (QMIC, OSM, etc.)
* -t: the starting date (yyyy-mm-dd) of the gps points to consider in the fusion
* -p: 1 to plot the fused map, 0 not to plot it.

The output fused map is saved into `data/fused_map.geojson`

## Visualizing the output:
The fusion of the two maps can be rendered as follows:

- Green: Base map (OSM)
- Red: newly detected road segments
 
![alt text](img/fused_map.png?raw=true  "FusedMap")
