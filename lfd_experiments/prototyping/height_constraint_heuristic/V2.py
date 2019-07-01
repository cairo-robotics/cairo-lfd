import numpy as np
#from lfd.data_io import DataImporter, DataExporter

import os
import json
from collections import OrderedDict
import glob
import codecs
import itertools
import errno
import random
from vbgmm_precision import VariationalGMM
from sklearn import mixture

import numpy as np
from scipy import linalg
import matplotlib
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization!

color_iter = itertools.cycle(['navy', 'black', 'cornflowerblue', 'r',
							  'darkorange', 'g', 'brown'])

def load_json_file(path):
		# Load a single file with one function call
		with open(path, 'r') as f:
			datastore = json.load(f)
			return datastore

def load_json_files(path, count=None):
	# Load multiple files with one function call
	entries = []
	files = glob.glob(path)
	if count is not None and count > 0:
		files = files[0:count]
	for name in files:
		try:
			with codecs.open(name, "r", 'utf-8') as f:
				file_data = json.load(f, object_pairs_hook=OrderedDict)
				entries.append(file_data)
		except IOError as exc:
			if exc.errno != errno.EISDIR:
				raise  # Propagate other kinds of IOError.
	return entries

def height(object_pose, reference_height, threshold_distance, axis="z"):
	"""
	Determines whether or an object's pose is above a reference height.

	Parameters
	----------
	object_pose : geometry_msgs/Pose
		The object's pose.
	reference_height : float/int
		The base axis value from which to measure the object's height. Using a negative reference value will indicate testing for below a height.
	 threshold_distance : float/int
		Threshold distance within two objects are in proximity.
	axis : str
		Axis against which to measure deviation.

	Returns
	-------
	: int
		1 if above/below threshold distance, 0 otherwise.
	"""
	if axis == "x":
		object_height = object_pose.position.x
	if axis == "y":
		object_height = object_pose.position.y
	if axis == "z":
		object_height = object_pose.position.z

	difference = fabs(object_height - reference_height)

	rospy.logdebug("Height differential: {}".format(difference))

	if difference >= threshold_distance:
		return 1
	else:
		return 0

class HeightConstraint(object):
	"""
	HeightConstraint class to evaluate the height predicate classifier assigned to a given item.

	height() returns true if object distance from reference_height is greater than threshold distnace.

		A positive threshold distance mean height above

		A negative threshold distance means height below.

	Attributes
	----------
	id : int
		Id of the constraint as defined in the config.json file.
	item_id : int
		Id of the item on which the constraint can be applied.
	reference_height : int
		The reference or starting height to compare an objects height distance against the threshold_distance.
	threshold_distance : int
		The distance from reference (positive: above; negative; below) to compare an object's distance
		from reference.
	button : string
		String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
		check the trigger.
	"""
	def __init__(self, constraint_id, item_id, button, reference_height, threshold_distance):

		"""
		These arguments should be in the "init_args" field of the config.json file's entry representing
		this constraint.

		Parameters
		----------
		constraint_id : int
			Id of the constraint as defined in the config.json file.
		item_id : int
			Id of the item on which the constraint can be applied.
		button : string
			String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
			check the trigger.
		reference_height : int
			The reference or starting height to compare an objects height distance against the threshold_distance.
		threshold_distance : int
			The distance from reference (positive: above; negative; below) to compare an object's distance
			from reference.
		"""

		self.id = constraint_id
		self.item_id = item_id
		self.reference_height = reference_height
		self.threshold_distance = threshold_distance
		self.button = button

	def check_trigger(self):
		"""
		This function evaluates whether the constrain has been triggered. In this case,
		this class's trigger uses the cuff buttons of Sawyer.

		Returns
		-------
		: int
			Boolean value of trigger result.
		"""
		if intera_interface.Navigator().get_button_state(self.button) != 0:
			return 1
		else:
			return 0

	def evaluate(self, environment, observation):
		"""
		This function evaluates an observation for the assigned constraint of the class. It differentiates
		between Sawyer (end-effector) and general items (blocks etc,.).

		Parameters
		----------
		environment : Environment
			The Environment object containing the current demonstrations environment (SawyerRobot, Items, Constraints)
			and helper methods.

		observation : Observation
			The observation to evaluate for the constraint.

		Returns
		-------
		 : int
			Boolean value of constraint evaluation for the height constraint.
		"""
		if self.item_id == environment.get_robot_info()["id"]:
			item_data = observation.get_robot_data()
			item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])
		else:
			item_data = observation.get_item_data(self.item_id)
			item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])

		return height(item_pose, self.reference_height, self.threshold_distance)

def vectorize_demonstrations(demonstrations):
	# Retrieve position coordinates for multiple demonstrations
	vectorized_demonstrations = []
	for demo in demonstrations:
		vectorized_demo = []
		for entry in demo:
			vector = entry['robot']['position']
			vectorized_demo.append(vector)
		#vectorized_demo.reverse()
		vectorized_demonstrations.append(vectorized_demo)
	return vectorized_demonstrations

def vectorize_demonstration(demonstration):
	# Retrieve position coordinates for a single demonstration
	vectorized_demonstrations = []
	vectorized_demo = []
	for entry in demonstration:
		vector = entry['robot']['position']
		vectorized_demo.append(vector)
	print(len(vectorized_demo))
	#vectorized_demo.reverse()
	vectorized_demonstrations.append(vectorized_demo)
	return vectorized_demonstrations

def height_heuristic(X, Y_, means, covariances, ax):
	colors = []
	for i, (mean, covar, color) in enumerate(zip(
			means, covariances, color_iter)):

		# as the DP will not use every component it has access to
		# unless it needs it, we shouldn't plot the redundant
		# components.
		if not np.any(Y_ == i):
			continue

		# find the rotation matrix and radii of the axes
		U, s, rotation = linalg.svd(covar)
		radii = np.sqrt(s)

		# now carry on with EOL's answer
		u = np.linspace(0.0, 2.0 * np.pi, 100)
		v = np.linspace(0.0, np.pi, 100)
		x = radii[0] * np.outer(np.cos(u), np.sin(v))
		y = radii[1] * np.outer(np.sin(u), np.sin(v))
		z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

		for i in range(len(x)):
			for j in range(len(x)):
				[x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation) + mean

		# plot
		maxz = -100
		maxzindouter = 0
		maxzindinner = 0
		
		minz = 100
		minzindouter = 0
		minzindinner = 0
		
		for i in range(len(z)):
			temp = z[i].tolist()
			if max(temp) > maxz:
				maxz = max(temp)
				maxzindouter = i
				maxzindinner = temp.index(max(temp))
			if min(temp) < minz:
				minz = min(temp)
				minzindouter = i
				minzindinner = temp.index(min(temp))

		print(maxz)
		print(maxzindouter)
		print(maxzindinner)

		print(minz)
		print(minzindouter)
		print(minzindinner)

		heights = np.linspace(minz,maxz,5)
		print(heights)

		return heights

# http://kylebarbary.com/nestle/examples/plot_ellipsoids.html
# https://www.oreilly.com/learning/three-dimensional-plotting-in-matplotlib
# https://web.ma.utexas.edu/users/m408m/Display12-5-4.shtml
# https://stackoverflow.com/questions/3461869/plot-a-plane-based-on-a-normal-vector-and-a-point-in-matlab-or-matplotlib/12503243
def plot_results(X, Y_, means, covariances, ax):
	#fig = plt.figure()
	#ax = fig.add_subplot(111, projection='3d')
	colors = []
	for i, (mean, covar, color) in enumerate(zip(
			means, covariances, color_iter)):

		# as the DP will not use every component it has access to
		# unless it needs it, we shouldn't plot the redundant
		# components.
		if not np.any(Y_ == i):
			continue
		ax.scatter(X[Y_ == i, 0], X[Y_ == i, 1], X[Y_ ==i, 2], s=.8, color=color)
		#plt.show()

		# find the rotation matrix and radii of the axes
		U, s, rotation = linalg.svd(covar)
		radii = np.sqrt(s)
		#print(radii)

		# now carry on with EOL's answer
		u = np.linspace(0.0, 2.0 * np.pi, 100)
		v = np.linspace(0.0, np.pi, 100)
		x = radii[0] * np.outer(np.cos(u), np.sin(v))
		y = radii[1] * np.outer(np.sin(u), np.sin(v))
		z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

		for i in range(len(x)):
			for j in range(len(x)):
				[x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation) + mean

		# plot
		maxz = -100
		maxzindouter = 0
		maxzindinner = 0
		
		minz = 100
		minzindouter = 0
		minzindinner = 0
		
		for i in range(len(z)):
			print(i)
			temp = z[i].tolist()
			if max(temp) > maxz:
				maxz = max(temp)
				maxzindouter = i
				maxzindinner = temp.index(max(temp))
			if min(temp) < minz:
				minz = min(temp)
				minzindouter = i
				minzindinner = temp.index(min(temp))
				print("new min found")
				print(min(temp))
				print(i)
				print(minzindouter)
				print(minzindinner)
				print(len(z[minzindouter]))
				print(z[minzindouter])
				print(z[minzindouter][minzindinner])
				#print(len(z[minzindouter][minzindinner]))
				print("end")

		#print(maxz)
		#print(maxzindouter)
		#print(maxzindinner)

		print("min info")
		print(minz)
		print(minzindouter)
		print(minzindinner)

		heights = np.linspace(minz,maxz,5)
		print(heights)

		# plot upper and lower height as points
		ax.plot_wireframe(x, y, z, rstride=4, cstride=4, color=color, alpha=0.2)
		ax.scatter3D(x[maxzindouter][maxzindinner],y[maxzindouter][maxzindinner],z[maxzindouter][maxzindinner])
		ax.scatter3D(x[minzindouter][minzindinner],y[minzindouter][minzindinner],z[minzindouter][minzindinner])

		# plot upper height as plane
		psize = 0.02*z[maxzindouter][maxzindinner]
		plot_plane(psize,x,y,z,maxzindouter,maxzindinner,ax)
		# q = np.array([x[maxzindouter][maxzindinner],y[maxzindouter][maxzindinner],z[maxzindouter][maxzindinner]])
		# r = np.array([x[maxzindouter+1][maxzindinner+1],y[maxzindouter+1][maxzindinner+1],z[maxzindouter][maxzindinner]])
		# s = np.array([x[maxzindouter+2][maxzindinner+2],y[maxzindouter+2][maxzindinner+2],z[maxzindouter][maxzindinner]])
		# qr = r-q
		# qs = s-q
		# normal = np.cross(qr,qs)        
		# point = q
		# d = -point.dot(normal)
		# xx, yy = np.meshgrid(np.linspace(x[maxzindouter][maxzindinner]-psize,x[maxzindouter][maxzindinner]+psize,10),np.linspace(y[maxzindouter][maxzindinner]-psize,y[maxzindouter][maxzindinner]+psize,10))
		# zz = (-normal[0] * xx - normal[1] * yy - d) * 1. /normal[2]
		# ax.plot_surface(xx, yy, zz)

		# plot lower height as plane
		psize = 0.02*z[maxzindouter][maxzindinner]
		q = np.array([x[minzindouter][minzindinner],y[minzindouter][minzindinner],z[minzindouter][minzindinner]])
		r = np.array([x[minzindouter+1][minzindinner+1],y[minzindouter+1][minzindinner+1],z[minzindouter][minzindinner]])
		s = np.array([x[minzindouter+2][minzindinner+2],y[minzindouter+2][minzindinner+2],z[minzindouter][minzindinner]])
		qr = r-q
		qs = s-q
		normal = np.cross(qr,qs)        
		point = q
		d = -point.dot(normal)
		xx, yy = np.meshgrid(np.linspace(x[minzindouter][minzindinner]-psize,x[minzindouter][minzindinner]+psize,10),np.linspace(y[minzindouter][minzindinner]-psize,y[minzindouter][minzindinner]+psize,10))
		zz = (-normal[0] * xx - normal[1] * yy - d) * 1. /normal[2]
		ax.plot_surface(xx, yy, zz)

		plt.show()

def plot_plane(psize,x,y,z,outerind,innerind,ax):
	q = np.array([x[outerind][innerind],y[outerind][innerind],z[outerind][innerind]])
	r = np.array([x[outerind+1][innerind+1],y[outerind+1][innerind+1],z[outerind][innerind]])
	s = np.array([x[outerind+2][innerind+2],y[outerind+2][innerind+2],z[outerind][innerind]])
	qr = r-q
	qs = s-q
	normal = np.cross(qr,qs)        
	point = q
	d = -point.dot(normal)
	xx, yy = np.meshgrid(np.linspace(x[outerind][innerind]-psize,x[outerind][innerind]+psize,10),np.linspace(y[outerind][innerind]-psize,y[outerind][innerind]+psize,10))
	zz = (-normal[0] * xx - normal[1] * yy - d) * 1. /normal[2]
	ax.plot_surface(xx, yy, zz)

class VBGMMSegmentation():
	def __init__(self, alldemos):
		self.alldemos = alldemos
		self.build_model(alldemos)

	def build_model(self, data):
		# Build model using every observation available
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		demos = vectorize_demonstrations(data)
		X = np.array([e for sl in demos for e in sl])
		n_samples_tot = len(X)
		#print(n_samples_tot)
		if n_samples_tot < 10:
			self.vbgmm = mixture.GaussianMixture(n_components=5).fit(X)
			#self.vbgmm = VariationalGMM(n_components=n_samples_tot).fit(X)
		else:
			self.vbgmm = mixture.GaussianMixture(n_components=5).fit(X)
		   #self.vbgmm = VariationalGMM(n_components=10).fit(X)
		#print(self.vbgmm.predict(X))
		#print(self.vbgmm.predict(X))
		plot_results(X, self.vbgmm.predict(X), self.vbgmm.means_, self.vbgmm.covariances_, ax)
		plt.show()

	def segment(self, data):
		# Predict segmentation using trained model
		demo = vectorize_demonstration(data)
		#print(demo)
		X = np.array([e for sl in demo for e in sl])
		n_samples = len(X)
		#print(X)
		prediction = self.vbgmm.predict(X)
		#print(prediction)
		#print(len(prediction))

		startindex = []
		endindex = []
		uniquepreds = np.array(list(set(prediction)))
	   
		#print(uniquepreds)
		for num in uniquepreds:
		   result = np.where(prediction == num)
		   startindex.append(result[0][0])
		   if len(result[0]) == 1:
			   endindex.append(result[0][0])
		   else:
			   endindex.append(result[0][len(result[0])-1])

		#Find start and end indices for each observation
		# startindex = []
		# endindex = []
		# for i in range(len(prediction)):
		#     if i == 0:
		#         currentnum = prediction[i]
		#         startindex.append(i)
		#     elif (prediction[i] != currentnum) & (i == len(prediction)-1):
		#         endindex.append(i-1)
		#         startindex.append(i)
		#         endindex.append(i)
		#         currentnum = prediction[i]
		#     elif (prediction[i] != currentnum):
		#         endindex.append(i-1)
		#         startindex.append(i)
		#         currentnum = prediction[i]
		#     elif i == len(prediction)-1:
		#         endindex.append(i)

		#print(startindex)
		#print(endindex)

		# Use start and end indices to create/splice segments
		#print(prediction)

		#sum = 0
		segments = []
		for index in range(len(startindex)):
			#print(prediction[startindex[index]:endindex[index]])
			segments.append(data[startindex[index]:endindex[index]])

		#sum = 0
		# segments = []
		# for index in range(len(startindex)):
		#     if startindex[index] == endindex[index]:
		#         print(prediction[startindex[index]:endindex[index]+1])
		#         #sum = sum + len(prediction[startindex[index]:endindex[index]])
		#         segments.append(data[startindex[index]:endindex[index]+1])
		#     else:
		#         print(prediction[startindex[index]:endindex[index]])
		#         segments.append(data[startindex[index]:endindex[index]])

		#print(sum)
		#print(len(prediction))
		#print(startindex)
		#print(endindex)

		# Return
		return segments           

class Segment():
	def __init__(self,segments):
		self.segments = segments

class DemonstrationSegmenter():
	def __init__(self, segmenter):
		self.segmenter = segmenter

	def segment_demonstrations(self, demonstrations):
		allsegments = {}
		for i in range(len(demonstrations)):
			demo_id = i
			demo_segments = self.segmenter.segment(demonstrations[i])
			allsegments[demo_id] = demo_segments
		# {
		#   1: [segment1for1, segment2for1, segment3for1],
		#   2: [segment1for2, segment2for2, segment3for2]
		# }    		
		return allsegments

	def segment_classification(self, allsegments):
		temp = {}
		for demonum in range(len(allsegments)):
			for segnum in range(len(allsegments[demonum])):
				if segnum not in temp:
					temp[segnum] = {demonum:allsegments[demonum][segnum]}
				else:
					temp[segnum][demonum] = allsegments[demonum][segnum]


		segobjects = []
		for item in temp:
			#print(temp.get(item))
			segobjects.append(Segment(temp.get(item)))
		# SegmentationObject1.segments -> {
		#     1: segment1for1,
		#     2: segment1for2
		# }
		return segobjects

	def scatter_plot_segments(self,allsegments):

		#print(len(allsegments[0]))

		colorcount = 0
		colorlist = ['navy', 'r', 'darkorange', 'black', 'cornflowerblue']

		for demoind in range(len(allsegments)):
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')
			for segind in range(len(allsegments[demoind])):
				x = []
				y = []
				z = []
				color = []
				if colorcount > (len(colorlist)-1):
					colorcount = 0
				#print(colorlist[colorcount])
				segment = allsegments[demoind][segind]
				temp = vectorize_demonstration(segment)
				X = np.array([e for sl in temp for e in sl])
				for point in X:
					x.append(point[0])
					y.append(point[1])
					z.append(point[2])
					color.append(colorlist[colorcount])
				ax.scatter(x,y,z,c=color, edgecolors=color, s= 5)
				#ax.plot_wireframe(x, y, z, rstride=4, cstride=4, alpha=0.2, linewidth = 3, color=color, facecolors=color)
				colorcount = colorcount+1
			ax.set_xlabel("x coordinate")
			ax.set_ylabel("y coordinate")
			ax.set_zlabel("z coordinate")
			plt.show()

		# colorcount = 0
		# for demoind in range(len(allsegments)):
		#     for segind in range(len(allsegments[demoind])):
		#         x = []
		#         y = []
		#         z = []
		#         color = []
		#         if colorcount > (len(colorlist)-1):
		#             colorcount = 0
		#         print(colorlist[colorcount])
		#         segment = allsegments[demoind][segind]
		#         temp = vectorize_demonstration(segment)
		#         X = np.array([e for sl in temp for e in sl])
		#         for point in X:
		#             x.append(point[0])
		#             y.append(point[1])
		#             z.append(point[2])
		#             color.append(colorlist[colorcount])
		#         fig = plt.figure()
		#         ax = fig.add_subplot(111, projection='3d')
		#         ax.scatter(x,y,z,c=color, edgecolors=color, )
		#         #ax.plot_wireframe(x, y, z, rstride=4, cstride=4, alpha=0.2, color=color)
		#         ax.set_xlabel("x coordinate")
		#         ax.set_ylabel("y coordinate")
		#         ax.set_zlabel("z coordinate")
		#         plt.show()
		#         colorcount = colorcount+1

	def plot_segments(self, allsegments, segobjects, vbgmm_segmenter):
		# Plot allsegments
		for demoind in range(len(allsegments)):
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')
			for segind in range(len(allsegments[demoind])):
				#print(segind)   
				segment = allsegments[demoind][segind]
				#print(demoind)
				#print(segind)
				#print(segment)
				#print(len(segment))
				if len(segment) > 1:
					demo = vectorize_demonstration(segment)
					X = np.array([e for sl in demo for e in sl])
					#print(vbgmm_segmenter.vbgmm.predict(X))
					plot_results(X, vbgmm_segmenter.vbgmm.predict(X), vbgmm_segmenter.vbgmm.means_, vbgmm_segmenter.vbgmm.covariances_, ax)
			print('plotting')
			plt.show()
		# {
		#   1: [segment1for1, segment2for1, segment3for1],
		#   2: [segment1for2, segment2for2, segment3for2]
		# }   

		# # Plot segmentation objects
		# fig = plt.figure()
		# ax = fig.add_subplot(111, projection='3d')
		# for obj in segobjects:
		#     for key in obj.segments:
		#         segment = obj.segments[key]
		#         #print(segment)
		#         #print(len(segment))
		#         if len(segment) > 1:
		#             demo = vectorize_demonstration(segment)
		#             X = np.array([e for sl in demo for e in sl])
		#             if segind == 0:
		#                 #print(vbgmm_segmenter.vbgmm.predict(X))
		#                 plot_results(X, vbgmm_segmenter.vbgmm.predict(X), vbgmm_segmenter.vbgmm.means_, vbgmm_segmenter.vbgmm.covariances_, ax)
		#             else:
		#                 #print(vbgmm_segmenter.vbgmm.predict(X))
		#                 plot_results(X, vbgmm_segmenter.vbgmm.predict(X), vbgmm_segmenter.vbgmm.means_, vbgmm_segmenter.vbgmm.covariances_, ax)
		#     print('plotting')
		#     plt.show()
		# # SegmentationObject1.segments -> {
		# #     1: segment1for1,
		# #     2: segment1for2
		# # }      

def main():
	# Build model
	dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'test_data')
	alldemos = load_json_files(os.path.join(dir_path, '*.json'))
	vbgmm_segmenter = VBGMMSegmentation(alldemos)

	# Deploy model to segment each demonstration
	demo_segmenter = DemonstrationSegmenter(vbgmm_segmenter)
	dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'test_data')
	jsonfiles = [f for f in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, f))]
	demonstrations = []
	for file in jsonfiles:
		data = load_json_file(os.path.join(dir_path,file))
		demonstrations.append(data)
	allsegments = demo_segmenter.segment_demonstrations(demonstrations)
	segobjects = demo_segmenter.segment_classification(allsegments)

	# Plot segments
	#print(vbgmm_segmenter)
	demo_segmenter.plot_segments(allsegments, segobjects, vbgmm_segmenter)
	#demo_segmenter.scatter_plot_segments(allsegments)
	
	print("ending")

if __name__ == '__main__':
	main()