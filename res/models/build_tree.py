# bin/include/python3.7

"""
Bachelor Thesis
Nils Harbke
> Optimized Out-of-Core Rendering for Interactive
> Presentation of Large Point Clouds
HAW Hamburg

"""

import os
import sys
import time
import queue
import psutil
import argparse
import threading
import numpy as np

# color codes for console output
W = '\033[0m'  # white
R = '\033[31m' # red
G = '\033[32m' # green
O = '\033[33m' # orange
B = '\033[34m' # blue
P = '\033[35m' # purple

XDIM = 'X'
YDIM = 'Y'
ZDIM = 'Z'

DIM_P_IDX	= { XDIM:0, YDIM:1, ZDIM:2 }
DIM_COLOR	= { XDIM:R, YDIM:G, ZDIM:B }

SPHERE 	   = 'SPHERE'
AABB   	   = 'AABB'
SPLITPLANE = 'SPLITPLANE'

MIN_IN_S	= 60
HOUR_IN_S	= MIN_IN_S * 60

tree_depth = 0

if __debug__:
	LOG_QUEUE = queue.Queue(maxsize=5)
	MERGE_PROGRESS = { XDIM : dict(), YDIM : dict(), ZDIM : dict() }
	

def main(args):

	global tree_depth

	# create/empty tmp folder for chunk files
	# ---------------------------------------
	if os.path.exists('tmp'):
		for file in os.listdir('tmp'):
			os.unlink('tmp/'+file)
	else:
		os.mkdir('tmp')
	
	KD_OUT = open(args.data[:-8]+'%s_B%s.bin'%(args.boundingvolume,args.bucket), 'wb+')
	#KD_OUT = np.memmap(args.data[:-8]+'KDTREE_B%s_%s.bin'%(args.bucket,args.type), dtype=np.uint32, mode='w+')

	# memory map the input files
	# --------------------------
	XMAP = np.memmap(args.x, dtype=np.uint32, mode='r', offset=4)
	YMAP = np.memmap(args.y, dtype=np.uint32, mode='r', offset=4)
	ZMAP = np.memmap(args.z, dtype=np.uint32, mode='r', offset=4)
	DATA = np.memmap(args.data, 
		dtype=[(XDIM, np.float32), (YDIM, np.float32), (ZDIM, np.float32)],
		mode='r')
	
	# write header information into KD_OUT
	# ------------------------------------
	KD_OUT.write(np.uint32(-1))		   # tree depth (placeholder)
	KD_OUT.write(np.uint32(len(XMAP))) # number of elements
	if args.boundingvolume == SPHERE:  # BVH flag
		KD_OUT.write(np.uint32(0))
	elif args.boundingvolume == AABB: 
		KD_OUT.write(np.uint32(1))
	elif args.boundingvolume == NONE:
		KD_OUT.write(np.uint32(2))
	elif args.boundingvolume == SPLITPLANE:
		KD_OUT.write(np.uint32(3))
	
	# check if everything fits into memory
	# ------------------------------------
	# note: this is entirely made up and not accurate at all as the size
	# of values in memory tremendously differs from the bytesize in storage
	# TODO
	lists_size	= XMAP[0] * 8  # [nb indices] *  sizeof(uint32) * 2
		# note: don't forget that the lists are doubled when
		# being split into two newly sortet subsets
	data_size	= XMAP[0] * 12 # [nb indices] * (sizeof(float32)*3)
	expected_memory = lists_size + data_size
	
	if expected_memory < psutil.virtual_memory().available:
		
		# load everything into memory replacing the memory mapped files
		# with actual in-memory lists
		# -------------------------------------------------------------
		xlist = [np.uint32] * len(XMAP)
		ylist = [np.uint32] * len(YMAP)
		zlist = [np.uint32] * len(ZMAP)
		
		data_dict = dict()
		
		for i in range(len(XMAP)):
			
			xlist[i] = XMAP[i]
			ylist[i] = YMAP[i]
			zlist[i] = ZMAP[i]
			
			# load only the required subset of the whole data file as
			# dict to still be able to use the original indexing
			# note: luckily arrays and dicts share some of their syntax
			# ---------------------------------------------------------
			# Key = Index ; Value = Point
			data_dict[XMAP[i]] = DATA[XMAP[i]]
			
			if __debug__:
				log("\rMove whole data into memory: {0}{1: >3.0f}%".format(G,(i/len(XMAP)*100)))
		if __debug__:
			log('\n')
		
		# replace mapped lists with in-memory lists
		# -----------------------------------------
		XMAP = xlist
		YMAP = ylist
		ZMAP = zlist
	
		DATA = data_dict
		
	elif __debug__:
		log("Not fitting into memory.")
			
	# begin kd-sorting function
	# -------------------------
	kdtree(XMAP,YMAP,ZMAP,DATA,args.bucket,args.boundingvolume,KD_OUT,-1, 'root',1)
	
	KD_OUT.seek(0,0)
	KD_OUT.write(np.uint32(tree_depth))
	
	KD_OUT.close()
	
	# clean up tmp folder
	# -------------------
	for file in os.listdir('tmp'):
		os.unlink('tmp/'+file)
		# this should never be called as each merge thread calls os.unlink()
		# on merged chunks to remove them
		log(R+"%s wasn't properly removed from tmp/ folder\n"%file)
	os.rmdir('tmp')
	
	
def kdtree(X,Y,Z,DATA,BUCKET,TYPE,KD_OUT,child_offset_pos,name,depth):
	
	global tree_depth
	
	# write file offset to parent node
	# --------------------------------
	if child_offset_pos != -1:
		current_pos = KD_OUT.tell()
		KD_OUT.seek(child_offset_pos,0)
		KD_OUT.write(np.uint32(current_pos/4)) 
			# '/4' for direct use with float pointers in C++ and less work
			# for the loading process
		KD_OUT.seek(current_pos,0)
	
	# get the number of elements m to be sorted
	# ---------------------------------------
	m = len(X)
	
	#if __debug__: print('')
	
	if m <= BUCKET:
		if depth > tree_depth:
			tree_depth = depth
		KD_OUT.write(np.float32(0))
		KD_OUT.write(np.uint32(len(X)))
		for element in X:
			KD_OUT.write(np.uint32(element*3))
		return
	
	LISTS = {XDIM:X, YDIM:Y, ZDIM:Z}
	
	# get dimension of largest extend X/Y/Z
	# -------------------------------------
	cutdim, min_vals, max_vals = dim_of_largest_extend(LISTS,DATA)
	
	# read median point index from the list of that cutting dimension
	# ---------------------------------------------------------------
	median = LISTS[cutdim][np.uint32(m/2)]
	
	split_point = DATA[median]
	
	# calculate bounding sphere with center split_point
	# ---------------------------------------------
	if TYPE == SPHERE:
		radius = max(
			abs(split_point[XDIM] - min_vals[XDIM]),
			abs(split_point[XDIM] - max_vals[XDIM]),
			abs(split_point[YDIM] - min_vals[YDIM]),
			abs(split_point[YDIM] - max_vals[YDIM]),
			abs(split_point[ZDIM] - min_vals[ZDIM]), 
			abs(split_point[ZDIM] - max_vals[ZDIM]))
	elif TYPE == AABB:
		aabb = {
			XDIM : min_vals[XDIM],
			YDIM : min_vals[YDIM],
			ZDIM : min_vals[ZDIM],
			'width' : abs(max_vals[XDIM] - min_vals[XDIM]),
			'height': abs(max_vals[YDIM] - min_vals[YDIM]),
			'depth' : abs(max_vals[ZDIM] - min_vals[ZDIM])
		}
			
	# initialize child arrays
	# -----------------------
	if type(X) is type(np.memmap):
		XL = np.memmap('tmp/'+name+'-XL.bin', dtype=np.uint32, mode='w+')
		XR = np.memmap('tmp/'+name+'-XR.bin', dtype=np.uint32, mode='w+')
		YL = np.memmap('tmp/'+name+'-YL.bin', dtype=np.uint32, mode='w+')
		YR = np.memmap('tmp/'+name+'-YR.bin', dtype=np.uint32, mode='w+')
		ZL = np.memmap('tmp/'+name+'-ZL.bin', dtype=np.uint32, mode='w+')
		ZR = np.memmap('tmp/'+name+'-ZR.bin', dtype=np.uint32, mode='w+')
	else:
		XL,XR = [],[]
		YL,YR = [],[]
		ZL,ZR = [],[]
	
	# iterate over all points and sort them to left/right of the
	# splitting plane
	# ----------------------------------------------------------
	for i in range(m):
		
		if DATA[X[i]][cutdim] < split_point[cutdim]:
			XL.append(X[i])
		elif X[i] != median:
			XR.append(X[i])
			
		if DATA[Y[i]][cutdim] < split_point[cutdim]:
			YL.append(Y[i])
		elif Y[i] != median:
			YR.append(Y[i])
			
		if DATA[Z[i]][cutdim] < split_point[cutdim]:
			ZL.append(Z[i])
		elif Z[i] != median:
			ZR.append(Z[i])
			
		if __debug__:
			#log("\r{0} {3}m: {1}{4} Sort lists: {2: >3.0f}%".format(name,m,(i/m*100),O,W),end='')
			log("\rSort lists: {0}{1: >3.0f}% {2}m:{3: >10d} {4}{5}".format(
				G,(i/m*100),O,m,W,name))
	#if __debug__: print('')
	
	# clear memory by deleting no longer needed lists
	# -----------------------------------------------
	del X,Y,Z
	
	# write current node
	# ------------------
	if TYPE == SPHERE:
		KD_OUT.write(np.float32(radius))
	elif TYPE == AABB:
		KD_OUT.write(np.float(aabb[XDIM]))
		KD_OUT.write(np.float(aabb[YDIM]))
		KD_OUT.write(np.float(aabb[ZDIM]))
		KD_OUT.write(np.float(aabb['width']))
		KD_OUT.write(np.float(aabb['height']))
		KD_OUT.write(np.float(aabb['depth']))
	elif TYPE == SPLITPLANE:
		KD_OUT.write(np.uint32(DIM_P_IDX[cutdim]))
	KD_OUT.write(np.uint32(median*3))
	child_offset_pos = KD_OUT.tell()	
	# reserve 8 byte for child offsets
	# --------------------------------
	KD_OUT.write(np.uint32(-1))
	KD_OUT.write(np.uint32(-1))
	
	# branch further into recursion
	if m/2 > 0:
		kdtree(XL,YL,ZL,DATA,BUCKET,TYPE,KD_OUT,child_offset_pos,name+R+'>L',depth+1)
	if m/2 +1 < m:
		kdtree(XR,YR,ZR,DATA,BUCKET,TYPE,KD_OUT,child_offset_pos+4,name+G+'>R',depth+1)
	
	
def dim_of_largest_extend(LISTS,DATA):

	# store value as tuple with dim metadata
	# --------------------------------------
	largest_extend = (-1,'')
	
	min_vals = {}
	max_vals = {}
	
	for dim in [XDIM, YDIM, ZDIM]:
		
		# calculate extend
		# ----------------
		smallest = DATA[LISTS[dim][0]][dim]
		largest  = DATA[LISTS[dim][-1]][dim]
		extend = np.sqrt(np.power(largest-smallest, 2))
		min_vals[dim] = smallest
		max_vals[dim] = largest
		
		# update largest extend
		# ---------------------
		if extend > largest_extend[0]:
			largest_extend = (extend,dim)
	
	return largest_extend[1], min_vals, max_vals

	
""" Worker thread for handling output to console """
class Logger(threading.Thread):

	def __init__(self):
		super(Logger,self).__init__()
		self.daemon = True
		self.stop_event = threading.Event()
		self.last_line = ""
	
	def run(self):
		global LOG_QUEUE
		while True:
			line = LOG_QUEUE.get(block=True)
			if line != self.last_line:
				sys.stdout.write(line+W)
				sys.stdout.flush()
				self.last_line = line
			LOG_QUEUE.task_done()


def log(message):
	global LOG_QUEUE
	try:
		LOG_QUEUE.put_nowait(message)
	except queue.Full:
		pass
	
	
if __name__ == '__main__':

	if __debug__:
		my_logger = Logger()
		my_logger.start()
			
	# initiate argument parser
	# ------------------------
	parser = argparse.ArgumentParser(description = "[Datastructure Builder]\n Takes BIN point data and sorted lists for each dimension, stores them in a custom kdtree structure including bounding volumes and cluster of given size.")
	
	# add arguments to the parser
	# ---------------------------
	parser.add_argument("-d","--data",
		help="file containing vertex information")
	parser.add_argument("-x",
		help="file with indices sorted by x-axis")
	parser.add_argument("-y",
		help="file with indices sorted by y-axis")
	parser.add_argument("-z",
		help="file with indices sorted by z-axis")
	parser.add_argument("-b","--bucket", type=int, default=2,
		choices=range(2,9999999),
		help="size of data cluster at the trees leafs")
	parser.add_argument("-bv","--boundingvolume", default=SPHERE,
		choices=['SPHERE','AABB','SPLITPLANE'],
		help="type of metadata for different culling techniques: SPHERE, AABB or SPLITPLANE")
	
	# read arguments from command line
	# --------------------------------
	args = parser.parse_args()
	
	if args.x and args.y and args.z and args.data:
	
		if __debug__:
			log(O+"{0}\n START BUILD-STRUCTURE\n{0}\n".format('-'*23))
	
		# start main function
		# -------------------
		main_tstart = time.time()
		main(args)
		main_tend = time.time()		
		
		if __debug__:
				# print total runtime in minutes
			log(O+"\n\n{0}\nTime total: {1:6.3f} min\n\n".format('-'*19,
				(main_tend-main_tstart)/MIN_IN_S))
				# stop Logger thread
			LOG_QUEUE.join()
		else:
			print(O+"\n\n{0}\nTime total: {1:6.3f} min\n\n".format('-'*19,
				(main_tend-main_tstart)/MIN_IN_S))
		
	else:
		print("missing arguments\n\tbuild_structure.py -d <datafile> -x <x-list>"
			+"-y <y-list> -z <z-list> -b <bucketsize> -t <bvhtype>")
		sys.exit(2)
	