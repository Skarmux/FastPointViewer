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
import collections
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

PROCESS 	= psutil.Process(os.getpid())
MB_IN_B		= 1000000
GB_IN_B		= MB_IN_B * 1000
MIN_IN_S	= 60
HOUR_IN_S	= MIN_IN_S * 60

BYTE_OFFSET	= { 'POINT':12, XDIM:0, YDIM:4, ZDIM:8 }

PLY_PATH 	 = ""
MEMORY_CAP 	 = 2
MERGE_QUEUES = { XDIM:queue.Queue(), YDIM:queue.Queue(), ZDIM:queue.Queue() }
FILE_CTR 	 = 0

if __debug__:
	LOG_QUEUE = queue.Queue(maxsize=5)
	MERGE_PROGRESS = { XDIM : dict(), YDIM : dict(), ZDIM : dict() }


def main():

	# create/empty tmp folder for chunk files
	# ---------------------------------------
	if os.path.exists('tmp'):
		for file in os.listdir('tmp'):
			os.unlink('tmp/'+file)
	else:
		os.mkdir('tmp')

	# read PLY header information
	# ---------------------------
	PLY_IN = open(PLY_PATH, 'rb')
	FORMAT_OPT	 = {	b'binary_big_endian'	: '>',
						b'binary_little_endian'	: '<',
						b'ascii' 				: '' }
					
	for line in PLY_IN:
		if line.startswith(b"element vertex"):
			nelems = np.uint32(line.split(b' ')[2])
			if __debug__:
				log("{:<10}: {}{}\n".format("Points",P,nelems))
			
		elif line.startswith(b"format"):
			format = line.split(b' ')[1]
			if __debug__:
				log("{:<10}: {}{}\n".format("Format",P,format))
			format = FORMAT_OPT[format]
			del FORMAT_OPT
		elif line.startswith(b"end_header"):
			break
	del line
	ply_vertices_offset = PLY_IN.tell()

	if __debug__:
		bar_len	= -1
		last_memcap_percent = -1

	# split data into sorted chunks
	# -----------------------------
	if __debug__:
		time_start 	= time.time()
		log(O+"{0}\n SPLIT DATA TO SORTED CHUNKS \n{0}\n".format('-'*29))
	c_points = collections.deque()
	# collections.deque elements can be removed and added on either ends
	# this is benefitial when emptying it into chunks
	
	for i in range(nelems):
		
		if format == '':
			# load point from ascii context
			data = PLY_IN.readline().split(b' ')
			XVAL = np.float32(data[0])
			YVAL = np.float32(data[1])
			ZVAL = np.float32(data[2])
		else:
			# load point from binary context
			XVAL = np.frombuffer(PLY_IN.read(4),dtype=format+'f')[0]
			YVAL = np.frombuffer(PLY_IN.read(4),dtype=format+'f')[0]
			ZVAL = np.frombuffer(PLY_IN.read(4),dtype=format+'f')[0]
		
		c_points.append((XVAL,YVAL,ZVAL,i))
	
		used_bytes = PROCESS.memory_info()[0]

		# print progress
		if __debug__:
			memcap_percent = int((used_bytes / MEMORY_CAP)*100)
			if memcap_percent > last_memcap_percent or (i+1) == nelems:
				last_memcap_percent = memcap_percent
				megabytes = int(used_bytes/MB_IN_B)
				progress = ((i+1)/nelems)
				if round(30*progress) > bar_len:
					# update progress bar string
					bar_len = int(30*progress)
					bar = "<{:30}>".format('='*bar_len)	
				log("\r{}{} {:6.2f}% {}| memory usage: {} MB".format(
					G,bar,progress*100,W,megabytes))
		
		if used_bytes >= MEMORY_CAP:
			if __debug__:
				log(O+"\nmemory cap reached\n")
			write_sorted_chunks(c_points)
			if __debug__:
				log("discard chached points\n")
				last_memcap_percent = -1
			del c_points
			c_points 	= collections.deque()

	# store remaining points and indices in the last chunk
	if __debug__: log("\n")
	
	write_sorted_chunks(c_points)
	
	if __debug__: log("remove points from memory\n\n")
	
	del c_points
	
	if __debug__:
		time_end = time.time()
		log("Time: {}{:6.3f}m\n\n".format(O,(time_end-time_start)/MIN_IN_S))
	
	# move point data from PLY to binary file
	# ---------------------------------------
	def store_p_data( ply_file, nelems, offset, format ):
		ply_file.seek( offset, 0 )
		p_data = open( PLY_PATH[:-4]+"_DATA.bin", 'wb' )
		for _ in range(nelems):
			if format == '':
				# load point from ascii context
				data = ply_file.readline().split(b' ')
				p_data.write(np.float32(data[0]))
				p_data.write(np.float32(data[1]))
				p_data.write(np.float32(data[2]))
			else:
				# load point from binary context
				p_data.write(np.frombuffer(ply_file.read(4),dtype=format+'f')[0])
				p_data.write(np.frombuffer(ply_file.read(4),dtype=format+'f')[0])
				p_data.write(np.frombuffer(ply_file.read(4),dtype=format+'f')[0])
		ply_file.close()
	p_data_thread = threading.Thread(target=store_p_data,
		args=(PLY_IN,nelems,ply_vertices_offset,format))
	p_data_thread.start()
	
	# merge all chunks together
	# -------------------------
	if MERGE_QUEUES[XDIM].qsize() > 1:
		
		if __debug__:
			time_start = time.time()
			log(O+"{0}\n MERGE ALL CHUNKS TOGETHER \n{0}\n".format('-'*27))

		xmerge = QueueHandler(XDIM)
		ymerge = QueueHandler(YDIM)
		zmerge = QueueHandler(ZDIM)
		xmerge.start() ; ymerge.start() ; zmerge.start()
		
		# wait for merger threads to finish the chunk queues
		if __debug__:
			while xmerge.is_alive() or ymerge.is_alive() or zmerge.is_alive():
				log("\r")
				for dim in [XDIM,YDIM,ZDIM]:
					log("%s%s "%(DIM_COLOR[dim],dim))
					for progress in MERGE_PROGRESS[dim].values():
						log(DIM_COLOR[dim]+"{: >4.1f}% ".format(progress*100))
				time.sleep(1)
			log("\n\n")
		else:	
			xmerge.join() ; ymerge.join() ; zmerge.join()
		
		if __debug__:
			time_end = time.time()
			log("Time: {}{:6.3f}m\n".format(O,(time_end-time_start)/MIN_IN_S))
	else:
		# no multiple chunks available - move and rename single chunk
		ifile = open(MERGE_QUEUES[XDIM].get(),'rb');
		ofile = open("%s_X.bin"%PLY_PATH[:-4],'wb')
		ifile.seek(0)
		ofile.write(np.frombuffer(ifile.read(4),dtype=np.uint32)[0])
		for i in range(nelems):
			ofile.write(np.frombuffer(ifile.read(4),dtype=np.uint32)[0])
			ifile.seek(12,1)
		ofile.close() ; ifile.close()
		os.remove(ifile.name)
		
		ifile = open(MERGE_QUEUES[YDIM].get(),'rb');
		ofile = open("%s_Y.bin"%PLY_PATH[:-4],'wb')
		ifile.seek(0)
		ofile.write(np.frombuffer(ifile.read(4),dtype=np.uint32)[0])
		for i in range(nelems):
			ofile.write(np.frombuffer(ifile.read(4),dtype=np.uint32)[0])
			ifile.seek(12,1)
		ofile.close() ; ifile.close()
		os.remove(ifile.name)
		
		ifile = open(MERGE_QUEUES[ZDIM].get(),'rb');
		ofile = open("%s_Z.bin"%PLY_PATH[:-4],'wb')
		ifile.seek(0)
		ofile.write(np.frombuffer(ifile.read(4),dtype=np.uint32)[0])
		for i in range(nelems):
			ofile.write(np.frombuffer(ifile.read(4),dtype=np.uint32)[0])
			ifile.seek(12,1)
		ofile.close() ; ifile.close()
		os.remove(ifile.name)
		
	# clean up tmp folder
	# -------------------
	for file in os.listdir('tmp'):
		os.unlink('tmp/'+file)
		# this should never be called as each merge thread calls os.unlink()
		# on merged chunks to remove them
		print(R+"%s wasn't properly removed from tmp/ folder\n"%file)
	os.rmdir('tmp')
	
	# wait for P_DATA to be written
	if p_data_thread.is_alive():
		if __debug__: log("Waiting for point data to be written to binary file...\n\n")
		p_data_thread.join()



def write_sorted_chunks(points):

	global FILE_CTR
	
	for dim in [XDIM,YDIM,ZDIM]:
	
		# sort arrays by points dim-value
		if __debug__:
			log("sort indices by %s%s-axis\t\t"%(DIM_COLOR[dim],dim))
			time_start = time.time()
			
		# transform deque objects to lists
		points	= list(points)
		#points, indices = [list(t) for t in zip(*sorted(zip(points, indices),
		#	key = lambda e: e[0][DIM_P_IDX[dim]]))]
		points.sort(key=lambda XDIM: XDIM[DIM_P_IDX[dim]])
		
		if __debug__:
			time_end = time.time()
			log(O+" {:6.3f}s\n".format(time_end-time_start))
			
		# create chunk file
		chunk = open("tmp/%s_chunk%d.bin"%(dim,FILE_CTR),'wb+')
		
		if __debug__:
			log("write data to %s%s\t"%(DIM_COLOR[dim],chunk.name))
			time_start = time.time()
			
		# write number of elements
		chunk.write(np.uint32(len(points)))
		
		# write sorted index list including point data
		for p in points:
			chunk.write(np.uint32(p[3]))
			chunk.write(np.float32(p[0]))
			chunk.write(np.float32(p[1]))
			chunk.write(np.float32(p[2]))
			
		if __debug__:
			time_end = time.time()
			log(O+" {:6.3f}s\n".format(time_end-time_start))
			
		# append chunk to merge sort queue
		MERGE_QUEUES[dim].put(chunk.name)
		chunk.close()
		del chunk
		
	del points
	FILE_CTR += 1
		

""" Worker thread to start new Merger() 
as long as there are more than one chunk in queue """		
class QueueHandler(threading.Thread):

	def __init__(self,dim):
		super(QueueHandler,self).__init__()
		self.daemon = True
		self.dim 	= dim
		self.ctr 	= FILE_CTR
		self.st_ctr = 0
	
	def run(self):
	
		result_queue = queue.Queue()
		
		while True:
			while MERGE_QUEUES[self.dim].qsize() > 2:
				a = MERGE_QUEUES[self.dim].get() 
				b = MERGE_QUEUES[self.dim].get()
				merge_res = open("tmp/%s_chunk%d.bin"%(self.dim,self.ctr), 'wb+')
				self.ctr += 1
				st = Merger(a,b,self.dim,merge_res,result_queue)
				st.name = merge_res.name
				st.start()
				self.st_ctr += 1
			if self.st_ctr > 0:
				# wait for the next subthread to post a merge result into queue
				merge_res_name = result_queue.get(block=True)
				self.st_ctr -= 1
				MERGE_QUEUES[self.dim].put(merge_res_name)
			else:
				# no more open subthreads. Merge last two chunks.
				a = MERGE_QUEUES[self.dim].get()
				b = MERGE_QUEUES[self.dim].get()
				merge_res = open(sys.argv[1][:-4]+'_%s.bin'%self.dim, 'wb+')
				st = Merger(a,b,self.dim,merge_res,result_queue)
				st.name = merge_res.name
				st.start()
				st.join()
				break


""" Worker thread that combines two chunks into one """				
class Merger( threading.Thread):

	def __init__( self, a, b, dim, merge_res, result_queue):
		super( Merger,self).__init__()
		self.a = open( a, 'rb')
		self.b = open( b, 'rb')
		self.dim = dim
		# file to write the merge result to
		self.merge_res = merge_res
		# used to hand over the finished merge file to parent process
		self.result_queue = result_queue
		if __debug__:
			self.points_written = 0
	
	def run( self):		
		# store sum of to be merged elements at the beginning of chunk_c
		size_a = np.frombuffer( self.a.read(4), dtype = np.uint32)[0]
		size_b = np.frombuffer( self.b.read(4), dtype = np.uint32)[0]
		size_c = size_a + size_b
		self.merge_res.write( np.uint32( size_c))
		# now all file pointer are exactly behind the number of elements

		# load the first two indices as buffer objects
		# those objects have size 0 when end of file is reached
		index_a = np.frombuffer( self.a.read(4), dtype = np.uint32)
		index_b = np.frombuffer( self.b.read(4), dtype = np.uint32)
		
		self.a.seek( BYTE_OFFSET[self.dim], 1)
		self.b.seek( BYTE_OFFSET[self.dim], 1)

		point_a = np.frombuffer( self.a.read(4), dtype = np.float32)
		point_b = np.frombuffer( self.b.read(4), dtype = np.float32)
		# move to next index position
		self.a.seek( 8-BYTE_OFFSET[self.dim], 1)
		self.b.seek( 8-BYTE_OFFSET[self.dim], 1)

		while index_a.size > 0 and index_b.size > 0:
			
			if point_a[0] <= point_b[0]:
				# write index_a to chunk_c
				self.merge_res.write( index_a[0])
				if not self.i_only: self.merge_res.write( point_a[0])
				# load next index_a
				index_a = np.frombuffer( self.a.read(4), dtype = np.uint32)
				self.a.seek( BYTE_OFFSET[self.dim], 1)
				point_a = np.frombuffer( self.a.read(4), dtype = np.float32)
				# move to next index position
				self.a.seek( 8-BYTE_OFFSET[self.dim], 1)
			else:
				# write index_b to chunk_c
				self.merge_res.write( index_b[0])
				if not self.i_only: self.merge_res.write( point_b[0])
				# load next index_b
				index_b = np.frombuffer( self.b.read(4), dtype = np.uint32)
				self.b.seek( BYTE_OFFSET[self.dim], 1)
				point_b = np.frombuffer( self.b.read(4), dtype = np.float32)
				# move to next index position
				self.b.seek( 8-BYTE_OFFSET[self.dim], 1)

			if __debug__:
				self.points_written += 1
				MERGE_PROGRESS[self.dim][self.name] = self.points_written / size_c
		
		# continue to write remaining indices for either chunk_a or _b
		while index_a.size > 0:
			self.merge_res.write(index_a[0])
			if not self.i_only: self.merge_res.write(point_a[0])
			index_a = np.frombuffer(self.a.read(4), dtype = np.uint32)
			self.a.seek(BYTE_OFFSET[self.dim],1)
			point_a = np.frombuffer(self.a.read(4), dtype = np.float32)
			# move to next index position
			self.a.seek(8-BYTE_OFFSET[self.dim],1)

			if __debug__:
				self.points_written += 1
				MERGE_PROGRESS[self.dim][self.name] = self.points_written / size_c
				
		while index_b.size > 0:
			self.merge_res.write(index_b[0])
			if not self.i_only: self.merge_res.write(point_b[0])
			index_b = np.frombuffer(self.b.read(4), dtype = np.uint32)
			self.b.seek(BYTE_OFFSET[self.dim],1)
			point_b = np.frombuffer(self.b.read(4), dtype = np.float32)
			# move to next index position
			self.b.seek(8-BYTE_OFFSET[self.dim],1)

			if __debug__:
				self.points_written += 1
				MERGE_PROGRESS[self.dim][self.name] = self.points_written / size_c
			
		self.a.close() ; self.b.close()
		os.unlink(self.a.name) ; os.unlink(self.b.name)
		del self.a ; del self.b
		
		if __debug__: del MERGE_PROGRESS[self.dim][self.name]
		
		self.merge_res.close()
		if not self.i_only: self.result_queue.put(self.merge_res.name)
		del self.merge_res

		
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
	parser = argparse.ArgumentParser(description = "[Out-of-Core Point Sorting]\nTakes a PLY file as input and sorts the contained data on all dimensional axes, storing all point data and sorted lists in BIN files afterwards.")
	
	# add arguments to the parser
	# ---------------------------
	parser.add_argument("-i","--input",
		help="input PLY-file")
	parser.add_argument("-m","--memcap", type=int, default=2,
		help="maximum amount of system RAM to be used in GB")
		
	# read arguments from command line
	# --------------------------------
	args = parser.parse_args()
	
	if args.input and args.memcap:
		
		PLY_PATH = args.input;
		MEMORY_CAP = int(args.memcap)*GB_IN_B
		
		if __debug__:
			log(O+"{0}\n START OOC-POINT-SORTING\n{0}\n".format('-'*25))
		
		# start main function
		# -------------------
		main_tstart = time.time()
		main()
		main_tend = time.time()

		if __debug__:
				# print total runtime in minutes
			log(O+"{0}\nTime total: {1:6.3f} min\n\n".format('-'*19,
				(main_tend-main_tstart)/MIN_IN_S))
				# stop Logger thread
			LOG_QUEUE.join()
		else:
			print(O+"{0}\nTime total: {1:6.3f} min\n\n".format('-'*19,
				(main_tend-main_tstart)/MIN_IN_S))
	else:
		print("missing arguments\n\tooc_point_sorting.py -i <plyfile> -m <memorycap_in_gigabyte>")
		sys.exit(2)
