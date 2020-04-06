# bin/include/python3.7.3

"""
Bachelor Thesis
Nils Harbke
> Optimized Out-of-Core Rendering for Interactive
> Presentation of Large Point Clouds
HAW Hamburg
"""

import numpy as np 	# adds more control over binary size of variables

DIM_X, DIM_Y, DIM_Z = 0,1,2
P_OFFSET = 12 # bytes
DEBUG = False

if __name__ == '__main__':

	points = open("points.bin",'rb')
	x_sort = open("x_sort.bin",'rb')
	y_sort = open("y_sort.bin",'rb')
	z_sort = open("z_sort.bin",'rb')
	
	size_x = np.frombuffer(x_sort.read(4), dtype = np.uint32)[0]
	size_y = np.frombuffer(y_sort.read(4), dtype = np.uint32)[0]
	size_z = np.frombuffer(z_sort.read(4), dtype = np.uint32)[0]
	
	print("Check x_sort.bin")
	idx = np.frombuffer(x_sort.read(4), dtype = np.uint32)
	val = 0
	while idx.size > 0:
		val += idx[0]
		idx = np.frombuffer(x_sort.read(4), dtype = np.uint32)
	print("> Sum index %d"%val)
	
	val = 0
	for i in range(35947):
		val += np.uint32(i)
	print("> Check val = %d"%val)
	
	print("Check 0_chunk_0.bin")
	test = open("0_chunk_0.bin", 'rb')
	test_size = np.frombuffer(test.read(4), dtype = np.uint32)[0]
	idx = np.frombuffer(test.read(4), dtype = np.uint32)
	val = 0
	while idx.size > 0:
		val += idx[0]
		idx = np.frombuffer(test.read(4), dtype = np.uint32)
	print("> Sum index %d"%val)
	test.close()
	
	val = 0
	for i in range(30000):
		val += np.uint32(i)
	print("> Check val = %d"%val)	
	
	x_sort.close()
	y_sort.close()
	z_sort.close()
	points.close()
	
	
	# error checking
	# --------------------------------------------------------------------

	time_start = time.time()
	log(O+"\n{0}\n CHECKING FOR ERRORS \n{0}\n".format('-'*21))

	def test_sorted_list(dim):
		with open(PLY_PATH[:-4]+'_%s.bin'%dim, 'rb') as f:
			f_nelems = np.frombuffer(f.read(4), dtype=np.uint32)[0]
			if f_nelems != nelems:
				log(R+"\nWrong number of elements given! "
					+"Expected %d but was %d.\n"%(nelems,f_nelems))
			#last_point = np.float32('-Infinity')
			for i in range(f_nelems):
					# read index
				index = np.frombuffer(f.read(4), dtype=np.uint32)
				if index.size <= 0:
					log(R+"Point sorting failure! Not enough elements.")
					break
					# read point value
				#f.seek(BYTE_OFFSET[dim],1)
				#point = np.frombuffer(f.read(4), dtype=np.float32)[0]
				#f.seek(8-BYTE_OFFSET[dim],1)
				#if last_point > point:
				#	log(R+"\nPoint sorting failure! Point at i-1 is bigger!\n"+
				#		"\t(i-1) %d > %d (i)\n"%(last_point,point))
				#	break
				#else:
				#	last_point = point
				
	x_test = threading.Thread(target=test_sorted_list,args=(XDIM))
	x_test.start()

	y_test = threading.Thread(target=test_sorted_list,args=(YDIM))
	y_test.start()

	z_test = threading.Thread(target=test_sorted_list,args=(ZDIM))
	z_test.start()

	x_test.join() ; y_test.join() ; z_test.join()

	time_end = time.time()
	log("\nTime: {}{:6.3f}m\n\n".format(O,(time_end-time_start)/MIN_IN_S))