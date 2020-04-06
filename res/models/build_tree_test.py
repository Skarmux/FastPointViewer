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

def tree_test(tree, tree_offset, data, last_x_cut, last_y_cut, last_z_cut, max_depth):
	
	# read current node information
	tree.seek(tree_offset, 0)
	flag = np.frombuffer(tree.read(4), dtype = np.uint32)[0]
	if flag == 0:
		# is leaf node (list of elements)
		list_size = np.frombuffer(tree.read(4), dtype = np.uint32)[0]
		for i in range(0,list_size):
			point_index = np.frombuffer(tree.read(4), dtype = np.uint32)[0]
			point_offset = point_index * 3
			data.seek(point_offset,0)
			point_x = np.frombuffer(data.read(4), dtype = np.float32)[0]
			point_y = np.frombuffer(data.read(4), dtype = np.float32)[0]
			point_z = np.frombuffer(data.read(4), dtype = np.float32)[0]
			
			if last_x_cut[0] != 0:
				if last_x_cut[1] == "LEFT" and point_x > last_x_cut[0]:
					print("-------------------------------------------")
					print("flag: %d"%flag)
					print("point_index: %d"%point_index)
					print("Assertion failure! (X)")
				elif last_x_cut[1] == "RIGHT" and point_x <= last_x_cut[0]:
					print("-------------------------------------------")
					print("flag: %d"%flag)
					print("point_index: %d"%point_index)
					print("Assertion failure! (X)")
				
			if last_y_cut[0] != 0:
				if last_y_cut[1] == "LEFT" and point_y > last_y_cut[0]:
					print("-------------------------------------------")
					print("flag: %d"%flag)
					print("point_index: %d"%point_index)
					print("Assertion failure! (Y)")
				elif last_y_cut[1] == "RIGHT" and point_y <= last_y_cut[0]:
					print("-------------------------------------------")
					print("flag: %d"%flag)
					print("point_index: %d"%point_index)
					print("Assertion failure! (Y)")
					
			if last_z_cut[0] != 0:
				if last_z_cut[1] == "LEFT" and point_z > last_z_cut[0]:
					print("-------------------------------------------")
					print("flag: %d"%flag)
					print("point_index: %d"%point_index)
					print("Assertion failure! (Z)")
				elif last_z_cut[1] == "RIGHT" and point_z <= last_z_cut[0]:
					print("-------------------------------------------")
					print("flag: %d"%flag)
					print("point_index: %d"%point_index)
					print("Assertion failure! (Z)")
					
	else:
		# is node
		point_index 	= np.frombuffer(tree.read(4), dtype = np.uint32)[0]
		left_offset 	= np.frombuffer(tree.read(4), dtype = np.uint32)[0]
		right_offset 	= np.frombuffer(tree.read(4), dtype = np.uint32)[0]
		
		point_offset = point_index *12
		
		#print("point_offset: %d"%point_offset)
		
		data.seek(point_offset, 0)
		point_x = np.frombuffer(data.read(4), dtype = np.float32)[0]
		point_y = np.frombuffer(data.read(4), dtype = np.float32)[0]
		point_z = np.frombuffer(data.read(4), dtype = np.float32)[0]
		
		print("(%f,%f,%f)"%(point_x,point_y,point_z))
		
		# check value for if it is correctly positionied
		
		if last_x_cut[0] != 0:
			if last_x_cut[1] == "LEFT" and point_x > last_x_cut[0]:
				print("-------------------------------------------")
				print("flag: %d"%flag)
				print("point_index: %d"%point_index)
				print("left_offset: %d"%left_offset)
				print("right_offset: %d"%right_offset)
				print("Assertion failure! (X)")
			elif last_x_cut[1] == "RIGHT" and point_x <= last_x_cut[0]:
				print("-------------------------------------------")
				print("flag: %d"%flag)
				print("point_index: %d"%point_index)
				print("left_offset: %d"%left_offset)
				print("right_offset: %d"%right_offset)
				print("Assertion failure! (X)")
				
		if last_y_cut[0] != 0:
			if last_y_cut[1] == "LEFT" and point_y > last_y_cut[0]:
				print("-------------------------------------------")
				print("flag: %d"%flag)
				print("point_index: %d"%point_index)
				print("left_offset: %d"%left_offset)
				print("right_offset: %d"%right_offset)
				print("Assertion failure! (Y)")
			elif last_y_cut[1] == "RIGHT" and point_y <= last_y_cut[0]:
				print("-------------------------------------------")
				print("flag: %d"%flag)
				print("point_index: %d"%point_index)
				print("left_offset: %d"%left_offset)
				print("right_offset: %d"%right_offset)
				print("Assertion failure! (Y)")
				
		if last_z_cut[0] != 0:
			if last_z_cut[1] == "LEFT" and point_z > last_z_cut[0]:
				print("-------------------------------------------")
				print("flag: %d"%flag)
				print("point_index: %d"%point_index)
				print("left_offset: %d"%left_offset)
				print("right_offset: %d"%right_offset)
				print("Assertion failure! (Z)")
			elif last_z_cut[1] == "RIGHT" and point_z <= last_z_cut[0]:
				print("-------------------------------------------")
				print("flag: %d"%flag)
				print("point_index: %d"%point_index)
				print("left_offset: %d"%left_offset)
				print("right_offset: %d"%right_offset)
				print("Assertion failure! (Z)")
					
		# update last_cutdim value
		if max_depth > 0:
			if flag == 1: # x
				tree_test(tree, left_offset, data, (point_x,"LEFT"), last_y_cut, last_z_cut, max_depth-1)
				tree_test(tree, right_offset, data, (point_x,"RIGHT"), last_y_cut, last_z_cut, max_depth-1)
			elif flag == 2: # y
				tree_test(tree, left_offset, data, last_x_cut, (point_y,"LEFT"), last_z_cut, max_depth-1)
				tree_test(tree, right_offset, data, last_x_cut, (point_y,"RIGHT"), last_z_cut, max_depth-1)
			elif flag == 3: # z
				tree_test(tree, left_offset, data, last_x_cut, last_y_cut, (point_z,"LEFT"), max_depth-1)
				tree_test(tree, right_offset, data, last_x_cut, last_y_cut, (point_z,"RIGHT"), max_depth-1)

if __name__ == '__main__':

	data = open("lucy_DATA.bin",'rb')
	tree = open("lucy_KDTREE.bin",'rb')
	
	tree_test(tree, 0, data, (0,""),(0,""),(0,""), 16)
	
	data.close()
	tree.close()
