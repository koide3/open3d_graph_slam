#!/usr/bin/python
import os
import csv
import math
import numpy
import open3d


def generate_pcd(path):
	setting = list(csv.reader(open(path + '/img.cfg', 'r'), delimiter=';', skipinitialspace=True))[1]

	h_start_angle = math.radians(float(setting[2]))
	h_end_angle = math.radians(float(setting[3]))
	v_angles = numpy.radians(numpy.float32(setting[4:]))

	image_files = sorted([path + '/' + x for x in os.listdir(path) if '.png' in x])

	vecs = []
	for row in range(int(setting[1])):
		for col in range(int(setting[0])):
			yaw_step = (h_end_angle - h_start_angle) / int(setting[0])
			yaw = h_start_angle + col * yaw_step
			pitch = v_angles[row]

			vecs.append(yawpitch2_vec(pitch, yaw))
	vecs = numpy.float32(vecs)

	for image_file in image_files:
		print image_file
		image = numpy.asarray(open3d.read_image(image_file)).flatten()

		points = (vecs.T * (image / 500.0).flatten()).T

		cloud = open3d.geometry.PointCloud()
		cloud.points = open3d.Vector3dVector(points)
		open3d.estimate_normals(cloud, search_param=open3d.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))

		dst_filename = image_file[:-4] + '.pcd'
		open3d.write_point_cloud(dst_filename, cloud)


def yawpitch2_vec(yaw, pitch):
	c_a = math.cos(yaw)
	s_a = math.sin(yaw)
	c_b = math.cos(pitch)
	s_b = math.sin(pitch)

	R = numpy.float32([[c_a * c_b, s_a * c_b, -s_a], [-s_a, c_a, 0.0], [c_a * s_b, s_a * s_b, c_b]])

	return R.dot([1, 0, 0])


def main():
	generate_pcd('/home/koide/datasets/kit/scenario1')

if __name__ == '__main__':
	main()
