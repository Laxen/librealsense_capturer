// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Struct for managing rotation of pointcloud view
struct state {
	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
	ml(false), offset_x(0.0f), offset_y(0.0f), tex() {}
	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; texture tex;
};

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, rs2::points& points);
void write_to_pcd(std::vector<pcl::PointXYZRGBA> cloud_points);

bool save = false;
std::string save_path = "/home/robot/AA/share_files/";
int cloud_idx = 0;

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense Pointcloud Example");
	// Construct an object to manage view state
	state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream (RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream (RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	// Start streaming with default recommended configuration
	rs2::pipeline_profile profile = pipe.start(cfg);
	float depth_scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();

	rs2_intrinsics color_K = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
	rs2_intrinsics depth_K = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

	while (app) // Application still alive?
	{
		rs2::frameset frames;
		bool successful = false;
		int unsuccessful_counter = 0;
		while(!successful) {
			try {
				frames = pipe.wait_for_frames(1000);
				successful = true;
			} catch(std::runtime_error e) {
				std::cout << "wait_for_frames() timed out" << std::endl;
				unsuccessful_counter++;

				if(unsuccessful_counter >= 6) {
					std::cout << "Streams are completely out of sync!" << std::endl;
				} else if(unsuccessful_counter >= 4) {
					std::cout << "Streams out of sync, resetting..." << std::endl;

					cfg.disable_stream(RS2_STREAM_COLOR, -1);
					pipe.stop();
					profile = pipe.start(cfg);

					cfg.enable_stream (RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
				} else if(unsuccessful_counter >= 2) {
					std::cout << "Streams out of sync, resetting..." << std::endl;

					cfg.disable_stream(RS2_STREAM_DEPTH, -1);
					pipe.stop();
					profile = pipe.start(cfg);

					cfg.enable_stream (RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
				}

				pipe.stop();
				profile = pipe.start(cfg);
				depth_scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
				color_K = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
				depth_K = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
			}	
		}

		rs2::align align(RS2_STREAM_COLOR);
		rs2::frameset processed_frames = align.proccess(frames);
		const uint16_t* depth_data = reinterpret_cast<const uint16_t*> (processed_frames.get_depth_frame().get_data());
		const uint8_t* color_data = reinterpret_cast<const uint8_t*> (processed_frames.get_color_frame().get_data());
		float color_point[3];
		float scaled_depth;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());

		if(save) {
			for(int y = 0; y < 480; y++) {
				for(int x = 0; x < 640; x++) {
					uint16_t depth_value = depth_data[640*y + x];
					scaled_depth = depth_value * depth_scale;
					float color_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
					rs2_deproject_pixel_to_point(color_point, &color_K, color_pixel, scaled_depth);
					if (color_point[2] <= 0.1f || color_point[2] > 5.f) {
						continue;
					}
					
					pcl::PointXYZRGBA p;
					p.x = color_point[0];
					p.y = color_point[1];
					p.z = color_point[2];

					int i = static_cast<int>(color_pixel[0]);
					int j = static_cast<int>(color_pixel[1]);
					int offset = i*3 + j*color_K.width*3;

					p.r = static_cast<uint8_t>(color_data[offset]);
					p.g = static_cast<uint8_t>(color_data[offset + 1]);
					p.b = static_cast<uint8_t>(color_data[offset + 2]);
					cloud->push_back(p);
				}
			}

			std::ostringstream oss;
			oss << save_path << cloud_idx << ".pcd";
			std::cout << "Saving cloud in " << oss.str() << std::endl;
			pcl::io::savePCDFileBinaryCompressed(oss.str(), *cloud);
			save = false;
			cloud_idx++;
			std::cout << "Cloud saved!" << std::endl;
		}

		// Wait for the next set of frames from the camera
		rs2::depth_frame depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		rs2::video_frame color = frames.get_color_frame();

		// Tell pointcloud object to map to this color frame
		pc.map_to(color);

		// Upload the color frame to OpenGL
		app_state.tex.upload(color);

		// Draw the pointcloud
		draw_pointcloud(app, app_state, points);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

/*
void write_to_pcd(std::vector<pcl::PointXYZRGBA> cloud_points) {
	std::cout << "Saving cloud as " << save_idx << "..." << std::endl;

	std::ofstream ofs;
	std::ostringstream oss;
	oss << save_idx;
	ofs.open(oss.str() + ".pcd");

	ofs << "VERSION .7" << "\n";
	ofs << "FIELDS x y z rgb" << "\n";
	ofs << "SIZE 4 4 4 4" << "\n";
	ofs << "TYPE F F F U" << "\n";
	ofs << "COUNT 1 1 1 1" << "\n";
	ofs << "WIDTH " << cloud_points.size() << "\n";
	ofs << "HEIGHT 1" << "\n";
	ofs << "VIEWPOINT 0 0 0 1 0 0 0" << "\n";
	ofs << "POINTS " << cloud_points.size() << "\n";
	ofs << "DATA ascii" << "\n";
	
	for(int i = 0; i < cloud_points.size(); i++) {
		//cloud_point p = cloud_points[i];
		pcl::PointXYZRGBA p = cloud_points[i];
		int rgb = ((int)p.r) << 16 | ((int)p.g) << 8 | ((int)p.b);
		ofs << p.x << " " << p.y << " " << p.z << " " << rgb << "\n";
	}

	ofs.close();

	save = false;
	save_idx++;
	std::cout << "Cloud saved!" << std::endl;
}
*/

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
	app.on_left_mouse = [&](bool pressed)
	{
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset)
	{
		app_state.offset_x -= static_cast<float>(xoffset);
		app_state.offset_y -= static_cast<float>(yoffset);
	};

	app.on_mouse_move = [&](double x, double y)
	{
		if (app_state.ml)
		{
			app_state.yaw -= (x - app_state.last_x);
			app_state.yaw = std::max(app_state.yaw, -120.0);
			app_state.yaw = std::min(app_state.yaw, +120.0);
			app_state.pitch += (y - app_state.last_y);
			app_state.pitch = std::max(app_state.pitch, -80.0);
			app_state.pitch = std::min(app_state.pitch, +80.0);
		}
		app_state.last_x = x;
		app_state.last_y = y;
	};

	app.on_key_release = [&](int key)
	{
		if (key == 32) // Space
		{
			app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
		}
		else if(key == 257) // Enter
		{
			save = true;
		}
	};
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, rs2::points& points)
{
	if (!points)
		return;

	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

	glClearColor(153.f/ 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
	float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER); // GL_CLAMP_TO_BORDER
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER); // GL_CLAMP_TO_BORDER
	glBegin(GL_POINTS);


	/* this segment actually prints the pointcloud */
	auto vertices = points.get_vertices();              // get vertices
	auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
	for (int i = 0; i < points.size(); i++)
	{
		if (vertices[i].z)
		{
			// upload the point and texture coordinates only for points we have depth data for
			glVertex3fv(vertices[i]);
			glTexCoord2fv(tex_coords[i]);
		}
	}

	// OpenGL cleanup
	glEnd();
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}
