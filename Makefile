all:
	g++ -std=c++11 -o move_detect move_detecting.cpp `pkg-config --cflags --libs opencv4`
	./move_detect

trak:
	g++ -std=c++11 -o detect_objects traking_len.cpp `pkg-config --cflags --libs opencv4`
	./detect_objects

gst_flash:
	g++ -o test gst_opencv_test.cpp `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0` -lpthread
	./test

gst:
	g++ -std=c++11 -o test gst_camera_test.cpp `pkg-config --cflags --libs opencv4`
	./test

gstv:
	g++ -std=c++11 -o test gst_lib_test.cpp `pkg-config --cflags --libs opencv4`
	./test

flash:
	g++ -std=c++11 -o test flash_track.cpp `pkg-config --cflags --libs opencv4`
	./test

noui:
	g++ -std=c++11 -o test no_ui_flash.cpp `pkg-config --cflags --libs opencv4`
	./test

gr_mv:
	g++ -std=c++11 -o green_move green_move.cpp `pkg-config --cflags --libs opencv4`
	./green_move

green:
	g++ -std=c++17 -o web_green_detect web_green_detect.cpp `pkg-config --cflags --libs opencv4` -lpthread
	./web_green_detect

web:
	g++ -std=c++17 -o webcam_stream simple_web_server.cpp `pkg-config --cflags --libs opencv4` -lpthread
	./webcam_stream

color:
	g++ -std=c++11 -o test color_select.cpp `pkg-config --cflags --libs opencv4`
	./test