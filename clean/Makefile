all:
	g++ -o cam -std=c++17 cam_opencv.cpp `pkg-config --cflags --libs opencv4` -lpthread
	./cam

clean: clear
	g++ -o clean -std=c++17 clean.cpp `pkg-config --cflags --libs opencv4` -lpthread
	./clean

clear:
	rm -rf cam clean