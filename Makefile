all:
	g++ -std=c++11 -o detect_objects move_detecting.cpp `pkg-config --cflags --libs opencv4`
	./detect_objects

trak:
	g++ -std=c++11 -o detect_objects traking_len.cpp `pkg-config --cflags --libs opencv4`
	./detect_objects