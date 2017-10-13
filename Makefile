tourGuide: src/tourGuide.cpp
	g++ -Wall -o tourGuide src/tourGuide.cpp -lAria -ldl `pkg-config --cflags --libs opencv` -lpthread -L /usr/local/Aria/lib -I/usr/local/Aria/include
