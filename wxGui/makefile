app: app.cpp UIMyFrame1.cpp freeUI.cpp
	g++ UIMyFrame1.cpp `wx-config --cxxflags --libs` -c
	g++ freeUI.cpp `wx-config --cxxflags --libs` -c
	g++ app.cpp UIMyFrame1.o freeUI.o `wx-config --cxxflags --libs` -o app
