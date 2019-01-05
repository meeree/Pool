CFLAGS=-std=c++11  
GDB=-ggdb 
GPROF=
OPENGL=-L/usr/local/lib -lGLEW -lGLU -lm -lglfw3 -lrt -lm -ldl -lXrandr -lXinerama -lXi -lXcursor -lXrender -lGL -lm -lpthread -ldl -ldrm -lXdamage -lXfixes -lX11-xcb -lxcb-glx -lxcb-dri2 -lXxf86vm -lXext -lX11 -lpthread -lxcb -lXau -lXdmcp

run : sceneGraph.o base.o main.o logger.o runtimeOptions.o graphics.o
	g++ sceneGraph.o base.o main.o logger.o runtimeOptions.o graphics.o -o run $(CFLAGS) $(GPROF) $(GDB) $(OPENGL) 

sceneGraph.o : ../src2/sceneGraph.cpp ../src2/base.cpp ../src2/runtimeOptions.cpp
	g++ -c ../src2/sceneGraph.cpp $(CFLAGS) $(GPROF) $(GDB) 

base.o : ../src2/base.cpp ../src2/logger.cpp
	g++ -c ../src2/base.cpp $(CFLAGS) $(GPROF) $(GDB) 

main.o : ../src2/main.cpp ../src2/graphics.cpp ../src2/runtimeOptions.cpp
	g++ -c ../src2/main.cpp $(CFLAGS) $(GPROF) $(GDB) 

logger.o : ../src2/logger.cpp 
	g++ -c ../src2/logger.cpp $(CFLAGS) $(GPROF) $(GDB)

runtimeOptions.o : ../src2/runtimeOptions.cpp
	g++ -c ../src2/runtimeOptions.cpp $(CFLAGS) $(GPROF) $(GDB)

graphics.o : ../src2/graphics.cpp
	g++ -c ../src2/graphics.cpp $(OPENGL) $(CFLAGS) $(GPROF) $(GDB)

clean : 
	rm *.o run
