GDB=-ggdb 
GPROF=
CFLAGS=-std=c++11 $(GDB) $(GPROF)
OPENGL=-L/usr/local/lib -lGLEW -lGLU -lm -lglfw3 -lrt -lm -ldl -lXrandr -lXinerama -lXi -lXcursor -lXrender -lGL -lm -lpthread -ldl -ldrm -lXdamage -lXfixes -lX11-xcb -lxcb-glx -lxcb-dri2 -lXxf86vm -lXext -lX11 -lpthread -lxcb -lXau -lXdmcp
ARMA=-lblas -llapack -I /home/jhazelden/Downloads/armadillo-7.500.2/include -DARMA_DONT_USE_WRAPPER

physics : example_27_feb.o phys.o renderer.o base.o quaternion.o constraints.o bookKeeping.o logger.o runtimeOptions.o graphics_internal.o camera.o
	g++ example_27_feb.o phys.o renderer.o base.o quaternion.o constraints.o bookKeeping.o logger.o runtimeOptions.o graphics_internal.o camera.o -o physics -lblas -llapack $(CFLAGS) $(OPENGL) 
	
example_27_feb.o : example_27_feb.cpp ../../src/phys.cpp ../../src/renderer.cpp
	g++ -c example_27_feb.cpp $(ARMA) $(CFLAGS)

phys.o : ../../src/phys.cpp ../../src/quaternion.cpp ../../src/constraints.cpp
	g++ -c ../../src/phys.cpp $(ARMA) $(CFLAGS)

constraints.o : ../../src/constraints.cpp ../../src/phys.cpp ../../../SGV3D/src/logger.cpp
	g++ -c ../../src/constraints.cpp $(ARMA) $(CFLAGS)

quaternion.o : ../../src/quaternion.cpp
	g++ -c ../../src/quaternion.cpp $(ARMA) $(CFLAGS)

bookKeeping.o : ../../src/bookKeeping.cpp
	g++ -c ../../src/bookKeeping.cpp $(CFLAGS)
	
renderer.o : ../../src/renderer.cpp ../../../SGV3D/src/graphics_internal.cpp
	g++ -c ../../src/renderer.cpp $(ARMA) $(CFLAGS) 

base.o : ../../../SGV3D/src/base.cpp ../../../SGV3D/src/logger.cpp
	g++ -c ../../../SGV3D/src/base.cpp $(CFLAGS) 

logger.o : ../../../SGV3D/src/logger.cpp 
	g++ -c ../../../SGV3D/src/logger.cpp $(CFLAGS)

runtimeOptions.o : ../../../SGV3D/src/runtimeOptions.cpp
	g++ -c ../../../SGV3D/src/runtimeOptions.cpp $(CFLAGS)

graphics_internal.o : ../../../SGV3D/src/graphics_internal.cpp ../../../SGV3D/src/base.cpp
	g++ -c ../../../SGV3D/src/graphics_internal.cpp $(OPENGL) $(CFLAGS)

camera.o : ../../../SGV3D/src/camera.cpp ../../../SGV3D/src/base.cpp
	g++ -c ../../../SGV3D/src/camera.cpp $(CFLAGS)

clean : 
	rm *.o physics
