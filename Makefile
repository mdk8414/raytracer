program: raytracer.cpp raytracer.h BVH.h
	clang++ -I .. -I /opt/X11/include raytracer.cpp -L /opt/X11/lib -lX11 -lpthread -lpng -lz -o program

program0: raytracer0.cpp raytracer.h BVH.h
	clang++ -I .. -I /opt/X11/include raytracer0.cpp -L /opt/X11/lib -lX11 -lpthread -lpng -lz -o program0

program2: raytracer.cpp
	clang++ -O3 -I. rasterizer.cpp -o program

build: program

run: ./program
	./program $(file)

run0: ./program0
	./program0 $(file)

.PHONY: build, run, clean
clean:
	rm *.o program
