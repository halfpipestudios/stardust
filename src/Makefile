# build the sample
build:
	cl \
	/std:c++17 /W2 /nologo /Od /Zi /EHsc \
	/I../include \
	.\main.cpp \
	/Fo../bin/ /Fe../bin/game /Fm../bin/ \
	/link stardust.lib \
	/LIBPATH:../bin \
	/SUBSYSTEM:CONSOLE


# build the core library
lib:
	cl /c \
	/std:c++17 /W2 /nologo /O2 /Zi /EHsc \
	/I../include /I../libs/stb_image /I../libs/assimp/include \
	.\sd_memory.cpp \
	.\sd_win32.cpp \
	.\sd_math.cpp \
	.\sd_input.cpp \
	.\sd_software_renderer.cpp \
	.\sd_mesh.cpp \
	.\sd_animation.cpp \
	/Fo../bin/ /Fm../bin/
	
	lib \
	/OUT:../bin/stardust.lib \
	../bin/sd_memory.obj \
	../bin/sd_win32.obj \
	../bin/sd_math.obj \
	../bin/sd_input.obj \
	../bin/sd_software_renderer.obj \
	../bin/sd_mesh.obj \
	../bin/sd_animation.obj \
	../libs/assimp/bin/assimp-vc143-mt.lib
	
	  
# run the sample
run:
	..\bin\game.exe