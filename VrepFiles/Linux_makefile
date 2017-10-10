CFLAGS = -Wall -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -fPIC
LDFLAGS = -lpthread -ldl

CFLAGS += -D__linux
LDFLAGS += -shared

all: 
	@rm -f lib/*.*
	@rm -f *.o
	gcc $(CFLAGS) -std=gnu99 -c vrep_main.c -o vrep_main.o 
	gcc $(CFLAGS) -c extApi.c -o extApi.o
	gcc $(CFLAGS) -c extApiPlatform.c -o extApiPlatform.o
	@mkdir -p lib
	gcc extApi.o extApiPlatform.o vrep_main.o -o lib/vrepClient -lpthread
