CC		= gcc
CPP	= g++
LD		= g++
CP		= cp
CFLAGS    = -std=c99 -g -I../include -I../libopeninv/include
CPPFLAGS    = -g -I../include -I../libopeninv/include
LDFLAGS     = -g
BINARY		= test_vcu
OBJS		= test_main.o my_string.o params.o throttle.o test_throttle.o
VPATH = ../src ../libopeninv/src

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS)

%.o: ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: ../%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY)
