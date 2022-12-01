default: all

CFLAGS := -I./../include -I./../../include -Wall
CC := g++

BINARIES := dynamixel-ach-client-template
all : $(BINARIES)

LIBS := -lrt -lstdc++ -lach

$(BINARIES): src/main.o
	gcc -o $@ $< $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

