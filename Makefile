#Vars
CC=g++
CFLAGS=-std=c++11 -pthread
OBJECTS=user_proc oss
DEPS=structures.h

#Make both bin objects
all: $(OBJECTS)

user_proc:
	$(CC) $(CFLAGS) -o user_proc user_proc.cpp $(DEPS)

oss:
	$(CC) $(CFLAGS) -o oss oss.cpp $(DEPS)

#Clean
clean: 
	rm $(OBJECTS) 