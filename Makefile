all:	compile

clean:
	rm *.o

compile: zdrojak.cpp
		g++ -Wall -pedantic -Ofast zdrojak.cpp -o zdrojak.o -g
