CC=g++-10
FLAGS=-std=c++2a -Wno-narrowing -Werror

linker.o: linker.cpp linker.hpp
	$(CC) $(FLAGS) linker.cpp -o linker.o -c

test/linker_test.o: test/linker_test.cpp
	$(CC) $(FLAGS) test/linker_test.cpp -o test/linker_test.o -c -I.

test/linker_test: linker.o test/linker_test.o
	$(CC) $(FLAGS) linker.o test/linker_test.o -o test/linker_test

.PHONY: clean
clean:
	rm *.o test/*.o
