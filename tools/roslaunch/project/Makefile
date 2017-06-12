CXX = g++
CXXFLAGS = -O2 -Wall -std=c++11 -Iinclude
LDFLAGS =
 
EXE = main
SRCDIR = src
BINDIR = bin
OBJDIR = tests/bin
 
OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(BINDIR)/%.o,$(wildcard $(SRCDIR)/*.cpp))

all: $(EXE)
 
$(EXE): $(BINDIR) $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(EXE) $(LDFLAGS)
 
$(BINDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -MMD -o $@ $<
 
include $(wildcard $(BINDIR)/*.d)
 
$(OBJDIR):
	mkdir -p $(OBJDIR)

main_test: $(OBJDIR) tests/src/main_test.cpp
	g++ -O2 -Wall -std=c++11 tests/src/main_test.cpp -o main_test

val_check:
	g++ -I ./include -g $(CXXFLAGS) src/main.cpp
	valgrind --tool=memcheck --track-origins=yes --error-limit=no --leak-check=full --show-leak-kinds=all ./a.out roslaunch agitr test.launch 

val_check_test:
	g++ -I ./include -g $(CXXFLAGS) tests/src/main_test.cpp
	valgrind --tool=memcheck --track-origins=yes --error-limit=no --leak-check=full --show-leak-kinds=all ./a.out 
 	
clean_tests:
	rm -rf $(OBJDIR)

$(BINDIR):
	mkdir -p $(BINDIR)

clean:
	rm -rf $(BINDIR) $(OBJDIR) $(EXE) main_test
 
.PHONY: clean all
