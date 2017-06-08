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
	
clean_tests:
	rm -rf $(OBJDIR)

$(BINDIR):
	mkdir -p $(BINDIR)

clean:
	rm -rf $(BINDIR) $(OBJDIR) $(EXE) main_test
 
.PHONY: clean all
