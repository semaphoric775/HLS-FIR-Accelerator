CXXFLAGS = -Wall -Wno-unknown-pragmas -std=c++11 -DHLS_CATAPULT -DCONNECTIONS_ACCURATE_SIM -DSC_INCLUDE_DYNAMIC_PROCESSES

EXE_NAME=main.x

all: rel

rel: OPTFLAGS = -O3

rel: $(EXE_NAME)

dbg: OPTFLAGS = -O0 -g

dbg: $(EXE_NAME)

.PHONY: dbg rel


CXXFLAGS+=$(OPTFLAGS)


SRC = $(wildcard *.cpp)
OBJ = $(addsuffix .o, $(basename $(SRC)))
INCLUDES = -I. -I$(SYSTEMC_HOME)/include -I$(BOOST_HOME)/include -I$(CATAPULT_HOME)/Mgc_home/shared/include -I$(MATCHLIB_HOME)/cmod/include
LIBS = -L. -L$(SYSTEMC_HOME)/lib-linux64 -L$(BOOST_HOME)/lib -lstdc++ -lsystemc -lm -lpthread -lboost_timer -lboost_chrono -lboost_system -L$(RISCV)/lib64 -Wl,-rpath,$(RISCV)/lib64 -lspike

REBUILDABLES=$(OBJ) $(EXE_NAME)

#   $@ target name, $^ target deps, $< matched pattern
$(EXE_NAME): $(OBJ) nvhls_pch.h.gch
	$(CXX) $(CXXFLAGS) $(LIBS) -o $@ $(OBJ)
	@echo "Built $@ successfully" 

nvhls_pch.h.gch: nvhls_pch.h
	$(CXX) -o nvhls_pch.h.gch $(CXXFLAGS) $(INCLUDES) -x c++-header -c nvhls_pch.h

#include the autogenerated dependency files for each .o file
-include $(OBJ:.o=.d)

# build dependency list via gcc -M and save to a .d file
%.d : %.cpp
	@$(CXX) -M $(CXXFLAGS) $(INCLUDES) $< > $@

# build all .cpp files to .o files
%.o : %.cpp nvhls_pch.h.gch
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

clean: 
	-rm -f $(REBUILDABLES) *.d nvhls_pch.h.gch

sim:
	./$(EXE_NAME)

gdb:
	gdb ./$(EXE_NAME)
