# Makefile

LOCALDIRROOT = /homes/bcdandurand/comp_infrastructure/DSP

SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic
CPLEXDIR      = /homes/bcdandurand/comp_infrastructure/CPLEX/cplex
CONCERTDIR    = /homes/bcdandurand/comp_infrastructure/CPLEX/concert
OPENMPIDIR = /usr/lib/openmpi/lib
DSPDIR = /homes/bcdandurand/comp_infrastructure/DSP
PSCGDIR = /homes/bcdandurand/comp_infrastructure/PSCG


SRC = .
OBJDIR = .
OBJS = $(OBJDIR)/DWBBMain.o  
BIN = .


# C++ Compiler command
#CXX = g++
CXX = mpicxx 

# C++ Compiler options
CXXFLAGS = -O3 -g -std=c++11 -fpic -DDSP_HAS_MPI
#CXXFLAGS = -g -O3 -pipe -DNDEBUG -pedantic-errors -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wconversion -Wno-unknown-pragmas -Wno-long-long   -DBCPS_BUILD 
#CXXFLAGS = -O3 

# Stochastic data directory

# additional C++ Compiler options for linking
CXXLINKFLAGS =  -Wl,--rpath -Wl,$(DSPDIR)/build/lib -Wl,--rpath -Wl,$(ALPSDIR)/lib 
#CXXLINKFLAGS =  -Wl,--rpath -Wl,$(DSPDIR)/build/lib -Wl,--rpath -Wl,$(ALPSDIR)/lib -Wl,--rpath -Wl,$(PSCGDIR)/lib

#INCLPSCGBB = -I ./include
INCLDSP = -I$(DSPDIR)/src -I$(DSPDIR)/build/include/coin -I$(DSPDIR)/build/include/ooqp -I$(DSPDIR)/build/include/pscg -I$(DSPDIR)/src/Model -I$(DSPDIR)/src/TreeSearch -I$(DSPDIR)/src/Solver -I$(DSPDIR)/src/Utility
INCLCPLEX = -I$(CPLEXDIR)/include -I$(CONCERTDIR)/include 
INCLMPI = -I$(OPENMPIDIR)/include
#INCLPSCG = -I$(PSCGDIR)/src

INCL = $(INCLDSP) $(INCLCPLEX)
#INCL = $(INCLPSCGBB) $(INCLPSCG) $(INCLDSP) $(INCLSMI) $(INCLALPS) $(INCLCPLEX)
#INCL = -I$(SMIDIR)/include/coin -I$(CPLEXDIR)/include -I$(CONCERTDIR)/include $(INCLDSP)
#INCL += $(ADDINCFLAGS)

#DSPLIBS = -L$(DSPDIR)/build/lib -lDsp -D_GLIBCXX_USE_CXX11_ABI=0
DSPLIBS = -L$(DSPDIR)/build/lib -lDsp 
#DSPLIBS =  

#COINORLIBS = -L$(SMIDIR)/lib -lSmi -lOsiCpx -lClp -lClpSolver -lCoinUtils -lOsi -lOsiClp -lOsiCommonTests  
#ALPSLIBS = -L$(ALPSDIR)/lib -lAlps -lBcps 
#ALPSLIBS = -L$(ALPSDIR)/lib -lBcps 

CPLEXLIBR = -DIL_STD $(CPPFLAGS) $(LDFLAGS) -L$(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT) -lilocplex -lcplex -L$(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT) -lconcert 

PSCGLIBR = 
#-L$(PSCGDIR)/lib -lPSCG

OTHERLIBS = -lstdc++ -lm -lpthread -lz -lbz2
#OTHERLIBS = -lstdc++ -lgcc_s -lc -lgcc -lm -lpthread

#PSCGModel.cpp  PSCGModel.h  PSCGModelScen.cpp  PSCGModelScen.h	PSCGParams.h
ALG = dwbb
ALG2 = pscgbb
#SOLVER = CPLEXsolverSCG
#SOLVER = PSCGModelScen

#SRCFILES = $(SRC)/PSCGMain.cpp $(SRC)/$(ALG).cpp $(SRC)/$(ALG).h $(SRC)/$(SOLVER).cpp $(SRC)/$(SOLVER).h $(SRC)/PSCGNodeDesc.h $(SRC)/PSCGTreeNode.cpp $(SRC)/PSCGTreeNode.h  
SRCFILES = $(SRC)/DWBBMain.cpp   
	
	
all: all_dwbb all_pscgbb 
#all_serial	

all_dwbb: $(ALG)

all_pscgbb: $(ALG2)

#mpicxx -o $(BIN)/$(ALG) $(OBJDIR)/PSCGMain.o  $(CPLEXLIBR) $(COINORLIBS) $(DSPLIBS) $(ALPSLIBS) $(PSCGLIBR) $(OTHERLIBS) $(CXXFLAGS) $(CXXLINKFLAGS) -D USING_MPI 
#$(CXX) -c -o $(OBJDIR)/$(SOLVER).o $(SRC)/$(SOLVER).cpp $(CPLEXLIBR) $(OTHERLIBS) $(CXXFLAGS) $(INCL)
#$(CXX) -c -o $(OBJDIR)/$(SOLVER)_serial.o $(SRC)/$(SOLVER).cpp $(CPLEXLIBR) $(OTHERLIBS) $(CXXFLAGS) $(INCL)

$(ALG): $(SRCFILES) 
	$(CXX) -c -o $(OBJDIR)/DWBBMain.o $(INCLMPI) $(INCL) $(SRC)/DWBBMain.cpp $(DSPLIBS) $(CPLEXLIBR) $(OTHERLIBS) $(CXXFLAGS) 
	
	mpicxx -o $(BIN)/$(ALG) $(OBJS) $(DSPLIBS) $(CPLEXLIBR) $(OTHERLIBS) $(CXXFLAGS) $(CXXLINKFLAGS)
	
$(ALG2): $(SRC)/PSCGBBMain.cpp
	$(CXX) -c -o $(OBJDIR)/PSCGBBMain.o $(INCLMPI) $(INCL) $(SRC)/PSCGBBMain.cpp $(DSPLIBS) $(CPLEXLIBR) $(PSCGLIBR) $(OTHERLIBS) $(CXXFLAGS)
	
	mpicxx -o $(BIN)/$(ALG2) $(OBJDIR)/PSCGBBMain.o $(DSPLIBS) $(PSCGLIBR) $(CPLEXLIBR) $(OTHERLIBS) $(CXXFLAGS) $(CXXLINKFLAGS) 

#	$(CXX) -c -o $(OBJDIR)/$(ALG)_serial.o $(SRC)/$(ALG).cpp $(CPLEXLIBR) $(DSPLIBS) $(ALPSLIBS) $(OTHERLIBS) $(CXXFLAGS) $(INCL)
#	
#	$(CXX) -c -o $(OBJDIR)/PSCGTreeNode_serial.o $(INCL) $(SRC)/PSCGTreeNode.cpp $(CPLEXLIBR) $(DSPLIBS) $(ALPSLIBS) $(OTHERLIBS) $(CXXFLAGS) 
#	
#	$(CXX) -o $(BIN)/$(ALG)_serial $(OBJS_serial) $(CPLEXLIBR) $(COINORLIBS) $(DSPLIBS) $(ALPSLIBS) $(PSCGLIBR) $(OTHERLIBS) $(CXXFLAGS) $(CXXLINKFLAGS) 

clean:
	rm -f $(BIN)/$(ALG) $(BIN)/$(ALG2) $(OBJDIR)/*.o
