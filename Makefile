TBB=1
EIGEN=1

VPATH=./src
SLIB=sfc.so
ALIB=sfc.a
EXEC=sfc
OBJDIR=./obj/

CC=g++
AR=ar
ARFLAGS=rcs
OPTS=-Ofast
LDFLAGS= -lm
COMMON= -Isrc/
CFLAGS= -w -std=c++11 -pthread -fPIC

COMMON= -ID:/libraries/boost_1_66_0/ -ID:/libraries/CGAL-4.10.2/include/ -ID:/libraries/CGAL-4.10.2/build/include/ -ID:/libraries/CGAL-4.10.2/auxiliary/gmp/include/
LDFLAGS+= -LD:/libraries/boost_1_66_0/stage/lib_64/ -LD:/libraries/CGAL-4.10.2/build/lib/Release/ -LD:/libraries/CGAL-4.10.2/auxiliary/gmp/lib/ 

CFLAGS+=$(OPTS)

ifeq ($(TBB), 1) 
COMMON+= -ID:/libraries/TBB/include/
LDFLAGS+= -LD:/libraries/TBB/lib/intel64/vc14/
endif

ifeq ($(EIGEN), 1) 
COMMON+=  -ID:/libraries/Eigen/
endif

OBJ=diagram_types.o power_diagram.o power_diagram_paint.o power_crust.o sfc.o
EXECOBJA=sfc.o

LDFLAGS+= -lstdc++

EXECOBJ = $(addprefix $(OBJDIR), $(EXECOBJA))
OBJS = $(addprefix $(OBJDIR), $(OBJ))

all: obj $(SLIB) $(ALIB) $(EXEC)

$(EXEC): $(EXECOBJ) $(ALIB)
	$(CC) $(COMMON) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(ALIB)

$(ALIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $^

$(SLIB): $(OBJS)
	$(CC) $(CFLAGS) -shared $^ -o $@ $(LDFLAGS)

$(OBJDIR)%.o: %.cpp 
	$(CC) $(COMMON) $(CFLAGS) -c $< -o $@

obj:
	mkdir -p obj
	
.PHONY: clean

clean:
	rm -rf $(OBJS)