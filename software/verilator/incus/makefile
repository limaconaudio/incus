DEBUG?=no
TRACE?=no
PRINT_PERF?=no
TRACE_START=0
ADDCFLAGS += -CFLAGS -pthread -LDFLAGS -pthread


ifeq ($(TRACE),yes)
	VERILATOR_ARGS += --trace
	ADDCFLAGS += -CFLAGS -DTRACE
endif
ifeq ($(DEBUG),yes)
	ADDCFLAGS += -CFLAGS "-g3 -O0"
endif
ifneq ($(DEBUG),yes)
	ADDCFLAGS += -CFLAGS "-O3"
endif
ifeq ($(PRINT_PERF),yes)
	ADDCFLAGS += -CFLAGS -DPRINT_PERF
endif

ADDCFLAGS += -CFLAGS -DTRACE_START=${TRACE_START}



all: clean compile

run: compile
	./obj_dir/VIncus

verilate: ../../../Incus.v
	rm -f Incus.v*.bin
	cp ../../../Incus.v*.bin . | true
	verilator -I../../../.. -cc  ../../../Incus.v -CFLAGS -std=c++11  ${ADDCFLAGS} --gdbbt ${VERILATOR_ARGS} -Wno-WIDTH -Wno-UNOPTFLAT --x-assign unique --exe main.cpp
 	
compile: verilate
	make  -j  -C obj_dir/ -f VIncus.mk VIncus
 	
clean:
	rm -rf obj_dir
	rm -f Incus.v*.bin
	rm -f *.logTrace
 	
