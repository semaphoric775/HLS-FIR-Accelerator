#
# Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
# 
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


# Location of submodules
RAPIDJSON_HOME ?= $(MATCHLIB_HOME)/rapidjson

export TOP_NAME       := firUnit
export CLK_PERIOD     := 5
export ROOT           := ..
export SRC_PATH       := $(ROOT)/sc
export SEARCH_PATH    ?= $(MATCHLIB_HOME)/cmod $(MATCHLIB_HOME)/cmod/include $(RAPIDJSON_HOME)/include $(BOOST_HOME)/include
export HLS_CATAPULT   ?= 1
export RUN_SCVERIFY   ?= 0
export SYSTEMC_DESIGN ?= 1

export RUN_CDESIGN_CHECKER	?= 0

# Default compiler flags set by switches below.
export COMPILER_FLAGS ?=

# SIM_MODE (SystemC code, RTL is unaffected)
# 0 = Synthesis view of Connections port and combinational code.
#   This option can cause failed simulations due to SystemC's timing model.
# 1 = Cycle-accurate view of Connections port and channel code, CONNECTIONS_ACCURATE_SIM. (default)
# 2 = Faster TLM view of Connections port and channel code, CONNECTIONS_FAST_SIM.
SIM_MODE ?= 1
ifeq ($(SIM_MODE),1)
	COMPILER_FLAGS += CONNECTIONS_ACCURATE_SIM SC_INCLUDE_DYNAMIC_PROCESSES
endif
ifeq ($(SIM_MODE),2)
	COMPILER_FLAGS += CONNECTIONS_FAST_SIM SC_INCLUDE_DYNAMIC_PROCESSES
endif


# RAND_STALL (SystemC code, RTL is unaffected)
# 0 = Random stall of ports and channels disabled (default)
# 1 = Random stall of ports and channels enabled
#   This feature aids in latency insensitive design verication.
#   Note: Only valid if SIM_MODE = 1 (accurate) or 2 (fast)
RAND_STALL ?= 0
ifeq ($(RAND_STALL),1)
	COMPILER_FLAGS += CONN_RAND_STALL
endif

export FSDB_VCS_RD_SC_VALPTR_PROTECT=1

hls:
	date +%s > hls.begin
	catapult -shell -product ultra -file go_hls.tcl -logfile catapult_hls.log
	python3 parse_reports.py $(TOP_NAME) $(CLK_PERIOD)

shell:
	catapult -shell -product ultra

gui:
	catapult -product ultra

cdc:
	env RUN_CDESIGN_CHECKER=1 catapult -shell -product ultra -file go_hls.tcl
	$(ROOT)/hls/design_checker_summary.py
	cat DesignCheckSummary.csv

synth: hls
	date +%s > synth.begin
	sed s/NangateOpenCellLibrary_typical_ccs.db/NanGate_15nm_OCL_slow_conditional_nldm.db/ Catapult/$(TOP_NAME).v1/rtl.v.dc | sed s/vhdl/verilog/ | sed s/dc.v.vhdl/dc.v/ > Catapult/$(TOP_NAME).v1/rtl.v.dc.mod
	echo 'echo [exec date +%s] > synth' >> Catapult/$(TOP_NAME).v1/rtl.v.dc.mod
	echo exit >> Catapult/$(TOP_NAME).v1/rtl.v.dc.mod
	dc_shell-t -f Catapult/$(TOP_NAME).v1/rtl.v.dc.mod |& tee run_synth.log

.PHONY: clean
clean:
	-rm -rf ./catapult_cache
	-rm ./*~
	-rm ./*.key
	-rm ./core*
	-rm -rf ./Catapult*
	-rm -rf ./catapult*
	-rm ./*.log
	-rm ./design_checker_*.tcl
	-rm hls.begin
	-rm hls

setup:
	echo date__begin,date__end,module_name,clk_per,realops,latency,throughput,critpath,area_mux,area_func,area_logic,area_buffer,area_mem,area_rom,area_reg,area_fsm_reg,area_fsm_comb > results.csv
