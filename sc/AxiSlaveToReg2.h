/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License")
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Modified 2022-11-19 by W. Rhett Davis (rhett_davis@ncsu.edu)
 * from the NVLabs MatchLib AxiSlaveToReg template module.
 * This file is nearly identical, but modifies the assertions
 * to print meaningful error messages, rather than simply halting
 * the simulation.
 */

#ifndef __AXISLAVETOREG2_H__
#define __AXISLAVETOREG2_H__

#include <systemc.h>
#include <ac_int.h>
#include <hls_globals.h>
#include <axi/axi4.h>
#include <mem_array.h>
#include <fifo.h>
#include "Arbiter.h"

/**
 * \brief An AXI slave containing memory-mapped registers.
 * \ingroup AXI
 *
 * \tparam axiCfg                   A valid AXI config.
 * \tparam numReg                   The number of registers in the slave.  Each register has a width equivalent to the AXI data width.
 * \tparam numAddrBitsToInspect     The number of address bits to inspect when determining which slave to direct traffic to.  If this is less than the full address width, the routing determination will be made based on the number of address LSBs specified.  (Default: axiCfg::addrWidth)
 *
 * \par Overview
 * AxiSlaveToReg is an AXI slave that saves its state in a bank of registers.  The register state is accessible as an array of sc_out.
 *
 * \par Usage Guidelines
 *
 * This module sets the stall mode to flush by default to mitigate possible RTL
 * bugs that can occur in the default stall mode. If you are confident that
 * this class of bugs will not occur in your use case, you can change the stall
 * mode via TCL directive:
 *
 * \code
 * directive set /path/to/AxiSlaveToReg/run/while -PIPELINE_STALL_MODE stall
 * \endcode
 *
 * This may reduce area/power.
 * \par
 *
 */
template <typename axiCfg, int numReg, int numAddrBitsToInspect = axiCfg::addrWidth>
class AxiSlaveToReg2 : public sc_module {
 public:
  static const int kDebugLevel = 0;
  
  typedef typename axi::axi4<axiCfg> axi4_;
  
  sc_in<bool> clk;
  sc_in<bool> reset_bar;

  typename axi4_::read::template slave<> if_axi_rd;
  typename axi4_::write::template slave<> if_axi_wr;

  static const int regAddrWidth = nvhls::log2_ceil<numReg>::val;
  static const int bytesPerReg = axi4_::DATA_WIDTH >> 3;
  static const int axiAddrBitsPerReg = nvhls::log2_ceil<bytesPerReg>::val;

  sc_in<NVUINTW(numAddrBitsToInspect)> baseAddr;

  // Each reg is one AXI data word
  sc_out<NVUINTW(axi4_::DATA_WIDTH)> regOut[numReg];

  class reg_write {
	public:
    NVUINTW(3+nvhls::log2_ceil<numReg>::val) addr;
	  NVUINTW(axi4_::DATA_WIDTH) data;
    static const int width = axi4_::DATA_WIDTH + 7;
    template <unsigned int Size>
    void Marshall(Marshaller<Size>& m) {
      m & addr;
      m & data;
	  }

#ifdef CONNECTIONS_SIM_ONLY
  inline friend void sc_trace(sc_trace_file *tf, const reg_write& v, const std::string& NAME ) {
    std::ostringstream os1,os2;
    os1 << NAME << ".addr";
    sc_trace(tf,v.addr,os1.str());
    os2 << NAME << ".data";
    sc_trace(tf,v.data,os2.str());
  }
#endif
  inline friend std::ostream& operator<<(ostream& os, const reg_write& rhs) {
    os << "reg_write(addr:" << rhs.addr << ",data:" << rhs.data << ")";
    return os;
	  }
	};

  Connections::In<reg_write> regIn;

 public:
  SC_CTOR(AxiSlaveToReg2)
      : clk("clk"),
        reset_bar("reset_bar"),
        if_axi_rd("if_axi_rd"),
        if_axi_wr("if_axi_wr"),
        regIn("regIn")
  {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(reset_bar, false);
  }

 protected:
  void run() {
    if_axi_rd.reset();
    if_axi_wr.reset();
    regIn.Reset();
    
    NVUINTW(axi4_::DATA_WIDTH) reg[numReg];
    NVUINTW(numAddrBitsToInspect) maxValidAddr = baseAddr.read() + bytesPerReg*numReg - 1;

#pragma hls_unroll yes
    for (int i=0; i<numReg; i++) {
      reg[i] = 0;
      regOut[i].write(reg[i]);
    }

    typename axi4_::AddrPayload axi_rd_req;
    typename axi4_::ReadPayload axi_rd_resp;
    typename axi4_::AddrPayload axi_wr_req_addr;
    typename axi4_::WritePayload axi_wr_req_data;
    typename axi4_::WRespPayload axi_wr_resp;

    NVUINTW(numAddrBitsToInspect) axiRdAddr;
    NVUINTW(axi4_::ALEN_WIDTH) axiRdLen=0;
    bool valid_rd_addr;
    NVUINTW(numAddrBitsToInspect) axiWrAddr;
    bool valid_wr_addr;

    bool read_arb_req = 0;
    bool write_arb_req = 0;
    bool regIn_arb_req = 0;
    bool arb_needs_update = 1;
    NVUINTW(3) valid_mask = 0;
    NVUINTW(3) select_mask = 0;
    Arbiter<3> arb;

    reg_write regwr;
    regwr.addr = 0; regwr.data=0;

    #pragma hls_pipeline_init_interval 1
    #pragma pipeline_stall_mode flush
    while (1) {
      wait();
      // cout << "S2R Check 1\n";

      valid_mask = regIn_arb_req << 2 | write_arb_req << 1 | read_arb_req;
      if (arb_needs_update) {
        select_mask = arb.pick(valid_mask);
        if (select_mask != 0) arb_needs_update = 0;
      }

      // cout << "S2R Check 2\n";
      if (!read_arb_req) {
        if (if_axi_rd.nb_aread(axi_rd_req)) { 
          read_arb_req = 1;
          NVUINTW(numAddrBitsToInspect) addr_temp(static_cast<sc_uint<numAddrBitsToInspect> >(axi_rd_req.addr));
          NVUINTW(axi4_::ALEN_WIDTH) len_temp(static_cast< sc_uint<axi4_::ALEN_WIDTH> >(axi_rd_req.len));
          axiRdAddr = addr_temp;
          axiRdLen = len_temp;
        }
      }

      // cout << "S2R Check 3\n";
      if (!write_arb_req) { 
        if (if_axi_wr.aw.PopNB(axi_wr_req_addr)) {
          write_arb_req = 1;
          NVUINTW(numAddrBitsToInspect) addr_temp(static_cast< sc_uint<numAddrBitsToInspect> >(axi_wr_req_addr.addr));
          axiWrAddr = addr_temp;
        }
      }
      if (!regIn_arb_req) { 
        if (regIn.PopNB(regwr)) {
          regIn_arb_req = 1;
        }
      }
      // cout << "S2R Check 4\n";
      if (select_mask == 1) {
        valid_rd_addr = (axiRdAddr >= baseAddr.read() && axiRdAddr <= maxValidAddr);
        if (!valid_rd_addr) cout << "Read address " << axiRdAddr << " is out of bounds: [" << baseAddr.read() << "," << maxValidAddr << "]" << endl;
        CMOD_ASSERT_MSG(valid_rd_addr, "Read address is out of bounds");
        NVUINTW(regAddrWidth) regAddr = (axiRdAddr - baseAddr.read()) >> axiAddrBitsPerReg;
        axi_rd_resp.id = axi_rd_req.id;
        if (valid_rd_addr) {
          axi_rd_resp.resp = axi4_::Enc::XRESP::OKAY;
          NVUINTW(axi4_::DATA_WIDTH) read_data;
          axi_rd_resp.data = reg[regAddr];
        } 
        else {
          axi_rd_resp.resp = axi4_::Enc::XRESP::SLVERR;
        }
        if (axiRdLen == 0) {
          axi_rd_resp.last = 1;
          read_arb_req = 0;
          arb_needs_update = 1;
        } else {
          axi_rd_resp.last = 0;
          axiRdLen--;
          axiRdAddr += bytesPerReg;
        }
        if_axi_rd.rwrite(axi_rd_resp);
        CDCOUT(sc_time_stamp() << " " << name() << " Read from local reg:"
                      << " axi_addr=" << hex << axiRdAddr.to_int64()
                      << " reg_addr=" << regAddr.to_int64()
                      << " data=" << hex << axi_rd_resp.data
                      << endl, kDebugLevel);
      } else if (select_mask == 2) {
        // cout << "S2R Check 4.1\n";
        if (if_axi_wr.w.PopNB(axi_wr_req_data)) {
          // cout << "S2R Check 4.2\n";
          valid_wr_addr = (axiWrAddr >= baseAddr.read() && axiWrAddr <= maxValidAddr);
          // cout << "S2R Check 4.2.0\n";
          if (!valid_wr_addr) cout << "Write address " << axiWrAddr << " is out of bounds: [" << baseAddr.read() << "," << maxValidAddr << "]" << endl;
          CMOD_ASSERT_MSG(valid_wr_addr, "Write address is out of bounds");
          // cout << "S2R Check 4.2.1\n";
          NVUINTW(axi4_::DATA_WIDTH) axiData(static_cast<typename axi4_::Data>(axi_wr_req_data.data));
          // cout << "S2R Check 4.2.2\n";
          NVUINTW(regAddrWidth) regAddr = (axiWrAddr - baseAddr.read()) >> axiAddrBitsPerReg;
          // cout << "S2R Check 4.2.3\n";
          if (!axi_wr_req_data.wstrb.and_reduce()) { // Non-uniform write strobe - need to do read-modify-write
            // cout << "S2R Check 4.2.4\n";
            NVUINTW(axi4_::DATA_WIDTH) old_data = reg[regAddr];
#pragma hls_unroll yes
            // cout << "S2R Check 4.2.5\n";
            for (int i=0; i<axi4_::WSTRB_WIDTH; i++) {
              if (axi_wr_req_data.wstrb[i] == 0) {
                axiData = nvhls::set_slc(axiData, nvhls::get_slc<8>(old_data,8*i), 8*i);
              }
            }
          }
#pragma hls_unroll yes
          // cout << "S2R Check 4.3\n";
          for (int i=0; i<numReg; i++) { // More verbose, but this is the preferred coding style for HLS
            if (i == regAddr) {
              reg[i] = axiData;
            }
          }
          // cout << "S2R Check 4.4\n";
          CDCOUT(sc_time_stamp() << " " << name() << " Wrote to local reg:"
                        << " axi_addr=" << hex << axiWrAddr.to_int64()
                        << " reg_addr=" << regAddr.to_int64()
                        << " data=" << hex << axi_wr_req_data.data
                        << " wstrb=" << hex << axi_wr_req_data.wstrb.to_uint64()
                        << endl, kDebugLevel);
          if (axi_wr_req_data.last == 1) {
            write_arb_req = 0;
            arb_needs_update = 1;
            if (axiCfg::useWriteResponses) {
              axi_wr_resp.id = axi_wr_req_addr.id;
              if (valid_wr_addr) {
                axi_wr_resp.resp = axi4_::Enc::XRESP::OKAY;
              } else {
                axi_wr_resp.resp = axi4_::Enc::XRESP::SLVERR;
              }
              if_axi_wr.bwrite(axi_wr_resp);
            }
          } else {
            axiWrAddr += bytesPerReg;
          }
        }
      } else if (select_mask == 4) {
        reg[regwr.addr>>3]=regwr.data;
        CDCOUT(sc_time_stamp() << " " << name() << " Wrote to local reg from regIn:"
                        << " addr=" << hex << regwr.addr
                        << " data=" << hex << regwr.data
                        << endl, kDebugLevel);
        regIn_arb_req = 0;
        arb_needs_update = 1;
        // cout << sc_time_stamp() << " S2R wrote " << regwr << endl;
      }
#pragma hls_unroll yes
      // cout << "S2R Check 5\n";
      for (int i=0; i<numReg; i++) {
        regOut[i].write(reg[i]);
      }
    }
  }
};

#endif
