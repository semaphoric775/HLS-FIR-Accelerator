/*
 * TlmToAxiMaster module
 *
 * Created 2022-11-19 by W. Rhett Davis (rhett_davis@ncsu.edu)
 *
 * This module translates standard SystemC TLM transactions
 * into MatchLib Axi transactions.  This is accomplished by
 * creating instances similar to the MatchLib Master and
 * AxiSlaveToReg template classes.  The b_transport handler
 * for the TLM slave socket puts transactions into the
 * Master's queue and waits for it to drive the AXI
 * channels the connect to the device under test.
 */


#ifndef __TLMTOAXI_H__ 
#define __TLMTOAXI_H__

#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include <axi/axi4.h>
#include "TlmToAxiMaster.h"
#include "firUnit.h"

class TlmToAxi: public sc_core::sc_module
{
  public:  

  static const unsigned int buswidth=64;
  sc_dt::uint64  m_memory_size;
  sc_core::sc_mutex m_mutex;

  TlmToAxi( sc_core::sc_module_name module_name);

  tlm_utils::simple_target_socket<TlmToAxi,buswidth>  slave;
 
  typedef firUnit::axi_ axi_;
  enum {
    numReg = firUnit::numReg,
    numAddrBitsToInspect = firUnit::numAddrBitsToInspect
  };

  struct Mcfg {
    enum {
      numWrites = 1,
      numReads = 2,
      readDelay = 0,
      addrBoundLower = 0x000,
      addrBoundUpper = 0x06F,
      seed = 0,
      useFile = false,
    };
  };

  TlmToAxiMaster<axi::cfg::standard, Mcfg> master;

  CCS_DESIGN(firUnit) dut;

  sc_clock clk;
  sc_signal<bool> reset_bar;
  sc_signal<bool> done;

  typename axi_::read::template chan<> axi_read;
  typename axi_::write::template chan<> axi_write;

  // sc_signal<NVUINTW(axi::axi4<axi::cfg::standard>::DATA_WIDTH)> regOut[numReg];

  private:


  void run();	    

  void custom_b_transport
  ( tlm::tlm_generic_payload &gp, sc_core::sc_time &delay );

};


#endif /* __LSTMTOP_H__ */
