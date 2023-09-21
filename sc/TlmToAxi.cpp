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

#include "nvhls_pch.h"
#include "TlmToAxi.h"
#include <string>
#include <iostream>
#include <iomanip>

#include <ac_reset_signal_is.h>

using namespace  std;


SC_HAS_PROCESS(TlmToAxi);
TlmToAxi::TlmToAxi( sc_core::sc_module_name module_name)
  : sc_module (module_name),
    master("master"),
    dut("dut"),
    clk("clk", 1.0, SC_NS, 0.5, 0, SC_NS, true),
    reset_bar("reset_bar"),
    axi_read("axi_read"),
    axi_write("axi_write")
{
  slave.register_b_transport(this, &TlmToAxi::custom_b_transport);

  Connections::set_sim_clk(&clk);

  dut.clk(clk);
  master.clk(clk);

  dut.reset_bar(reset_bar);
  master.reset_bar(reset_bar);

  master.if_rd(axi_read);
  master.if_wr(axi_write);

  dut.axi_read(axi_read);
  dut.axi_write(axi_write);

  // for (int i = 0; i < numReg; i++) {
  //   dut.regOut[i](regOut[i]);
  // }

  master.done(done);

  SC_THREAD(run);
}

void TlmToAxi::run()
{
    reset_bar = 1;
    wait(2, SC_NS);
    reset_bar = 0;
    wait(2, SC_NS);
    reset_bar = 1;

    while (1) {
      wait();
    }
}

void                                        
TlmToAxi::custom_b_transport
 ( tlm::tlm_generic_payload &gp, sc_core::sc_time &delay )
{
  sc_dt::uint64    address   = gp.get_address();
  tlm::tlm_command command   = gp.get_command();
  unsigned long    length    = gp.get_data_length();
  sc_core::sc_time mem_delay(10,sc_core::SC_NS);

  tlm::tlm_generic_payload *gpp;

  cout << sc_core::sc_time_stamp() << " " << sc_object::name();
  switch (command) {
    case tlm::TLM_WRITE_COMMAND:
    {
      cout << " WRITE len:0x" << hex << length << " addr:0x" << address << endl;
      break;
    }
    case tlm::TLM_READ_COMMAND:
    {
      cout << " READ len:0x" << hex << length << " addr:0x" << address << endl; 
      break;
    }
    default:
    {
      cout << " ERROR Command " << command << " not recognized" << endl;
    } 
  }

  m_mutex.lock();
  master.inq.push(&gp);
  wait(master.outpeq.get_event());
  gpp=master.outpeq.get_next_transaction();
  if (gpp!=&gp) {
    cout << sc_core::sc_time_stamp() << " " << sc_object::name() 
          << " ERROR: incomming payload pointer does not match outgoing payload pointer" << endl;
  }
  m_mutex.unlock();

  cout << sc_core::sc_time_stamp() << " " << sc_object::name() << " transaction complete" << endl;

  if (gp.get_address()==0x08 && command==tlm::TLM_WRITE_COMMAND) {
    if ((long long)dut.regOut_chan[1].read()==(long long)0x01) {
      for (int i = 0; i < firUnit::numReg; i++) {
        cout << sc_core::sc_time_stamp() << ' ' << name() << " regOut[" << dec << i << "] = " << hex << dut.regOut_chan[i] << endl;
      }
    }
    else if ((long long)dut.regOut_chan[1].read()==(long long)0x0f) {
      cout << sc_core::sc_time_stamp() << ' ' << name() << " received exit signal" << endl;
      sc_stop();
    }
  }

  return;     
}







