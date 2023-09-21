/*************************************************

SystemC Transaction Level Modeling Tutorial
(c) 2/3/2013 W. Rhett Davis (rhett_davis@ncsu.edu)

**************************************************/

//#include "top.h"
#include "nvhls_pch.h"
//#include <tlm.h>
#include <stdlib.h>
#include "spike.h"
#include "memctl.h"
#include "SimpleBusLT.h"
#include "SimpleBusLT16.h"
#include "dma.h"
#include "TlmToAxi.h"

int sc_main (int argc,char  *argv[])
{
  time_t begin_time, end_time;
  time(&begin_time);
  spike cpu("cpu",argc,argv,false);
  memctl mem("mem",0x10000,false);
  TlmToAxi tlm2axi("tlm2axi");
  SimpleBusLT<2,2> bus0("bus0");
  SimpleBusLT16<1,2> bus1("bus1");
  dma dma0("dma0");
  cpu.master(bus0.target_socket[0]);
  dma0.master(bus0.target_socket[1]);
  bus0.initiator_socket[0](mem.slave);
  bus0.initiator_socket[1](bus1.target_socket[0]);
  bus1.initiator_socket[0](dma0.slave);
  bus1.initiator_socket[1](tlm2axi.slave);
  sc_core::sc_start();
  time(&end_time);
  std::cout << "Simulation time: " << sc_core::sc_time_stamp() << std::endl
            << "Wall clock time: " << difftime(end_time,begin_time) 
            << " seconds\n";
  return 0;
}
