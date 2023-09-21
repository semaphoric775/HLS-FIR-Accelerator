/*************************************************

SystemC Transaction Level Modeling Tutorial
(c) 11/25/2019 W. Rhett Davis (rhett_davis@ncsu.edu)

**************************************************/

#ifndef __DMA_H__
#define __DMA_H__

#include <tlm.h>
#include "tlm_utils/simple_target_socket.h"


class dma
  : public sc_core::sc_module                       
  , virtual public tlm::tlm_bw_transport_if<>
{
  public:
  static const unsigned int buswidth=64;

  SC_HAS_PROCESS(dma);  
  dma(sc_core::sc_module_name name);

  ~dma();

  tlm::tlm_initiator_socket<buswidth> master;
  tlm_utils::simple_target_socket<dma,buswidth>  slave;

  class registers {
    public:
    long long st;
    long long ctrl;
    long long sr;
    long long dr;
    long long len;
  };
  registers *regs;
  unsigned char *data;
  sc_dt::uint64  m_memory_size;


  void transfer ( );

  private:
  sc_dt::uint64 m_coef_ptr;
  sc_core::sc_mutex m_mutex;

  void custom_b_transport
  ( tlm::tlm_generic_payload &gp, sc_core::sc_time &delay );

/// Not Implemented for this example but required by the initiator socket
  void invalidate_direct_mem_ptr
    (sc_dt::uint64 start_range, sc_dt::uint64 end_range);
  tlm::tlm_sync_enum nb_transport_bw (tlm::tlm_generic_payload  &gp, 
     tlm::tlm_phase &phase, sc_core::sc_time &delay);

};


#endif /* __DMA_H__ */
