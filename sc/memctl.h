/*************************************************

SystemC DDR SDRAM Controller Model
(c) 10/28/2020 W. Rhett Davis (rhett_davis@ncsu.edu)

**************************************************/


#ifndef __MEMCTL_H__ 
#define __MEMCTL_H__

#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"

class memctl: public sc_core::sc_module
{
  public:  

  bool m_verbose;

  memctl( sc_core::sc_module_name module_name,
       sc_dt::uint64  memory_size,  // memory size (bytes)
       bool verbose = true
      );

  ~memctl();

  tlm_utils::simple_target_socket<memctl,64>  slave;
 
  private:
	    
  bool m_initialized[4];
  sc_dt::uint64 m_memory_size,m_last_addr[4];
  unsigned char *data;

  void custom_b_transport
  ( tlm::tlm_generic_payload &gp, sc_core::sc_time &delay );

};


#endif /* __MEMCTL_H__ */
