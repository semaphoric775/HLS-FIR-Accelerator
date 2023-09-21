/*************************************************

SystemC Transaction Level Modeling Tutorial
(c) 11/25/2019 W. Rhett Davis (rhett_davis@ncsu.edu)

**************************************************/

#include "nvhls_pch.h"
#include "dma.h"
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>

#define NINP 16


using namespace std;

dma::dma (sc_core::sc_module_name name)
  : sc_module(name)
 { 
    master(*this);
    slave.register_b_transport(this, &dma::custom_b_transport);
    m_memory_size=sizeof(registers);
    data=new unsigned char[m_memory_size];
    regs=reinterpret_cast<registers*>(data);
    regs->st=0;  // No transfer in process 
}

dma::~dma()
{
  delete data;
}


void 
dma::transfer()
{
  sc_core::sc_time delay=sc_core::SC_ZERO_TIME; // Transaction delay
  tlm::tlm_generic_payload  gp;                 // Payload
  //sc_dt::uint64 addr;                           // Transaction address

  static const unsigned int bufsize=0x2000;
  unsigned char buf[bufsize];

  m_mutex.lock();
  regs->st=1;  // Transfer in process
  m_mutex.unlock();
 
  gp.set_command(tlm::TLM_READ_COMMAND);
  gp.set_address( regs->sr );
  gp.set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );
  gp.set_data_length(regs->len);
  gp.set_data_ptr(buf);

  cout << sc_core::sc_time_stamp() << " " << sc_object::name()
       << " transfer READ addr:0x" << hex << regs->sr << endl;

  master->b_transport(gp, delay);
  cout << sc_core::sc_time_stamp() << " " << sc_object::name()
       << " transfer READ Complete" << endl;

  gp.set_command(tlm::TLM_WRITE_COMMAND);
  gp.set_address( regs->dr );
  gp.set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );
  gp.set_data_length(regs->len);
  gp.set_data_ptr(buf);

  m_mutex.lock();
  regs->st=0;  // Transfer complete
  m_mutex.unlock();

  cout << sc_core::sc_time_stamp() << " " << sc_object::name()
       << " transfer WRITE addr:0x" << hex << regs->dr << endl;

  master->b_transport(gp, delay);
  cout << sc_core::sc_time_stamp() << " " << sc_object::name()
       << " transfer WRITE Complete" << endl;


  return;
}

void
dma::custom_b_transport
 ( tlm::tlm_generic_payload &gp, sc_core::sc_time &delay )
{
  sc_dt::uint64    address   = gp.get_address();
  tlm::tlm_command command   = gp.get_command();
  unsigned long    length    = gp.get_data_length();
  unsigned long    i;
  unsigned char    *dp       = gp.get_data_ptr();
  sc_core::sc_time mem_delay(1,sc_core::SC_NS);

  wait(delay);
  m_mutex.lock();
  wait(mem_delay);
  m_mutex.unlock();
  cout << sc_core::sc_time_stamp() << " " << sc_object::name();
  if (address < m_memory_size) {
    switch (command) {
      case tlm::TLM_WRITE_COMMAND:
      {
        cout << " WRITE len:0x" << hex << length << " addr:0x" << address; 
        if (dp) {
          cout << " data:0x";
          m_mutex.lock();
          for (i=length;i>0;i--) {
            cout << hex << setfill('0') << setw(2) << (unsigned int)dp[i-1];
            data[address+i-1]=dp[i-1];
	  }
          m_mutex.unlock();
          cout << endl;
	}
        else
          cout << endl;

        if (address==0x00000020)
          transfer();
        gp.set_response_status( tlm::TLM_OK_RESPONSE );
        break;
      }
      case tlm::TLM_READ_COMMAND:
      {
        cout << " READ len:0x" << hex << length << " addr:0x" << address;
        if (dp) {
          cout << " data:0x";
          for (i=length;i>0;i--) {
            cout << hex << setfill('0') << setw(2) << (unsigned int)data[address+i-1];
            dp[i-1]=data[address+i-1];
          }
          cout << endl;
        }
        else
          cout << endl;


        gp.set_response_status( tlm::TLM_OK_RESPONSE );
        break;
      }
      default:
      {
        cout << sc_core::sc_time_stamp() << " " << sc_object::name() 
             << " ERROR Command " << command << " not recognized" << endl;
        gp.set_response_status( tlm::TLM_COMMAND_ERROR_RESPONSE );
      }
    }
  }
  else {
    cout << " ERROR Address 0x" << hex << address << " out of range" << endl;
    gp.set_response_status( tlm::TLM_ADDRESS_ERROR_RESPONSE );
  } 

  /*
  cout << sc_core::sc_time_stamp() << " " << sc_object::name();
  cout << hex 
       << " st "   << regs->st    << " " << ((long long*)data)[0]
       << " ctrl " << regs->ctrl  << " " << ((long long*)data)[1]
       << " sr "   << regs->sr    << " " << ((long long*)data)[2]
       << " dr "   << regs->dr    << " " << ((long long*)data)[3]
       << " len "  << regs->len   << " " << ((long long*)data)[4];
  cout << " ";
  for (i=0; i<sizeof(registers); i++)
    cout << hex << setfill('0') << setw(2) << (unsigned int)(data[i]);
  cout << endl;
  */

  return;
}


tlm::tlm_sync_enum  dma::nb_transport_bw( tlm::tlm_generic_payload &gp,
                           tlm::tlm_phase &phase, sc_core::sc_time &delay)
{
  tlm::tlm_sync_enum status;
  status = tlm::TLM_ACCEPTED;
  return status;
} // end nb_transport_bw


void dma::invalidate_direct_mem_ptr					
  (sc_dt::uint64 start_range, sc_dt::uint64 end_range)
{  
    return;
} // end invalidate_direct_mem_ptr
