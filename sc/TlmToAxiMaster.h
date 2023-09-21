/*
 * TlmToAxiMaster module

 * Created 2022-11-19 by W. Rhett Davis (rhett_davis@ncsu.edu)
 * Modified from the Master.h file included with NVLabs MatchLib (2020)
 * This module is intended to be instantiated within the TlmToAxi module.
 * It may be possible to fold this functionality into TlmToAxi,
 * but new classes would likely need to be created to allow
 * sending/receiving transactions with the 
 * axi::axi4<>::read::chan<> and axi::axi4<>::write::chan<>
 * classes.
 */

#ifndef __TLMTOAXIMASTER_H__
#define __TLMTOAXIMASTER_H__

#include <systemc.h>
#include <ac_reset_signal_is.h>

#include <axi/axi4.h>
#include <nvhls_connections.h>
#include <hls_globals.h>

#include <queue>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <map>
#include <math.h>
#include <boost/assert.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <algorithm>

template <typename axiCfg,typename cfg>
class TlmToAxiMaster : public sc_module {
  BOOST_STATIC_ASSERT_MSG(axiCfg::useWriteResponses || cfg::numReads == 0 || cfg::readDelay != 0,
                "Must use a substantial read delay if reading without write responses");
 public:
  static const int kDebugLevel = 0;
  typedef axi::axi4<axiCfg> axi4_;

  typename axi4_::read::template master<> if_rd;
  typename axi4_::write::template master<> if_wr;

  sc_in<bool> reset_bar;
  sc_in<bool> clk;

  std::map<typename axi4_::Addr, typename axi4_::Data> localMem;
  std::map<typename axi4_::Addr, NVUINT8 > localMem_wstrb;
  std::vector<typename axi4_::Addr> validReadAddresses;
  std::vector<typename axi4_::Addr> validReadAddresses_q;
  std::vector< long long > validReadAddresses_ctr;

  static const int bytesPerBeat = axi4_::DATA_WIDTH >> 3;
  static const bool wResp = axiCfg::useWriteResponses;

  std::queue <tlm::tlm_generic_payload*> inq;
  tlm_utils::peq_with_get<tlm::tlm_generic_payload> outpeq;

  sc_out<bool> done;

  SC_CTOR(TlmToAxiMaster)
      : if_rd("if_rd"), if_wr("if_wr"), reset_bar("reset_bar"), clk("clk"), 
        outpeq("outpeq") {

    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(reset_bar, false);
  }

  
 protected:
  void run() {
    // static const int bytesPerWord = axi4_::DATA_WIDTH >> 3;
    // static const int axiAddrBitsPerWord = nvhls::log2_ceil<bytesPerWord>::val;
    // Workaround for useBurst=0, which sets ALEN field to 0 width
    static const int ALEN_W = axiCfg::useBurst != 0 ? axi4_::ALEN_WIDTH : 1; 
    static const int WSTRB_W = axiCfg::useWriteStrobes != 0 ? axi4_::WSTRB_WIDTH : 1; 
    std::queue <typename axi4_::Addr> raddr_queue;
    std::queue < NVUINTW(ALEN_W) > rlen_queue;
    std::queue <typename axi4_::Addr> waddr_queue;
    std::queue < NVUINTW(ALEN_W) > wlen_queue;
    typename axi4_::Data check_data;
    NVUINT8 check_data_wstrb;
    NVUINT8 rcv_data_wstrb;
    typename axi4_::AddrPayload addr_pld;
    typename axi4_::ReadPayload data_pld;

    // Follow the same priority as nvhls_rand: environment, then preprocessor define, then config
    unsigned int seed = cfg::seed;
#ifdef RAND_SEED
    seed = (RAND_SEED);
#endif
    const char* env_rand_seed = std::getenv("RAND_SEED");
    if (env_rand_seed != NULL) seed = atoi(env_rand_seed);
    boost::random::mt19937 gen(seed);
    boost::random::uniform_int_distribution<uint64_t> random_addr(cfg::addrBoundLower, cfg::addrBoundUpper);
    boost::random::uniform_int_distribution<> random_wstrb(1, pow(2,WSTRB_W)-1);
    boost::random::uniform_int_distribution<> random_burstlen(0, axiCfg::maxBurstSize-1);
    boost::random::uniform_int_distribution<> uniform_rand;

    typename axi4_::Addr wr_addr = cfg::addrBoundLower;
    typename axi4_::Data wr_data = 0xf00dcafe12345678;
    NVUINTW(WSTRB_W) wstrb = ~0;
    NVUINTW(ALEN_W) wr_len = 0;
    typename axi4_::Addr rd_addr_next;
    typename axi4_::Addr rd_addr;
    NVUINTW(ALEN_W) rd_len;
    typename axi4_::AddrPayload wr_addr_pld;
    typename axi4_::WritePayload wr_data_pld;
    typename axi4_::WRespPayload wr_resp_pld;

    done = 0;
    // unsigned int numWrites = 0;
    // unsigned int numReads = 0;
    unsigned int numWritesOfBurst = 0;
    unsigned int numReadsOfBurst = 0;
    unsigned int numWriteResponses = 0;
    unsigned int numReadResponses = 0;
    bool writeInProgress = false;
    // bool rd_conflict = false;
    // bool wr_conflict = false;

    tlm::tlm_generic_payload *gpp=NULL;
    unsigned char *dp=NULL;
    unsigned long  gplen;
    bool startNewWrite=false;
    bool startNewRead=false;

    if_rd.reset();
    if_wr.reset();

    wait(20);

    while (1) {
      // cout << "Check 0\n";
      wait();
      // cout << "Check 1\n";
      if (!inq.empty()) {
        gpp=inq.front();
        inq.pop();
        dp = gpp->get_data_ptr();
        if (gpp->get_command()==tlm::TLM_WRITE_COMMAND) {
          startNewWrite=true;
          wr_addr=gpp->get_address();
          gplen=gpp->get_data_length();
          wr_len=(gplen % bytesPerBeat)?(gplen/bytesPerBeat):(gplen/bytesPerBeat-1);
          // cout << "length " << dec << gpp->get_data_length() << ' ' << axi4_::DATA_WIDTH << ' ' << wr_len << endl;
          wr_data=reinterpret_cast<long long*>(dp)[0];
          wr_addr_pld.addr = wr_addr;
          wr_data_pld.data = wr_data;
          wr_data_pld.wstrb = wstrb;
          wr_addr_pld.len = wr_len;
        } else if (gpp->get_command()==tlm::TLM_READ_COMMAND) {
          startNewRead=true;
          rd_addr_next=gpp->get_address();
          gplen=gpp->get_data_length();
          rd_len=(gplen % bytesPerBeat)?(gplen/bytesPerBeat):(gplen/bytesPerBeat-1);
          addr_pld.addr = rd_addr_next;
          addr_pld.len = rd_len;
        } else {
          cout << "\nError @" << sc_time_stamp() << " from " << name()
              << ": Command " << gpp->get_command() 
              << ", read_addr=" << hex << rd_addr_next << " not recognized" << endl;
          gpp->set_response_status( tlm::TLM_COMMAND_ERROR_RESPONSE );
          outpeq.notify(*gpp,SC_ZERO_TIME);
        }
      }

      // cout << "Check 2\n";
      // READ

      // if (validReadAddresses.size() > 0 && numReads < cfg::numReads) {
      //   rd_addr_next = validReadAddresses[uniform_rand(gen) % validReadAddresses.size()];
      //   addr_pld.addr = rd_addr_next;
      //   if (axiCfg::useBurst) {
      //     if (uniform_rand(gen) % 2 == 0) { // 50% of reads are bursts (less once valid addresses are disallowed)
      //       rd_len = random_burstlen(gen);
      //     } else {
      //       rd_len = 0;
      //     }
      //     addr_pld.len = rd_len;
      //   }
      //   wr_conflict = false;
      //   for (unsigned int i=0; i<(rd_len+1); i++) {
      //     if (!localMem.count(rd_addr_next+bytesPerBeat*i)) {
      //       wr_conflict = true; // Not actually a conflict, but the read should be cancelled nonetheless
      //     }
      //   }
        // std::ostringstream ms3;
        // ms3 << "\nError @" << sc_time_stamp() << " from " << name()
        //     << ": Testharness attempted to read an address that it never wrote to"
        //     << ", read_addr=" << hex << rd_addr_next
        //     << endl;
        // if (!(localMem.count(rd_addr_next))) cout << ms3.str().c_str() << endl;
        // BOOST_ASSERT_MSG( localMem.count(rd_addr_next), ms3.str().c_str() );
        // for (unsigned int j=0; j<waddr_queue.size(); j++) {
        //   if ((rd_addr_next+bytesPerBeat*rd_len) >= waddr_queue.front() && rd_addr_next <= waddr_queue.front())
        //         wr_conflict = true;
        //   waddr_queue.push(waddr_queue.front());
        //   waddr_queue.pop();
        // }
        if (startNewRead) {
          rd_addr_next = addr_pld.addr;
          if (if_rd.ar.PushNB(addr_pld)) {
            CDCOUT(sc_time_stamp() << " " << name() << " Sent read request: ["
                          << addr_pld << "]"
                          << endl, kDebugLevel);
            // numReads++;
            for (unsigned int i=0; i<(rd_len+1); i++) {
              raddr_queue.push(rd_addr_next);
              rd_addr_next += bytesPerBeat;
            }
            rlen_queue.push(rd_len);
            startNewRead=false;
          }
        }
      // }
      // cout << "Check 3\n";
      if (if_rd.r.PopNB(data_pld)) {
        // cout << "Check 3.1\n";
        rd_addr = raddr_queue.front();
        // cout << "Check 3.2\n";
        // if (axiCfg::useWriteStrobes) {
        //   // cout << "Check 3.3\n";
        //   bool checked_one = false;
        //   for (int i=0; i<axi4_::WSTRB_WIDTH; i++) {
        //     // // cout << "Check 3.3.1\n";
        //     if (localMem_wstrb.count(rd_addr+i)) {
        //       rcv_data_wstrb = nvhls::get_slc<8>(data_pld.data,8*i);
        //       check_data_wstrb = localMem_wstrb[rd_addr+i];
        //       std::ostringstream msg;
        //       msg << "\nError @" << sc_time_stamp() << " from " << name()
        //           << ": Incorrect read data response"
        //           << ", base_addr=" << hex << rd_addr
        //           << ", byte=" << i
        //           << ", data=" << hex << rcv_data_wstrb
        //           << ", expected=" << hex << check_data_wstrb
        //           << std::endl;
        //       if (!(check_data_wstrb == rcv_data_wstrb)) cout << msg.str().c_str() << endl;
        //       BOOST_ASSERT_MSG( check_data_wstrb == rcv_data_wstrb, msg.str().c_str() );
        //       checked_one = true;
        //     }
        //   } 
        //   // cout << "Check 3.4\n";
        //   std::ostringstream msg3;
        //   msg3 << "\nError @" << sc_time_stamp() << " from " << name()
        //       << ": Testbench did not check any bytes of a read response" << std::endl;
        //   if (!checked_one) cout << msg3.str().c_str() << endl;
        //   BOOST_ASSERT_MSG( checked_one, msg3.str().c_str() );
        //   // cout << "Check 3.5\n";
        // } else {
          // cout << "Check 3.6\n";
          // check_data = localMem[rd_addr];
          // std::ostringstream msg;
          // msg << "\nError @" << sc_time_stamp() << " from " << name()
          //     << ": Incorrect read data response"
          //     << ", addr=" << hex << rd_addr
          //     << ", data=" << hex << data_pld.data.to_uint64()
          //     << ", expected=" << hex << check_data.to_uint64()
          //     << std::endl;
          // if (!(check_data == data_pld.data)) cout << msg.str().c_str() << endl;
          // BOOST_ASSERT_MSG( check_data == data_pld.data , msg.str().c_str() );
          // cout << "Check 3.7\n";
        // }
        // cout << "Check 3.8\n";
        std::ostringstream ms2;
        ms2 << "\nError @" << sc_time_stamp() << " from " << name()
            << ": Read response protocol error"
            << ", rresp=" << data_pld.resp.to_uint64()
            << std::endl;
        if (!((data_pld.resp == axi4_::Enc::XRESP::OKAY) |
                          (data_pld.resp == axi4_::Enc::XRESP::EXOKAY)))
                           cout << ms2.str().c_str() << endl;
        BOOST_ASSERT_MSG( (data_pld.resp == axi4_::Enc::XRESP::OKAY) |
                          (data_pld.resp == axi4_::Enc::XRESP::EXOKAY), ms2.str().c_str() );
        // cout << "Check 3.9\n";
        CDCOUT(sc_time_stamp() << " " << name() << " Received correct read response: ["
                      << data_pld << "]"
                      << endl, kDebugLevel);
        // cout << "Check 3.10\n";
        raddr_queue.pop();
        // cout << "Check 3.11\n";
        reinterpret_cast<long long*>(dp)[numReadsOfBurst] = data_pld.data;

        if (numReadsOfBurst++ == rlen_queue.front()) {
          // cout << "Check 3.12\n";
          numReadsOfBurst = 0;
          numReadResponses++;
          rlen_queue.pop();
          // cout << "Check 3.13\n";
          gpp->set_response_status( tlm::TLM_OK_RESPONSE );
          outpeq.notify(*gpp,SC_ZERO_TIME);
        }
      }

      // WRITE
      // cout << "Check 4\n";
      if (startNewWrite && !writeInProgress) {
      // if (!writeInProgress && numWrites < cfg::numWrites) {
        // rd_conflict = false;
        // for (unsigned int j=0; j<raddr_queue.size(); j++) {
        //   if ((wr_addr+bytesPerBeat*wr_len) >= raddr_queue.front() && wr_addr <= raddr_queue.front())
        //         rd_conflict = true;
        //   raddr_queue.push(raddr_queue.front());
        //   raddr_queue.pop();
        // }
        // if (!rd_conflict) {
          if (if_wr.aw.PushNB(wr_addr_pld)) {
            CDCOUT(sc_time_stamp() << " " << name() << " Sent write request: ["
                          << wr_addr_pld << "]"
                          << endl, kDebugLevel);
            for (unsigned int i=0; i<(wr_len+1); i++) {
              waddr_queue.push(wr_addr+bytesPerBeat*i);
              // If the address was already written to once, it needs to be removed from the list of valid addresses
              // because a read after this second write could return incorrect data
              // validReadAddresses.erase(std::remove(validReadAddresses.begin(), validReadAddresses.end(), waddr_queue.front()), validReadAddresses.end());
              // if (!wResp) {
              //   ptrdiff_t pos = find(validReadAddresses_q.begin(), validReadAddresses_q.end(), waddr_queue.front()) - validReadAddresses_q.begin();
              //   int size = validReadAddresses_q.size();
              //   // If the address is already in the queue, just reset the counter
              //   if (pos < size) {
              //     validReadAddresses_ctr[pos] = cfg::readDelay + 20*i;
              //   } else {
              //     validReadAddresses_q.push_back(waddr_queue.front());
              //     validReadAddresses_ctr.push_back(cfg::readDelay + 20*i);
              //   }
              //   waddr_queue.pop();
              // }
            }
            wlen_queue.push(wr_len);
            writeInProgress = true;
            startNewWrite=false;
          }
        //}
      }
      // cout << "Check 5\n";
      if (writeInProgress) {
        if (axiCfg::useBurst) {
          if (numWritesOfBurst == (wr_len)) {
            wr_data_pld.last = 1;
          } else {
            wr_data_pld.last = 0;
          }
        }
        if (if_wr.w.PushNB(wr_data_pld)) {
          CDCOUT(sc_time_stamp() << " " << name() << " Sent write data:"
                        << " addr=" << hex << wr_addr
                        << " data=[" << wr_data_pld << "]"
                        << " beat=" << dec << numWritesOfBurst
                        << endl, kDebugLevel);
          // if (axiCfg::useWriteStrobes) {
          //   for (int i=0; i<axi4_::WSTRB_WIDTH; i++) {
          //     if (wr_data_pld.wstrb[i] == 1) {
          //       localMem_wstrb[wr_addr+i] = nvhls::get_slc<8>(wr_data_pld.data, 8*i);
          //     }
          //   }
          // }
          localMem[wr_addr] = wr_data_pld.data; // Need to keep track of base addresses, even for wstrb case
          if (++numWritesOfBurst == (wr_len+1)) { // Whole burst is done
            // wr_addr = (random_addr(gen) >> axiAddrBitsPerWord) << axiAddrBitsPerWord; // Keep all requests word-aligned
            // if (axiCfg::useBurst) {
            //   if (uniform_rand(gen) % 5 == 0) { // 20% of writes are bursts
            //     wr_len = random_burstlen(gen);
            //     if (wr_addr + bytesPerBeat*wr_len > cfg::addrBoundUpper) wr_len = 0;
            //   } else {
            //     wr_len = 0;
            //   }
            // }
            writeInProgress = false;
            // numWrites++;
            numWritesOfBurst = 0;
            // startNewWrite=false;
            // gpp->set_response_status( tlm::TLM_OK_RESPONSE );
            // outpeq.notify(*gpp,SC_ZERO_TIME);
          } else { // Only this beat is done
            wr_addr += bytesPerBeat;
            wr_data = reinterpret_cast<long long*>(dp)[numWritesOfBurst];
            wr_data_pld.data = wr_data;
          }
          // wr_data = reinterpret_cast<long long*>(dp)[numWritesOfBurst];
          //wr_data.set_slc(axi4_::DATA_WIDTH-8,NVUINT8(uniform_rand(gen))); // Touches the 8 MSBs, in case of wide data words
          // if (axiCfg::useWriteStrobes) {
          //   if (uniform_rand(gen) % 5 == 0) { // 20% of writes have nonuniform strobe
          //     wstrb = random_wstrb(gen);
          //   } else {
          //     wstrb = ~0;
          //   }
          // }
        }
      }
      // cout << "Check 6\n";
      if (if_wr.b.PopNB(wr_resp_pld)) {
        std::ostringstream msg;
        msg << "\nError @" << sc_time_stamp() << " from " << name()
            << ":  Write response protocol error"
            << ", bresp=" << wr_resp_pld.resp.to_uint64()
            << ", addr=" << hex << waddr_queue.front()
            << std::endl;
        BOOST_ASSERT_MSG( (wr_resp_pld.resp == axi4_::Enc::XRESP::OKAY) |
                          (wr_resp_pld.resp == axi4_::Enc::XRESP::EXOKAY), msg.str().c_str() );
        CDCOUT(sc_time_stamp() << " " << name() << " Received write response"
                      << endl, kDebugLevel);
        numWriteResponses++;
        for (unsigned int i=0; i<wlen_queue.front()+1; i++) {
          // validReadAddresses_q.push_back(waddr_queue.front());
          // validReadAddresses_ctr.push_back(cfg::readDelay + 20*i);
          waddr_queue.pop();
        }
        wlen_queue.pop();
        gpp->set_response_status( tlm::TLM_OK_RESPONSE );
        outpeq.notify(*gpp,SC_ZERO_TIME);
      }
      // if (numWrites == cfg::numWrites &&
      //     numReads == cfg::numReads &&
      //     (numWriteResponses == numWrites || !wResp) &&
      //     numReadResponses == numReads)
      //       done = 1;
      //DCOUT(sc_time_stamp() << " from " << name() << " numWrites: " << dec << numWrites << "  numWriteResponses: " << numWriteResponses << "  numReads: " << numReads << "  numReadResponses: " << numReadResponses << endl);
      // std::ostringstream msg;
      // msg << "\nError @" << sc_time_stamp() << " from " << name()
      //     << ": Number of write responses exceeded total number of writes"
      //     << ", numWriteResponses=" << dec << numWriteResponses
      //     << std::endl;
      // BOOST_ASSERT_MSG( numWriteResponses <= cfg::numWrites, msg.str().c_str() );
      // std::ostringstream ms2;
      // ms2 << "\nError @" << sc_time_stamp() << " from " << name()
      //     << ": Number of read responses exceeded total number of reads"
      //     << ", numReadResponses=" << dec << numReadResponses
      //     << std::endl;
      // BOOST_ASSERT_MSG( numReadResponses <= cfg::numReads, msg.str().c_str() );

      // If there are no write responses we still want to test reads.
      // Finesse this by enforcing a fixed delay after a write until reads can be issued to that address.
      // cout << "Check 7\n";
      // for (unsigned int i=0; i<validReadAddresses_q.size(); i++) {
      //   if (validReadAddresses_ctr[i] >= 0) {
      //     if (validReadAddresses_ctr[i] == 0) {
      //       validReadAddresses.push_back(validReadAddresses_q[i]);
      //     }
      //     validReadAddresses_ctr[i]--;
      //   }
      // }
      // cout << "Check 9\n";
      //DCOUT("validReadAddresses: ");
      //for (typename std::vector< sc_uint<axi4_::ADDR_WIDTH> >::const_iterator q = validReadAddresses.begin(); q != validReadAddresses.end(); ++q)
      //      std::cout << *q << ' ';
      //DCOUT(endl);
    }

  }



};

#endif
