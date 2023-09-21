/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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
 */
#ifndef __FIRUNIT_H__
#define __FIRUNIT_H__

#ifdef __SYNTHESIS__
/* These are moved to the pre-compiled header for fast re-compilation,
 * but are still needed for high-level synthesis.  */
#include <systemc.h>
#include <nvhls_module.h>
#include <nvhls_connections.h>
#include <mc_scverify.h>
#endif

#include <ac_reset_signal_is.h>

#include <axi/axi4.h>
#include "AxiSlaveToReg2.h"
#include <CombinationalBufferedPorts.h>

#define TAPS 16
// Pack 4 shorts into 64 bit value
// SystemC concatenation not compatible with AC_INT, so manually masking & bit shifting instead
#define SHORTS_TO_LONG(val1, val2, val3, val4) ((((unsigned long long)val4 & 0xFFFF) << 48) | (((unsigned long long)val3 & 0xFFFF) << 32) | (((unsigned long long)val2 & 0xFFFF) << 16) | (((unsigned long long)val1 & 0xFFFF)))

class firUnit : public sc_module {
 public:
  static const int kDebugLevel = 4;

  typedef axi::axi4<axi::cfg::standard> axi_;
  enum { numReg = 14, baseAddress = 0x0, numAddrBitsToInspect = 16 };

  sc_in<bool> clk;
  sc_in<bool> reset_bar;

  typename axi_::read::template slave<> axi_read;
  typename axi_::write::template slave<> axi_write;

  AxiSlaveToReg2<axi::cfg::standard, numReg, numAddrBitsToInspect> slave;
  typedef AxiSlaveToReg2<axi::cfg::standard, numReg, numAddrBitsToInspect>::reg_write reg_write_;

  sc_signal<NVUINTW(numAddrBitsToInspect)> baseAddr;
  sc_signal<NVUINTW(axi_::DATA_WIDTH)> regOut_chan[numReg];

  Connections::CombinationalBufferedPorts<reg_write_,0,1> regIn_chan;

  //array to store weights in continuous block
  //HLS will optimize this out ideally
  sc_int<16> weights[16];

  //array to hold copies of last 16 inputs and new 16 inputs
  //HLS should pipeline this and not cache everything
  sc_int<16> inputBuffer[32];

  //Array for storing output of FIR calculation
  //sc_int not compatible with bit masking & shifting, so used short
  short outputArray[32];

  SC_HAS_PROCESS(firUnit);

  firUnit(sc_module_name name)
      : sc_module(name),
        clk("clk"),
        reset_bar("reset_bar"),
        axi_read("axi_read"),
        axi_write("axi_write"),
        slave("slave"),
        regIn_chan("regIn_chan")
  {
    slave.clk(clk);
    slave.reset_bar(reset_bar);

    slave.if_axi_rd(axi_read);
    slave.if_axi_wr(axi_write);
    slave.regIn(regIn_chan);

    slave.baseAddr(baseAddr);
    baseAddr.write(baseAddress);

    for (int i = 0; i < numReg; i++) {
      slave.regOut[i](regOut_chan[i]);
    }

    //initialize all buffers to zero
    for (int i = 0; i < 16; i++) {
      weights[i] = 0;
    } 

    for (int i = 0; i < 32; i++) {
      inputBuffer[i] = 0;
    } 

    SC_THREAD (run); 
    sensitive << clk.pos(); 
    NVHLS_NEG_RESET_SIGNAL_IS(reset_bar);
  }

  void run()
  {
    
    regIn_chan.ResetWrite();
    reg_write_ regwr;
    //clear FIR status register
    regwr.addr = 10*8;
    regwr.data = 0;
    NVUINTW(axi_::DATA_WIDTH) lastCtrl = 0;
    

    while (1)
    {
        //flush regOut FIFO
        //this should not be neccesary, but the simulation required a dummy transaction to work
        wait();
        //dummy transaction
        regIn_chan.TransferNBWrite();
        wait();

        if(regOut_chan[1].read() != lastCtrl) {
          lastCtrl = regOut_chan[1].read();

          //watch for control register change to 2 (FIR start code)
          if(regOut_chan[1].read() == 0x02) {
            //read weights in
            for(int i = 0; i < 4; i+= 1) {
                weights[i + 0] = regOut_chan[2].read().slc<16>(i*16);
                weights[i + 4] = regOut_chan[3].read().slc<16>(i*16);
                weights[i + 8] = regOut_chan[4].read().slc<16>(i*16);
                weights[i + 12] = regOut_chan[5].read().slc<16>(i*16);
            }
            //shift old inputs
            for(int i = 0; i < 16; i+=1) {
              inputBuffer[i] = inputBuffer[i + 16];
            }
            wait();

            //process new inputs
            for(int i = 0; i < 4; i+= 1) {
              inputBuffer[i + 16] = regOut_chan[6].read().slc<16>(i*16);
              inputBuffer[i + 20] = regOut_chan[7].read().slc<16>(i*16);
              inputBuffer[i + 24] = regOut_chan[8].read().slc<16>(i*16);
              inputBuffer[i + 28] = regOut_chan[9].read().slc<16>(i*16);
            }

            //FIR computation
            for (int n=0; n<32; n++) {
              outputArray[n]=0;
              for (int m=0; m<TAPS; m++) {
                if (n+m-TAPS+1 >= 0) {
                  outputArray[n]+=weights[m]*inputBuffer[n+m-TAPS+1];
                }
              }
            }

            // Write FIR results to regOut's
            // A for loop would make sense here to adhere to DRY
            // simulator had a tough time with loop, so it is manually unrolled
            regwr.data = SHORTS_TO_LONG(outputArray[16], outputArray[17], outputArray[18], outputArray[19]);
            regwr.addr = 80;
            regIn_chan.Push(regwr);
            regIn_chan.TransferNBWrite();
            wait();
            wait();
            wait();

            regwr.data = SHORTS_TO_LONG(outputArray[20], outputArray[21], outputArray[22], outputArray[23]);
            regwr.addr = 88;
            regIn_chan.Push(regwr);
            regIn_chan.TransferNBWrite();
            wait();
            wait();
            wait();


            regwr.addr = 96;
            regwr.data = SHORTS_TO_LONG(outputArray[24], outputArray[25], outputArray[26], outputArray[27]);
            regIn_chan.Push(regwr);
            regIn_chan.TransferNBWrite();
            wait();
            wait();
            wait();

            regwr.addr = 104;
            regwr.data = SHORTS_TO_LONG(outputArray[28], outputArray[29], outputArray[30], outputArray[31]);
            regIn_chan.Push(regwr);
            regIn_chan.TransferNBWrite();
            wait();
            wait();
            wait();

            regwr.data = 3;
            regwr.addr = 0;
            regIn_chan.Push(regwr);
            regIn_chan.TransferNBWrite();
            wait();
          }
        }
    }
    }
};

#endif
