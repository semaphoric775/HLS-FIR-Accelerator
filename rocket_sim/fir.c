
#include <stdio.h>

#define TAPS 16
#define TSTEP 48

// short coef[TAPS] = {
// #include "coef.inc"
// };

// short input[TSTEP] = {
// #include "input.inc"
// };


#include "expected.inc"

int main( int argc, char* argv[] )
{
  volatile long long *llp;
  volatile long long **llpp;

  llpp=(volatile long long**)0x70000010; // dma sr
  *llpp=(volatile long long*)0x00004000; // memctl coef1 address
  llpp=(volatile long long**)0x70000018; // dma dr
  *llpp=(volatile long long*)0x10010010; // fir tap coef
  llp=(volatile long long*)0x70000020;   // dma len
  *llp=(volatile long long)32; // starts transfer
  llp=(volatile long long*)0x70000000;   // dma st

  // The following line polls the dma st register until it is 0,
  // effectively waiting until the transfer is complete.
  while (*llp); 

  // If the previous line is omitted, then the next one will not
  // show the expected valye, because the transfer is not complete.
  printf("cpu main {W[3],W[2],W[1],W[0]} 0x%lx (0x2ffffffff0000 expected)\n",*((long long*)0x70010010));

  printf("Copying first batch of inputs to FIR unit\n");
  llpp=(volatile long long**)0x70000010; // dma sr
  *llpp=(volatile long long*)(0x00002000); // memctl coef1 address
  llpp=(volatile long long**)0x70000018; // dma dr
  *llpp=(volatile long long*)0x10010030; // fir input
  llp=(volatile long long*)0x70000020;   // dma len
  *llp=(volatile long long)32; // starts transfer
  llp=(volatile long long*)0x70000000;   // dma st
  while (*llp);

  llp=(volatile long long*)0x70010008; // fir ctrl
  *llp=(volatile long long)0x02;       // Start computation cycle 1
  llp=(volatile long long*)0x70010000;  //reset status register
  *llp = 0x07;
  //poll FIR unit status register
  while(*llp != 0x03) {printf("Core waiting for FIR unit\n");};

  printf("Copying first batch of FIR outputs to memory\n");
  llpp=(volatile long long**)0x70000010; // dma sr
  *llpp=(volatile long long*)(0x10010050);
  llpp=(volatile long long**)0x70000018; // dma dr
  *llpp=(volatile long long*)0x00001000;
  llp=(volatile long long*)0x70000020;   // dma len
  *llp=(volatile long long)32; // starts transfer
  llp=(volatile long long*)0x70000000;   // dma st
  while (*llp);

  printf("Copying second batch of inputs to FIR unit\n");
  llpp=(volatile long long**)0x70000010; // dma sr
  *llpp=(volatile long long*)(8192+32); // memctl coef1 address
  llpp=(volatile long long**)0x70000018; // dma dr
  *llpp=(volatile long long*)0x10010030; // fir input
  llp=(volatile long long*)0x70000020;   // dma len
  *llp=(volatile long long)32; // starts transfer
  llp=(volatile long long*)0x70000000;   // dma st
  while (*llp);

  printf("Starting second round of FIR computation\n");
  llp=(volatile long long*)0x70010000;  //reset status register
  *llp = 0x07;
  llp=(volatile long long*)0x70010008; // fir ctrl
  *llp=(volatile long long)0x03;       // Start computation cycle 1
  *llp=(volatile long long)0x02;       // Start computation cycle 1
  llp=(volatile long long*)0x70010000;  //reset status register
  while(*llp != 0x03) {printf("Core waiting for FIR unit\n");};

  printf("Copying second batch of FIR outputs to memory\n");
  llpp=(volatile long long**)0x70000010; // dma sr
  *llpp=(volatile long long*)(0x10010050); // memctl coef1 address
  llpp=(volatile long long**)0x70000018; // dma dr
  *llpp=(volatile long long*)0x00001020; // fir input
  llp=(volatile long long*)0x70000020;   // dma len
  *llp=(volatile long long)32; // starts transfer
  llp=(volatile long long*)0x70000000;   // dma st
  while (*llp);

  
  short total_error=0;
  short error;
  short *output=(short *)0x60001000;
  for (int n=0; n<TSTEP-TAPS; n++) {
    error=expected[n]-output[n];              // Error for this time-step
    total_error+=(error<0)?(-error):(error);  // Absolute value
    // Uncomment the next line for a detailed error check
    //printf("cpu main k: %d output: %d expected %d\n",n,output[n],expected[n]);
  }

  printf("cpu main error: %d\n",error);

  llp=(volatile long long*)0x70010008;
  *llp=(volatile long long)0x0f; 
}
