//======================================================================
//
// tb_uart.v
// ---------
// Testbench for the UART core.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2014, Secworks Sweden AB
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or 
// without modification, are permitted provided that the following 
// conditions are met: 
// 
// 1. Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer. 
// 
// 2. Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the 
//    distribution. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

//------------------------------------------------------------------
// Simulator directives.
//------------------------------------------------------------------
`timescale 1ns/10ps

module tb_uart();
  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG = 0;

  parameter CLK_PERIOD = 2;
  parameter CLK_HALF_PERIOD = 1;
  
  
  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;

  reg          tb_clk;
  reg          tb_reset_n;
  reg          tb_rxd;
  wire         tb_txd;
  wire [7 : 0] tb_debug;

  
  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  uart dut(
           .clk(tb_clk),
           .reset_n(tb_reset_n),
           
           .rxd(tb_rxd),
           .txd(tb_txd),

           .debug(tb_debug)
          );
  

  //----------------------------------------------------------------
  // clk_gen
  //
  // Clock generator process. 
  //----------------------------------------------------------------
  always 
    begin : clk_gen
      #CLK_HALF_PERIOD tb_clk = !tb_clk;
    end // clk_gen
    

  //----------------------------------------------------------------
  // sys_monitor
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      #(CLK_PERIOD);
      if (DEBUG)
        begin
          dump_dut_state();
        end
    end

  
  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dump when needed.
  //----------------------------------------------------------------
  task dump_dut_state();
    begin
      $display("State of DUT");
      $display("------------");
      $display("Inputs and outputs:");
      $display("rxd = 0x%01x, txd = 0x%01x, debug = 0x%01x", 
               dut.rxd, dut.txd, dut.debug);
      $display("");

      $display("Sample and data registers:");
      $display("rxd_reg = 0x%01x, rxd_byte_reg = 0x%01x", 
               dut.rxd_reg, dut.rxd_byte_reg);
      $display("");

      $display("Counters:");
      $display("rxd_bit_ctr_reg = 0x%01x, rxd_bitrate_ctr_reg = 0x%02x", 
               dut.rxd_bit_ctr_reg, dut.rxd_bitrate_ctr_reg);
      $display("");
      

      $display("Control signals and FSM state:");
      $display("erx_ctrl_reg = 0x%02x", 
               dut.erx_ctrl_reg);
      $display("");
    end
  endtask // dump_dut_state
  
  
  //----------------------------------------------------------------
  // reset_dut()
  //----------------------------------------------------------------
  task reset_dut();
    begin
      $display("*** Toggle reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
    end
  endtask // reset_dut

  
  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim();
    begin
      error_ctr = 0;
      tc_ctr = 0;
      
      tb_clk = 0;
      tb_reset_n = 1;
    end
  endtask // init_sim

  
  //----------------------------------------------------------------
  // display_test_result()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_result();
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d test cases did not complete successfully.", error_ctr);
        end
    end
  endtask // display_test_result
                         
    
  //----------------------------------------------------------------
  // sha1_core_test
  // The main test functionality. 
  //----------------------------------------------------------------
  initial
    begin : uart_test
      $display("   -- Testbench for uart core started --");

      init_sim();
      dump_dut_state();
      reset_dut();
      dump_dut_state();
      
      display_test_result();
      $display("*** Simulation done.");
      $finish;
    end // uart_test
endmodule // tb_uart

//======================================================================
// EOF tb_uart.v
//======================================================================
