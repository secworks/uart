//======================================================================
//
// uart.v
// ------
// A simple universal asynchronous receiver/transmitter (UART)
// interface. The interface contains 16 byte wide transmit and 
// receivea buffers and can handle start and stop bits. But in 
// general is rather simple. The primary purpose is as host 
// interface for the coretest design. The core also has a
// loopback mode to allow testing of a serial link.
//
// Note that the UART has a separate API interface to allow
// a control core to change settings such as speed. But the core
// has default values to allow it to start operating directly
// after reset. No config should be needed.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2014  Secworks Sweden AB
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

module uart(
            input wire           clk,
            input wire           reset_n,

            // External interface
            input wire           rxd,
            output wire          txd,

            // Debug
            output wire [7 : 0]  debug_out
           );

  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // The default clock rate is based on target clock frequency
  // divided by the bit rate times in order to hit the
  // center of the bits. I.e.
  // Clock: 50 MHz
  // Bitrate: 19200 bps
  // Divisor = 50*10E6 / 9600 = 5208
  parameter DEFAULT_CLK_RATE  = 5208;
  parameter DEFAULT_CLK_RATE2 = DEFAULT_CLK_RATE / 2;

  parameter DEFAULT_DATA_BITS = 8;
  
  parameter ERX_IDLE  = 0; 
  parameter ERX_START = 1;
  parameter ERX_BITS  = 2;
  parameter ERX_STOP  = 3;
 
  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          rxd_reg;

  reg [7 : 0]  rxd_byte_reg;
  reg [7 : 0]  rxd_byte_new;
  reg          rxd_byte_we;

  reg [4 : 0]  rxd_bit_ctr_reg;
  reg [4 : 0]  rxd_bit_ctr_new;
  reg          rxd_bit_ctr_we;
  reg          rxd_bit_ctr_rst;
  reg          rxd_bit_ctr_inc;

  reg [15 : 0] rxd_bitrate_ctr_reg;
  reg [15 : 0] rxd_bitrate_ctr_new;
  reg          rxd_bitrate_ctr_we;
  reg          rxd_bitrate_ctr_rst;
  reg          rxd_bitrate_ctr_inc;

  reg [2 : 0]  erx_ctrl_reg;
  reg [2 : 0]  erx_ctrl_new;
  reg          erx_ctrl_we;
  
  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------

  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign txd       = rxd_reg;
//  assign debug_out = {rxd_byte_reg;
  assign debug_out = {rxd_reg, rxd_reg, rxd_reg, rxd_reg,
                      rxd_reg, rxd_reg, rxd_reg, rxd_reg};
 
  
  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin: reg_update
      if (!reset_n)
        begin
          rxd_reg             <= 0;
          rxd_byte_reg        <= 8'h00;

          rxd_bit_ctr_reg     <= 4'h0;
          rxd_bitrate_ctr_reg <= 16'h0000;
          
          erx_ctrl_reg        <= ERX_IDLE;
        end
      else
        begin
          // We sample the rx input port every cycle.
          rxd_reg <= rxd;

          if (rxd_byte_we)
            begin
              rxd_byte_reg <= {rxd_byte_reg[6 : 1], rxd_reg};
            end

          if (rxd_bit_ctr_we)
            begin
              rxd_bit_ctr_reg <= rxd_bit_ctr_new;
            end

          if (rxd_bitrate_ctr_we)
            begin
              rxd_bitrate_ctr_reg <= rxd_bitrate_ctr_new;
            end
          
          if (erx_ctrl_we)
            begin
              erx_ctrl_reg <= erx_ctrl_new;
            end
        end
    end // reg_update


  //----------------------------------------------------------------
  // rxd_bit_ctr
  //
  // Bit counter for receiving data on the external 
  // serial interface.
  //----------------------------------------------------------------
  always @*
    begin: rxd_bit_ctr
      rxd_bit_ctr_new = 4'h0;
      rxd_bit_ctr_we  = 0;

      if (rxd_bit_ctr_rst)
        begin
          rxd_bit_ctr_new = 4'h0;
          rxd_bit_ctr_we  = 1;

        end
      else if (rxd_bit_ctr_inc)
        begin
          rxd_bit_ctr_new = rxd_bit_ctr_reg + 4'b0001;
          rxd_bit_ctr_we  = 1;
        end
    end // rxd_bit_ctr


  //----------------------------------------------------------------
  // rxd_bitrate_ctr
  //
  // Bitrate counter for receiving data on the external 
  // serial interface.
  //----------------------------------------------------------------
  always @*
    begin: rxd_bitrate_ctr
      rxd_bitrate_ctr_new = 16'h0000;
      rxd_bitrate_ctr_we  = 0;

      if (rxd_bitrate_ctr_rst)
        begin
          rxd_bitrate_ctr_new = 16'h0000;
          rxd_bitrate_ctr_we  = 1;

        end
      else if (rxd_bitrate_ctr_inc)
        begin
          rxd_bitrate_ctr_new = rxd_bitrate_ctr_reg + 16'h0001;
          rxd_bitrate_ctr_we  = 1;
        end
    end // rxd_bitrate_ctr


  //----------------------------------------------------------------
  // external_rx_engine
  //
  // Logic that implements the receive engine towards the externa
  // interface. Detects incoming data, collects it, if required 
  // checks parity and store correct data into the rx buffer.
  //----------------------------------------------------------------
  always @*
    begin: external_rx_engine
      rxd_bit_ctr_rst     = 0;
      rxd_bit_ctr_inc     = 0;
      rxd_bitrate_ctr_rst = 0;
      rxd_bitrate_ctr_inc = 0;
      rxd_byte_we         = 0;
      erx_ctrl_new        = ERX_IDLE;
      erx_ctrl_we         = 0;
      
      case (erx_ctrl_reg)
        ERX_IDLE:
          begin
            if (!rxd_reg)
              begin
                // Possible start bit detected.
                rxd_bitrate_ctr_rst = 1;
                erx_ctrl_new        = ERX_START;
                erx_ctrl_we         = 1;
              end
          end


        ERX_START:
          begin
            if (rxd_reg)
              begin
                // No real start bit, just a glitch.
                erx_ctrl_new        = ERX_IDLE;
                erx_ctrl_we         = 1;
              end
            else if (rxd_bitrate_ctr_reg == DEFAULT_CLK_RATE2)
              begin
                // Start bit assumed. We start sampling data.
                rxd_bit_ctr_rst     = 1;
                rxd_bitrate_ctr_rst = 1;
                erx_ctrl_new        = ERX_BITS;
                erx_ctrl_we         = 1;
              end
          end

        
        ERX_BITS:
          begin
            if (rxd_bitrate_ctr_reg == DEFAULT_CLK_RATE)
              begin
                rxd_byte_we     = 1;
                rxd_bit_ctr_inc = 1;
                if (rxd_bit_ctr_reg == DEFAULT_DATA_BITS)
                  begin
                    erx_ctrl_new = ERX_IDLE;
                    erx_ctrl_we  = 1;
                  end
              end
          end
            
        default:
          begin

          end
      endcase // case (erx_ctrl_reg)
      
    end // external_rx_engine
  
endmodule // uart

//======================================================================
// EOF uart.v
//======================================================================
