//======================================================================
//
// uart.v
// ------
// Top level wrapper for the uart core.
//
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

            // External interface.
            input wire           rxd,
            output wire          txd,

            // Internal receive interface.
            output wire          rxd_syn,
            output [7 : 0]       rxd_data,
            input wire           rxd_ack,

            // Internal transmit interface.
            input wire           txd_syn,
            input wire [7 : 0]   txd_data,
            output wire          txd_ack,
            
            // API interface.
            input wire           cs,
            input wire           we,
            input wire [3 : 0]   address,
            input wire [31 : 0]  write_data,
            output wire [31 : 0] read_data,
            output wire          error,

            // Debug output.
            output wire [7 : 0]  debug
           );

  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // API addresses.
  parameter ADDR_CORE_NAME0   = 4'h0;
  parameter ADDR_CORE_NAME1   = 4'h1;
  parameter ADDR_CORE_TYPE    = 4'h2;
  parameter ADDR_CORE_VERSION = 4'h3;

  // Core ID constants.
  parameter CORE_NAME0   = 32'h75617274;  // "uart"
  parameter CORE_NAME1   = 32'h20202020;  // "    "
  parameter CORE_TYPE    = 32'h20202031;  // "   1"
  parameter CORE_VERSION = 32'h302e3031;  // "0.01"

  // The default clock rate is based on target clock frequency
  // divided by the bit rate times in order to hit the
  // center of the bits. I.e.
  // Clock: 50 MHz
  // Bitrate: 19200 bps
  // Divisor = 50*10E6 / 9600 = 5208
  parameter DEFAULT_CLK_RATE      = 5208;
  parameter DEFAULT_HALF_CLK_RATE = DEFAULT_CLK_RATE / 2;

  parameter DEFAULT_DATA_BITS = 8;
  parameter DEFAULT_STOP_BITS = 1;
 
  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------

  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  wire [15 : 0] bit_rate;
  wire [1 : 0]  stop_bits;

  wire         core_rxd;
  wire         core_txd;
  
  wire         core_rxd_syn;
  wire [7 : 0] core_rxd_data;
  wire         core_rxd_ack;

  wire         core_txd_syn;
  wire [7 : 0] core_txd_data;
  wire         core_txd_ack;

  reg [31 : 0] tmp_read_data;
  reg          tmp_error;

  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign txd           = core_txd;
  assign core_rxd      = rxd;

  assign rxd_syn       = core_rxd_syn;
  assign rxd_data      = core_rxd_data;
  assign core_rxd_ack  = rxd_ack;
  
  assign core_txd_syn  = txd_syn;
  assign core_txd_data = txd_data;
  assign txd_ack       = core_txd_ack;
  
  assign read_data     = tmp_read_data;
  assign error         = tmp_error;

  assign debug         = core_rxd_data;
  

  //----------------------------------------------------------------
  // core
  //
  // Instantiation of the uart core.
  //----------------------------------------------------------------
  uart_core core(
                 .clk(clk),
                 .reset_n(reset_n),

                 // Configuration parameters
                 .bit_rate(bit_rate),
                 .stop_bits(stop_bits),
                 
                 // External data interface
                 .rxd(core_rxd),
                 .txd(core_txd),

                 // Internal receive interface.
                 .rxd_syn(core_rxd_syn),
                 .rxd_data(core_rxd_data),
                 .rxd_ack(core_rxd_ack),
                 
                 // Internal transmit interface.
                 .txd_syn(core_txd_syn),
                 .txd_data(core_txd_data),
                 .txd_ack(core_txd_ack)
                );

  
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

        end
      else
        begin

        end
    end // reg_update

  
  //----------------------------------------------------------------
  // api
  //
  // The core API that allows an internal host to control the
  // core functionality.
  //----------------------------------------------------------------
  always @*
    begin: api
      // Default assignments.
      tmp_read_data = 32'h00000000;
      tmp_error     = 0;
      
      if (cs)
        begin
          if (we)
            begin
              // Write operations.
              case (address)
                
                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
          else
            begin
              // Read operations.
              case (address)
                ADDR_CORE_NAME0:
                  begin
                    tmp_read_data = CORE_NAME0;
                  end

                ADDR_CORE_NAME1:
                  begin
                    tmp_read_data = CORE_NAME1;
                  end

                ADDR_CORE_TYPE:
                  begin
                    tmp_read_data = CORE_TYPE;
                  end

                ADDR_CORE_VERSION:
                  begin
                    tmp_read_data = CORE_VERSION;
                  end
                
                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
        end
    end
  
endmodule // uart

//======================================================================
// EOF uart.v
//======================================================================
