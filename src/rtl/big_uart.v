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
            
            // Internal interface
            output wire          rx_syn,
            output wire [7 : 0]  rx_data,
            input wire           rx_ack,

            input wire           tx_syn,
            input wire [7 : 0]   tx_data,
            output wire          tx_ack,

            // Debug
            output wire [7 : 0]  debug,
            
            // API interface
            input wire           cs,
            input wire           we,
            input wire [3 : 0]   address,
            input wire [31 : 0]  write_data,
            output wire [31 : 0] read_data,
            output wire          error
           );
  
  
  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // API addresses.
  parameter ADDR_CORE_NAME0   = 4'h0;
  parameter ADDR_CORE_NAME1   = 4'h1;
  parameter ADDR_CORE_TYPE    = 4'h2;
  parameter ADDR_CORE_VERSION = 4'h3;
  
  parameter ADDR_CTRL         = 4'h8; // Enable/disable. Loopback on/off.
  parameter ADDR_STATUS       = 4'h9; // Buffer status.
  parameter ADDR_CONFIG       = 4'ha; // Num start, data, stop, parity bits.
  parameter ADDR_CLK_DIV      = 4'hb; // Clock divisor to set bitrate.

  // These are WTC registers. The value written is ignored.
  parameter ADDR_STAT_PARITY  = 4'hc; // Stats: Num parity errors detected.
  parameter ADDR_STAT_RX_FULL = 4'hd; // Stats: Num Rx buffer full events.
  parameter ADDR_STAT_TX_FULL = 4'he; // Stats: Num Tx buffer full events.

  // Core ID constants.
  parameter CORE_NAME0   = 32'h75617274;  // "uart"
  parameter CORE_NAME1   = 32'h20202020;  // "    "
  parameter CORE_TYPE    = 32'h20202031;  // "   1"
  parameter CORE_VERSION = 32'h302e3031;  // "0.01"

  // The default clock rate is based on target clock frequency
  // divided by the bit rate times in order to hit the
  // center of the bits. I.e.
  // Clock: 50 MHz
  // Bitrate: 1200 bps
  // Divisor = 5010E6 / (19200 * 4) = 651.041666
  // Divisor = 50E6 / (1200 * 4) = 10416.6667 
  parameter DEFAULT_CLK_DIV = 10417;

  parameter DEFAULT_START_BITS = 2'h1;
  parameter DEFAULT_STOP_BITS  = 2'h1;
  parameter DEFAULT_DATA_BITS  = 4'h8;
  parameter DEFAULT_PARITY     = 1'h0;
  parameter DEFAULT_ENABLE     = 1'h1;
  parameter DEFAULT_ILOOPBACK  = 1'h0;
  parameter DEFAULT_ELOOPBACK  = 1'h0;
  
  parameter ITX_IDLE = 0;
  parameter ITX_ACK  = 1;
  
  parameter ETX_IDLE   = 0;
  parameter ETX_START  = 1;
  parameter ETX_DATA   = 2;
  parameter ETX_PARITY = 3;
  parameter ETX_STOP   = 4;
  parameter ETX_DONE   = 5;
  
  
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [15 : 0] clk_div_reg;
  reg [15 : 0] clk_div_new;
  reg          clk_div_we;

  reg          enable_bit_reg;
  reg          enable_bit_new;
  reg          enable_bit_we;
  
  reg          iloopback_bit_reg;
  reg          iloopback_bit_new;
  reg          iloopback_bit_we;
  
  reg          eloopback_bit_reg;
  reg          eloopback_bit_new;
  reg          eloopback_bit_we;
  
  reg [1 : 0]  start_bits_reg;
  reg [1 : 0]  start_bits_new;
  reg          start_bits_we;

  reg [1 : 0]  stop_bits_reg;
  reg [1 : 0]  stop_bits_new;
  reg          stop_bits_we;

  reg [3 : 0]  data_bits_reg;
  reg [3 : 0]  data_bits_new;
  reg          data_bits_we;

  reg          parity_bit_reg;
  reg          parity_bit_new;
  reg          parity_bit_we;
  
  // Rx data buffer with associated
  // read and write pointers as well
  // as counter for number of elements
  // in the buffer.
  reg [7 : 0] rx_buffer [0 : 15];
  reg         rx_buffer_we;
  
  reg [3 : 0] rx_rd_ptr_reg;
  reg [3 : 0] rx_rd_ptr_new;
  reg         rx_rd_ptr_we;
  reg         rx_rd_ptr_inc;

  reg [3 : 0] rx_wr_ptr_reg;
  reg [3 : 0] rx_wr_ptr_new;
  reg         rx_wr_ptr_we;
  reg         rx_wr_ptr_inc;

  reg [3 : 0] rx_ctr_reg;
  reg [3 : 0] rx_ctr_new;
  reg         rx_ctr_we;
  reg         rx_ctr_inc;
  reg         rx_ctr_dec;

  // Tx data buffer with associated
  // read and write pointers as well
  // as counter for number of elements
  // in the buffer.
  reg [7 : 0] tx_buffer [0 : 15];
  reg         tx_buffer_we;

  reg [3 : 0] tx_rd_ptr_reg;
  reg [3 : 0] tx_rd_ptr_new;
  reg         tx_rd_ptr_we;
  reg         tx_rd_ptr_inc;

  reg [3 : 0] tx_wr_ptr_reg;
  reg [3 : 0] tx_wr_ptr_new;
  reg         tx_wr_ptr_we;
  reg         tx_wr_ptr_inc;

  reg [3 : 0] tx_ctr_reg;
  reg [3 : 0] tx_ctr_new;
  reg         tx_ctr_we;
  reg         tx_ctr_inc;
  reg         tx_ctr_dec;
  
  reg         rxd_reg;

  reg [7 : 0] rxd_byte_reg;
  reg [7 : 0] rxd_byte_new;
  reg         rxd_byte_we;

  reg         txd_reg;
  reg         txd_new;
  reg         txd_we;

  reg [7 : 0] txd_byte_reg;
  reg [7 : 0] txd_byte_new;
  reg         txd_byte_we;

  reg [2 : 0] rxd_bit_ctr_reg;
  reg [2 : 0] rxd_bit_ctr_new;
  reg         rxd_bit_ctr_we;
  reg         rxd_bit_ctr_rst;
  reg         rxd_bit_ctr_inc;

  reg [15 : 0] rxd_bitrate_ctr_reg;
  reg [15 : 0] rxd_bitrate_ctr_new;
  reg          rxd_bitrate_ctr_we;
  reg          rxd_bitrate_ctr_rst;
  reg          rxd_bitrate_ctr_inc;
  
  reg [2 : 0] txd_bit_ctr_reg;
  reg [2 : 0] txd_bit_ctr_new;
  reg         txd_bit_ctr_we;
  reg         txd_bit_ctr_rst;
  reg         txd_bit_ctr_inc;

  reg [15 : 0] txd_bitrate_ctr_reg;
  reg [15 : 0] txd_bitrate_ctr_new;
  reg          txd_bitrate_ctr_we;
  reg          txd_bitrate_ctr_rst;
  reg          txd_bitrate_ctr_inc;

  reg [31 : 0] rx_parity_error_ctr_reg;
  reg [31 : 0] rx_parity_error_ctr_new;
  reg          rx_parity_error_ctr_we;
  reg          rx_parity_error_ctr_inc;
  reg          rx_parity_error_ctr_rst;
  
  reg [31 : 0] rx_buffer_full_ctr_reg;
  reg [31 : 0] rx_buffer_full_ctr_new;
  reg          rx_buffer_full_ctr_we;
  reg          rx_buffer_full_ctr_inc;
  reg          rx_buffer_full_ctr_rst;
  
  reg [31 : 0] tx_buffer_full_ctr_reg;
  reg [31 : 0] tx_buffer_full_ctr_new;
  reg          tx_buffer_full_ctr_we;
  reg          tx_buffer_full_ctr_inc;
  reg          tx_buffer_full_ctr_rst;
               
  reg          itx_ctrl_reg;
  reg          itx_ctrl_new;
  reg          itx_ctrl_we;
               
  reg [2 : 0]  etx_ctrl_reg;
  reg [2 : 0]  etx_ctrl_new;
  reg          etx_ctrl_we;
  
  
  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] tmp_read_data;
  reg          tmp_error;

  reg          muxed_txd;
  reg          muxed_rxd_reg;

  reg          tmp_rx_syn;
  reg [7 : 0]  tmp_rx_data;
  reg          tmp_tx_ack;
  
  reg          internal_rx_syn;
  reg [7 : 0]  internal_rx_data;
  reg          internal_rx_ack;
  reg          internal_tx_syn;
  reg [7 : 0]  internal_tx_data;
  reg          internal_tx_ack;
  
  reg          rx_empty;
  reg          rx_full;
  reg          tx_empty;
  reg          tx_full;

  
  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign txd = muxed_txd;

  assign rx_syn  = tmp_rx_syn;
  assign rx_data = tmp_rx_data;
  assign tx_ack  = tmp_tx_ack;
  
  assign read_data = tmp_read_data;
  assign error     = tmp_error;
  
  assign debug = {rxd_reg, rxd_reg, rxd_reg, rxd_reg, 
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
          clk_div_reg             <= DEFAULT_CLK_DIV;
          start_bits_reg          <= DEFAULT_START_BITS;
          stop_bits_reg           <= DEFAULT_STOP_BITS;
          data_bits_reg           <= DEFAULT_DATA_BITS;
          parity_bit_reg          <= DEFAULT_PARITY;
          enable_bit_reg          <= DEFAULT_ENABLE;
          iloopback_bit_reg       <= DEFAULT_ILOOPBACK;
          eloopback_bit_reg       <= DEFAULT_ELOOPBACK;
          
          rxd_reg                 <= 0;
          rxd_byte_reg            <= 8'h00;
          txd_reg                 <= 0;
          txd_byte_reg            <= 8'h00;
          
          rx_rd_ptr_reg           <= 4'h0;
          rx_wr_ptr_reg           <= 4'h0;
          rx_ctr_reg              <= 4'h0;
          tx_rd_ptr_reg           <= 4'h0;
          tx_wr_ptr_reg           <= 4'h0;
          tx_ctr_reg              <= 4'h0;

          rxd_bit_ctr_reg         <= 3'b000;
          rxd_bitrate_ctr_reg     <= 16'h0000;
          txd_bit_ctr_reg         <= 3'b000;
          txd_bitrate_ctr_reg     <= 16'h0000;
          
          rx_parity_error_ctr_reg <= 32'h00000000;
          rx_buffer_full_ctr_reg  <= 32'h00000000;
          tx_buffer_full_ctr_reg  <= 32'h00000000;

          itx_ctrl_reg            <= ITX_IDLE;
          etx_ctrl_reg            <= ETX_IDLE;
        end
      else
        begin
          // We sample the rx input port every cycle.
          rxd_reg <= rxd;
          
          if (rxd_byte_we)
            begin
              rxd_byte_reg <= {rxd_byte_reg[6 : 1], rxd_reg};
            end

          if (txd_we)
            begin
              txd_reg <= txd_new;
            end

          if (txd_byte_we)
            begin
              txd_byte_reg <= tx_buffer[tx_rd_ptr_reg];
            end
                    
          if (clk_div_we)
            begin
              clk_div_reg <= clk_div_new;
            end

          if (start_bits_we)
            begin
              start_bits_reg <= start_bits_new;
            end

          if (stop_bits_we)
            begin
              stop_bits_reg <= stop_bits_new;
            end

          if (data_bits_we)
            begin
              data_bits_reg <= data_bits_new;
            end

          if (parity_bit_we)
            begin
              parity_bit_reg <= parity_bit_new;
            end

          if (enable_bit_we)
            begin
              enable_bit_reg <= enable_bit_new;
            end

          if (iloopback_bit_we)
            begin
              iloopback_bit_reg <= iloopback_bit_new;
            end

          if (eloopback_bit_we)
            begin
              eloopback_bit_reg <= eloopback_bit_new;
            end

          if (rx_buffer_we)
            begin
              rx_buffer[rx_wr_ptr_reg] <= rxd_byte_reg;
            end

          if (tx_buffer_we)
            begin
              tx_buffer[tx_wr_ptr_reg] <= tx_data;
            end
          
          if (rx_rd_ptr_we)
            begin
              rx_rd_ptr_reg <= rx_rd_ptr_new;
            end
          
          if (rx_wr_ptr_we)
            begin
              rx_wr_ptr_reg <= rx_wr_ptr_new;
            end
          
          if (rx_ctr_we)
            begin
              rx_ctr_reg <= rx_ctr_new;
            end
          
          if (tx_rd_ptr_we)
            begin
              tx_rd_ptr_reg <= tx_rd_ptr_new;
            end
          
          if (tx_wr_ptr_we)
            begin
              tx_wr_ptr_reg <= tx_wr_ptr_new;
            end
          
          if (tx_ctr_we)
            begin
              tx_ctr_reg <= tx_ctr_new;
            end

          if (rx_parity_error_ctr_we)
            begin
              rx_parity_error_ctr_reg <= rx_parity_error_ctr_new;
            end

          if (rx_buffer_full_ctr_we)
            begin
              rx_buffer_full_ctr_reg <= rx_buffer_full_ctr_new;
            end

          if (tx_buffer_full_ctr_we)
            begin
              tx_buffer_full_ctr_reg <= tx_buffer_full_ctr_new;
            end

          if (rxd_bit_ctr_we)
            begin
              rxd_bit_ctr_reg <= rxd_bit_ctr_new;
            end

          if (rxd_bitrate_ctr_we)
            begin
              rxd_bitrate_ctr_reg <= rxd_bitrate_ctr_new;
            end

          if (txd_bit_ctr_we)
            begin
              txd_bit_ctr_reg <= txd_bit_ctr_new;
            end

          if (txd_bitrate_ctr_we)
            begin
              txd_bitrate_ctr_reg <= txd_bitrate_ctr_new;
            end

          if (itx_ctrl_we)
            begin
              itx_ctrl_reg <= itx_ctrl_new;
            end

          if (etx_ctrl_we)
            begin
              etx_ctrl_reg <= etx_ctrl_new;
            end
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
      tmp_read_data           = 32'h00000000;
      tmp_error               = 0;
      clk_div_new             = 16'h0000;
      clk_div_we              = 0;
      enable_bit_new          = 0;
      enable_bit_we           = 0;
      iloopback_bit_new       = 0;
      iloopback_bit_we        = 0;
      eloopback_bit_new       = 0;
      eloopback_bit_we        = 0;
      start_bits_new          = 2'b00;
      start_bits_we           = 0;
      stop_bits_new           = 2'b00 ;
      stop_bits_we            = 0;
      data_bits_new           = 4'h0;
      data_bits_we            = 0;
      parity_bit_new          = 0;
      parity_bit_we           = 0;
      rx_parity_error_ctr_rst = 0;
      rx_buffer_full_ctr_rst  = 0;
      tx_buffer_full_ctr_rst  = 0;
      
      if (cs)
        begin
          if (we)
            begin
              // Write operations.
              case (address)
                ADDR_CTRL:
                  begin
                    enable_bit_new   = write_data[0];
                    enable_bit_we    = 1;
                    iloopback_bit_new = write_data[1];
                    iloopback_bit_we  = 1;
                    eloopback_bit_new = write_data[2];
                    eloopback_bit_we  = 1;
                  end

                ADDR_CONFIG:
                  begin
                    start_bits_new = write_data[1 : 0];
                    start_bits_we  = 1;
                    stop_bits_new  = write_data[3 : 2];
                    stop_bits_we   = 1;
                    data_bits_new  = write_data[7 : 4];
                    data_bits_we   = 1;
                    parity_bit_new = write_data[8];
                    parity_bit_we  = 1;
                  end
                
                ADDR_CLK_DIV:
                  begin
                    clk_div_new = write_data[15 : 0];
                    clk_div_we  = 1;
                  end

                ADDR_STAT_PARITY:
                  begin
                    // Note that we ignore the data being written.
                    rx_parity_error_ctr_rst = 1;
                  end

                ADDR_STAT_RX_FULL:
                  begin
                    // Note that we ignore the data being written.
                    rx_buffer_full_ctr_rst = 1;
                  end

                ADDR_STAT_TX_FULL:
                  begin
                    // Note that we ignore the data being written.
                    tx_buffer_full_ctr_rst = 1;
                  end
                
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

                ADDR_CTRL:
                  begin
                    tmp_read_data = {28'h0000000, 2'b01, eloopback_bit_reg,
                                     iloopback_bit_reg, enable_bit_reg};
                  end
                
                ADDR_STATUS:
                  begin
                    tmp_read_data = {24'h000000, tx_ctr_reg, rx_ctr_reg};
                  end
                
                ADDR_CONFIG:
                  begin
                    tmp_read_data = {20'h00000, 3'b000, 
                                     parity_bit_reg, data_bits_reg,
                                     stop_bits_reg, start_bits_reg};
                  end
                
                ADDR_CLK_DIV:
                  begin
                    tmp_read_data = {16'h0000, clk_div_reg};
                  end

                ADDR_STAT_PARITY:
                  begin
                    tmp_read_data = rx_parity_error_ctr_reg;
                  end
                
                ADDR_STAT_RX_FULL:
                  begin
                    tmp_read_data = rx_buffer_full_ctr_reg;
                  end
                
                ADDR_STAT_TX_FULL:
                  begin
                    tmp_read_data = tx_buffer_full_ctr_reg;
                  end
                
                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
        end
    end

  
  //----------------------------------------------------------------
  // eloopback_mux
  //
  // The mux controlled by the eloopback_bit_reg. If set the
  // interfaces towards the external system is tied together
  // making the UART echoing received data back to 
  // the external host.
  //----------------------------------------------------------------
  always @*
    begin: eloopback_mux
      if (eloopback_bit_reg)
        begin
          muxed_rxd_reg = 8'hff;
          muxed_txd     = rxd_reg;
        end
      else
        begin
          muxed_rxd_reg = rxd_reg;
          muxed_txd = txd_reg;
        end
    end // eloopback_mux

  
  //----------------------------------------------------------------
  // iloopback_mux
  //
  // The mux controlled by the iloopback_bit_reg. If set the
  // interfaces towards the internal system is tied together
  // making the UART echoing received back to the external host 
  // via the buffers and serial/parallel conversions
  //----------------------------------------------------------------
//  always @*
//    begin: iloopback_mux
//      if (iloopback_bit_reg)
//        begin
//          internal_tx_syn  = internal_rx_syn;
//          internal_tx_data = internal_rx_data;
//          internal_rx_ack  = internal_tx_ack;
//
//          tmp_rx_syn  = 0;
//          tmp_rx_data = 8'h00;
//          tmp_tx_ack  = 0;
//        end
//      else
//        begin
//          tmp_rx_syn       = internal_rx_syn;
//          tmp_rx_data      = internal_rx_data;
//          internal_rx_ack  = rx_ack;
//
//          internal_xx_syn  = tx_syn;
//          internal_tx_data = tx_data;
//          tmp_tx_ack       = internal_tx_ack;
//        end
//    end // iloopback_mux
//

  //----------------------------------------------------------------
  // rx_rd_ptr
  //
  // Read pointer for the receive buffer.
  //----------------------------------------------------------------
  always @*
    begin: rx_rd_ptr
      rx_rd_ptr_new = 4'h00;
      rx_rd_ptr_we  = 0;

      if (rx_rd_ptr_inc)
        begin
          rx_rd_ptr_new = rx_rd_ptr_reg + 1'b1;
          rx_rd_ptr_we  = 1;
        end
    end // rx_rd_ptr


  //----------------------------------------------------------------
  // rx_wr_ptr
  //
  // Write pointer for the receive buffer.
  //----------------------------------------------------------------
  always @*
    begin: rx_wr_ptr
      rx_wr_ptr_new = 4'h00;
      rx_wr_ptr_we  = 0;

      if (rx_wr_ptr_inc)
        begin
          rx_wr_ptr_new = rx_wr_ptr_reg + 1'b1;
          rx_wr_ptr_we  = 1;
        end
    end // rx_wr_ptr


  //----------------------------------------------------------------
  // rx_ctr
  //
  // Counter for the receive buffer.
  //----------------------------------------------------------------
  always @*
    begin: rx_ctr
      rx_ctr_new = 4'h00;
      rx_ctr_we  = 0;
      rx_empty   = 0;

      if (rx_ctr_reg == 4'h0)
        begin
          rx_empty = 1;
        end
      
      if ((rx_ctr_inc) && (!rx_ctr_dec))
        begin
          rx_ctr_new = rx_ctr_reg + 1'b1;
          rx_ctr_we  = 1;
        end
      else if ((!rx_ctr_inc) && (rx_ctr_dec))
        begin
          rx_ctr_new = rx_ctr_reg - 1'b1;
          rx_ctr_we  = 1;
        end
    end // rx_ctr

  
  //----------------------------------------------------------------
  // tx_rd_ptr
  //
  // Read pointer for the transmit buffer.
  //----------------------------------------------------------------
  always @*
    begin: tx_rd_ptr
      tx_rd_ptr_new = 4'h00;
      tx_rd_ptr_we  = 0;

      if (tx_rd_ptr_inc)
        begin
          tx_rd_ptr_new = tx_rd_ptr_reg + 1'b1;
          tx_rd_ptr_we  = 1;
        end
    end // tx_rd_ptr


  //----------------------------------------------------------------
  // tx_wr_ptr
  //
  // Write pointer for the transmit buffer.
  //----------------------------------------------------------------
  always @*
    begin: tx_wr_ptr
      tx_wr_ptr_new = 4'h00;
      tx_wr_ptr_we  = 0;

      if (tx_wr_ptr_inc)
        begin
          tx_wr_ptr_new = tx_wr_ptr_reg + 1'b1;
          tx_wr_ptr_we  = 1;
        end
    end // tx_wr_ptr


  //----------------------------------------------------------------
  // tx_ctr
  //
  // Counter for the transmit buffer.
  //----------------------------------------------------------------
  always @*
    begin: tx_ctr
      tx_ctr_new = 4'h0;
      tx_ctr_we  = 0;
      tx_full    = 0;

      if (tx_ctr_reg == 4'hf)
        begin
          tx_full = 1;
        end
      
      if ((tx_ctr_inc) && (!tx_ctr_dec))
        begin
          tx_ctr_new = tx_ctr_reg + 1'b1;
          tx_ctr_we  = 1;
        end
      else if ((!tx_ctr_inc) && (tx_ctr_dec))
        begin
          tx_ctr_new = tx_ctr_reg - 1'b1;
          tx_ctr_we  = 1;
        end
    end // tx_ctr


  //----------------------------------------------------------------
  // external_rx_engine
  //
  // Logic that implements the receive engine towards the externa
  // interface. Detects incoming data, collects it, if required 
  // checks parity and store correct data into the rx buffer.
  //----------------------------------------------------------------
  always @*
    begin: external_rx_engine
      

    end // external_rx_engine

  
  //----------------------------------------------------------------
  // external_tx_engine
  //
  // Logic that implements the transmit engine towards the external
  // interface. When there is data in the tx buffer, the engine 
  // transmits the data including start, stop and possible 
  // parity bits.
  //----------------------------------------------------------------
  always @*
    begin: external_tx_engine
      tx_rd_ptr_inc       = 0;
      tx_ctr_dec          = 0;
      txd_byte_we         = 0;
      txd_we              = 0;
      txd_bit_ctr_rst     = 0;
      txd_bit_ctr_inc     = 0;
      txd_bitrate_ctr_rst = 0;
      txd_bitrate_ctr_inc = 0;
      etx_ctrl_new        = ETX_IDLE;
      etx_ctrl_we         = 0;
      
      case (etx_ctrl_reg)
        ETX_IDLE:
          begin
            if (!tx_empty)
              begin
                txd_byte_we         = 1;
                txd_bit_ctr_rst     = 1;
                txd_bitrate_ctr_rst = 1;
                tx_rd_ptr_inc       = 1;
                tx_ctr_dec          = 1;
                etx_ctrl_new        = ETX_START;
                etx_ctrl_we         = 1;
              end
          end

      endcase // case (etx_ctrl_reg)
    end // external_tx_engine


  //----------------------------------------------------------------
  // internal_rx_engine
  //
  // Logic that implements the receive engine towards the internal
  // interface. When there is data in the rx buffer it asserts
  // the syn flag to signal that there is data available on 
  // rx_data. When the ack signal is asserted the syn flag is
  // dropped and the data is considered to have been consumed and
  // can be discarded.
  //----------------------------------------------------------------
  always @*
    begin: internal_rx_engine
    end // internal_rx_engine


  //----------------------------------------------------------------
  // internal_tx_engine
  //
  // Logic that implements the transmit engine towards the internal
  // interface. When the tx_syn flag is asserted the engine checks
  // if there are any room in the tx buffer. If it is, the data
  // available at tx_data is stored in the buffer. The tx_ack
  // is then asserted. The engine then waits for the syn flag
  // to be dropped.
  //----------------------------------------------------------------
  always @*
    begin: internal_tx_engine
      // Default assignments
      tx_buffer_we  = 0;
      tx_wr_ptr_inc = 0;
      tx_ctr_inc    = 0;
      tmp_tx_ack    = 0;
      itx_ctrl_new  = ITX_IDLE;
      itx_ctrl_we   = 0;
      
      case (itx_ctrl_reg)
        ITX_IDLE:
          begin
            if (tx_syn)
              begin
                if (!tx_full)
                  begin
                    tx_buffer_we  = 1;
                    tx_wr_ptr_inc = 1;
                    tx_ctr_inc    = 1;
                    itx_ctrl_new  = ITX_ACK;
                    itx_ctrl_we   = 1;
                  end
              end
          end

        ITX_ACK:
          begin
            tmp_tx_ack = 1;
            if (!tx_syn)
              begin
                itx_ctrl_new = ITX_IDLE;
                itx_ctrl_we  = 1;
              end
          end
      endcase // case (itx_ctrl_reg)
    end // internal_tx_engine
  
endmodule // uart

//======================================================================
// EOF uart.v
//======================================================================
