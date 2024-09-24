// AHB-lite slave.
//
// The AHB slave includes a memory array from which data is read and to which
// data is written according to the received packets from the master.
module AHB_slave(

    // global signals
    i_hclk, // clock
    i_hresetn, // reset, active low

    // control signals aka. master signals (Table 2-2 Master signals)
    // Destination: Slave, Source: Master and decoder
    i_haddr, // the address the master wants to either write to or read from (Used in the address phase)
    //i_hburst, // type of transfer (SINGLE_TRANSFER or BURST, What type of burst, amount of beats) (See Table 3-3 Burst signal encoding)
    i_hwrite, // write transfer (= HIGH), read transfer (= LOW)
    i_hsize, // size of data type to transmit (See Table 3-2 Transfer size encoding)
    i_htrans, // transfer type (IDLE, BUSY, NONSEQ, SEQ) (See Table 3-2 Transfer size encoding. )
    i_hreadyin, // ready signal is feed back into all slaves! (??? Why)
    i_hwdata, // the data to write if i_hwrite signals a write transfer (used in the data phase)
    //i_hprot, // PROTECTION, feature is not used here yet. (See Table 3-4 Protection signal encoding)
    //i_hmastlock, // ???, feature is not used here yet. When HIGH, this signal indicates that the current transfer is part of a locked sequence. See Locked transfers on page 3-7 for more information.

    // control signals aka. decoder signals (Table 2-4 Decoder signals)
    // Destination: Slave
    i_hsel, // when the decoder activates this signal, this slave will respond to transfers

    // output signals
    // Destination: all slaves and/or multiplexor
    o_hreadyout, // allows the slave to insert wait states (= low) or signal the transfer is done (= high)
    o_hresp, // OK (= low) or ERROR (= high). An error issued by the client will terminate a burst.
    o_hrdata // in case of a read transfer, returns the read data to the multiplexor (used in the data phase)
);

// Parameters
parameter ADDR_WIDTH = 32;              // Address bus width
parameter DATA_WIDTH = 32;              // Data bus width
parameter MEMORY_DEPTH = 512;           // Slave memory

parameter WAIT_WRITE = 0;               // Number of wait cycles issued by the slave in response to a 'write' transfer
parameter WAIT_READ = 0;                // Number of wait cycles issued by the slave in response to a 'read' transfer

parameter REGISTER_SELECT_BITS = 12;    // Memory mapping - each slave's internal memory has maximum 2^REGISTER_SELECT_BITS-1 bytes (depends on MEMORY_DEPTH)
parameter SLAVE_SELECT_BITS = 20;       // Memory mapping - width of slave address

localparam IDLE = 2'b00;                // Indicates that no data transfer is required
localparam BUSY = 2'b01;                // The BUSY transfer type enables masters to insert idle cycles in the middle of a burst
localparam NONSEQ = 2'b10;              // Indicates a single transfer or the first transfer of a burst
localparam SEQ = 2'b11;                 // The remaining transfers in a burst are SEQUENTIAL

localparam BYTE = 3'b000;               // Transfer size encodding for 1-byte transfers. Note: 32-bit databus is assumed
localparam HALFWORD = 3'b001;           // Transfer size encodding for 2-byte transfers, i.e. halfword. Note: 32-bit databus is assumed
localparam WORD = 3'b010;               // Transfer size encodding for 4-byte transfers, i.e. word. Note: 32-bit databus is assumed

// Inputs
input logic i_hclk;                     // All signal timings are related to the rising edge of hclk
input logic i_hresetn;                  // Active low bus reset

// Control signals from decoder
input logic i_hsel;                     // Slave select signal

// Control signals from master
input logic [ADDR_WIDTH-1:0] i_haddr;   // Address bus
input logic i_hwrite;                   // Indicates the transfer direction. Logic high values indicates a 'write' and logic low a 'read'
input logic [2:0] i_hsize;              // Indicates the size of the transfer, i.e. byte, half word or word
input logic [1:0] i_htrans;             // Indicates the transfer type, i.e. IDLE, BUSY, NONSEQUENTIAL, SEQUENTIAL
input logic i_hreadyin;                 // HREADY is also required as an input so that the slave can determine when the previously selected slave has completed its final transfer and the first data phase transfer for this slave is about to commence.
input logic [DATA_WIDTH-1:0] i_hwdata;  // Write data bus for 'write' transfers from the master to a slave

// Outputs
output logic o_hreadyout;               // Slave's 'o_hreadyout' signal indicates that a trasnfer has finished on the bus with the specific slave
output logic o_hresp;                   // Slaves 'hresp' signal is the transfer response : when LOW the transfer status is OK and when HIGH the status is ERROR
output logic [DATA_WIDTH-1:0] o_hrdata; // Slave's read data bus

// Internal signals
logic [MEMORY_DEPTH-1:0][7:0] mem;      // Default: 512 entries of a 1 byte each (single byte access is supported)
logic [4:0] wait_counter;               // Used to extend read/write transfers. Note: width can be parametrized if required
logic write_en;                         // write_en signal rises to logic high for one cycle when the write data is valid.
                                        // Write operation to the internl slave memory is synchronized to the positive edge
                                        // of the clock when this signal is high.
logic [2:0] hsize_samp;                 // Sampled packet. This is required to execute trasfers with wait states since the i_x signals are modified during the wait phase to accomodate the following transfer
logic [ADDR_WIDTH-1:0] haddr_samp;      // Sampled packet
logic [DATA_WIDTH-1:0] hwdata_samp;     // Sampled packet
logic hwrite_samp;                      // Sampled packet
logic hsel_samp;                        // Sampled packet

// HDL code

always @(posedge i_hclk or negedge i_hresetn)

    if (!i_hresetn)
    begin
        // OKAY response is issued during reset
        o_hresp <= 1'b0;
        wait_counter <= '0;
        write_en <= 1'b0;
    end

    // Slave is activated if on a positive edge hready is logic high
    // and i_sel is logic high (first clock cycle)
    // or
    // after it has been activated but inserts wait states
    else if ((i_hsel && i_hreadyin) || (hsel_samp && !o_hreadyout))
    begin

        // transfer type can be IDLE, BUSY, NONSEQ, SEQ
        case (i_htrans)

            // master will use IDLE when locked transfers are used or
            // when it does not want to perform any transaction (Why would
            // that be usefull?)
            IDLE:
            begin
                // Slave must provide zero wait state OKAY response to IDLE
                // transfers and the transfer must be ignored by the slave
                o_hresp <= 1'b0;

                // Issue logic high hready signal during IDLE state
                o_hreadyout <= 1'b1;
                wait_counter <= '0;
                write_en <= 1'b0;

                // Sampled i_hsel
                hsel_samp <= i_hsel;
            end

            default:
            begin

                // OKAY response is issued
                o_hresp <= 1'b0;

                // sample input
                if (i_hreadyin)
                begin
                    // Sampled i_hwrite (read (low) or write (high) transfer)
                    hwrite_samp <= i_hwrite;

                    // Sampled i_haddr
                    haddr_samp <= {{SLAVE_SELECT_BITS{1'b0}}, i_haddr[REGISTER_SELECT_BITS-1:0]};

                    // Sampled i_hsize (data type size)
                    hsize_samp <= i_hsize;

                    // Sampled i_hsel (if this slave is activated or not)
                    hsel_samp <= i_hsel;

                    // if this is a write transfer, then the data to write is already available on the i_hwdata signal
                    if (i_hwrite == 1'b1)
                    begin
                        // Sampled i_hwdata
                        hwdata_samp <= i_hwdata;
                    end
                end

                // Write transfer - considers i_hwrite only for the first cycle (see previous comments on the sampled packet)
                //
                // the slave has an internal wait_counter variable which is used to
                // simulate wait states for testing purposes.
                if ((i_hwrite) && (wait_counter == 0) || ((hwrite_samp) && (wait_counter > 0)))
                begin
                    // as long as the wait_counter has not been incremented to WAIT_READ, wait
                    if (wait_counter < $bits(wait_counter)'(WAIT_WRITE))
                    begin
                        // keep adding wait states

                        // send not ready signal to the multiplexor so that the master knows that the slave inserted a wait state
                        o_hreadyout <= 1'b0;

                        // increment counter
                        wait_counter <= wait_counter + $bits(wait_counter)'(1);

                        // write_en is used to synchronize the signal change for outgoing data to a rising clock cycle
                        // Do not write
                        write_en <= 1'b0;
                    end
                    else
                    begin
                        // wait is over, perform the write transfer

                        // send a ready signal to the master
                        o_hreadyout <= 1'b1;

                        // reset the wait counter
                        wait_counter <= '0;

                        // write_en is used to synchronize the signal change for outgoing data to a rising clock cycle
                        write_en <= 1'b1;
                    end
                end
                else
                //
                // Read transfer
                //
                begin
                    // as long as the wait_counter has not been incremented to WAIT_READ, wait
                    if (wait_counter < $bits(wait_counter)'(WAIT_READ))
                    begin
                        // keep adding wait states
                        o_hreadyout <= 1'b0;
                        wait_counter <= wait_counter + $bits(wait_counter)'(1);
                        write_en <= 1'b0;
                    end
                    else
                    begin
                        // wait is over, perform the read transfer
                        o_hreadyout <= 1'b1;
                        wait_counter <= '0;
                        write_en <= 1'b0;
                    end
                end
            end
        endcase
    end
    else
    begin
        // this state is entered when transfer is complete and also no reset takes place

        // ready is output
        o_hreadyout <= 1'b1;

        // slave is not selected
        hsel_samp <= 1'b0;
    end

// Execute the 'write' transfer:
//
// 'write_en' rises to logic high when the data is ready to be stored into the slave memory
// Write operation follows big endian: MSB byte is stored in haddr and LSB byte is haddr+x
always @(posedge i_hclk or negedge i_hresetn)
    if (!i_hresetn)

        // Memory is initialized to all zeros
        mem <= '0;

    else

        // when
        if ((o_hreadyout) && (write_en) && (hsel_samp))
        begin
            case (hsize_samp)

                BYTE :
                begin
                    mem[haddr_samp    ] <= i_hwdata[31:24];
                end

                HALFWORD :
                begin
                    mem[haddr_samp    ] <= i_hwdata[31:24];
                    mem[haddr_samp + 1] <= i_hwdata[23:16];
                end

                WORD :
                begin
                    mem[haddr_samp    ] <= i_hwdata[31:24];
                    mem[haddr_samp + 1] <= i_hwdata[23:16];
                    mem[haddr_samp + 2] <= i_hwdata[15:8];
                    mem[haddr_samp + 3] <= i_hwdata[7:0];
                end

            endcase
        end

// Execute the 'read' transfer
//
// HRADATA is a continuous assignment and not a synchronized signal - represnts the slave's internal calculation delay. This also solves the read-after-write issue since HRDATA is sampled by the master at the next positive edge (with hready high).
// For additional details and power-related consideration please see attached documentation.
always @(*)
    if (hwrite_samp == 1'b0)
    begin

        case (hsize_samp)
            BYTE :
            begin
                o_hrdata[31:24] = mem[haddr_samp];
                o_hrdata[23:0]  = '0;
            end

            HALFWORD :
            begin
                o_hrdata[31:24] = mem[haddr_samp];
                o_hrdata[23:16] = mem[haddr_samp+1];
                o_hrdata[15:0]  = '0;
            end

            WORD :
            begin
                o_hrdata[31:24] = mem[haddr_samp];
                o_hrdata[23:16] = mem[haddr_samp+1];
                o_hrdata[15:8]  = mem[haddr_samp+2];
                o_hrdata[7:0]   = mem[haddr_samp+3];
            end
        endcase

    end

endmodule
