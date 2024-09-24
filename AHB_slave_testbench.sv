`timescale 1ns/100ps

module AHB_slave_testbench();

//
// Slave Management
//

parameter SLAVE_COUNT = 1; // Number of connected AHB slaves

parameter WAIT_WRITE = 0; // Number of wait cycles issued by the slave in response to a 'write' transfer
parameter WAIT_READ = 0; // Number of wait cycles issued by the slave in response to a 'read' transfer

//
// Bus Signals
//

parameter ADDR_WIDTH = 32; // Address bus width
parameter DATA_WIDTH = 32; // Data bus width
parameter MEMORY_DEPTH = 1024; // Slave memory



//
// Global Signals
//

logic clk; // System's clock
logic resetn; // Active high logic

//
// Control signals (Decoder)
//

logic [SLAVE_COUNT-1:0] hsel; // Slave select bus

//
// Control Signals (Master)
//

logic [ADDR_WIDTH-1:0] haddr;
logic hwrite;                               // Indicates the transfer direction issued by Master 0
logic [2:0] hsize;                          // Indicates the size of the transfer issued by Master 0
logic [1:0] htrans;                         // Indicates the transfer type, i.e. IDLE, BUSY, NONSEQUENTIAL, SEQUENTIAL for Master 0
logic [DATA_WIDTH-1:0] hwdata;              // Write data bus for 'write' transfers from the master to a slave

//
// Output signal slave
//

logic hreadyout;                            // Slave 0 ready signal
logic hresp;                                // Slave 0 response signal
logic [DATA_WIDTH-1:0] hrdata;              // Read data bus of Slave 0

// Input signal slave
logic hreadyin;

//
// slave 0
//
AHB_slave #(.ADDR_WIDTH(ADDR_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .MEMORY_DEPTH(MEMORY_DEPTH),
    .WAIT_WRITE(WAIT_WRITE),
    .WAIT_READ(WAIT_READ))
s0(
    // global signals
    .i_hclk(clk),
    .i_hresetn(resetn),

    // control signals (decoder)
    .i_hsel(hsel[0]),

    // control signals (master)
    .i_haddr(haddr), // addr to write or read from
    .i_hwrite(hwrite), // low == read transfer, high == write transfer
    .i_hsize(hsize), // data type size
    .i_htrans(htrans), // transfer type (IDLE, BUSY, SEQ, NONSEQ)
    .i_hreadyin(hreadyin), // feedback ready to all slaves
    .i_hwdata(hwdata), // data to write during write transfer

    // output towards the interconnection fabric
    .o_hreadyout(hreadyout), // slave output, transfer is ready
    .o_hresp(hresp), // OKAY (= low) or ERROR (= HIGH)
    .o_hrdata(hrdata) // read data is returned by the slave here
);

always #100 clk = ~clk;

// Initial blocks
initial
begin

    #0
    resetn = 1'b0; // 0 means perform a reset
    clk = 1'b0;
    //hreadyout = 1'b1;
    hreadyin = 1'b1;
    htrans = 2'b01;

    // enable the first slave (this test bench has no decoder as it is a unit test for the slave). The slave has to be enabled manually
    hsel[0] = 1'b1;

    hsize = 3'b010;

    //
    // RESET
    //

    #100
    resetn = 1'b0;  // 0 means perform a reset

    #100
    resetn = 1'b0;

    //
    // NORMAL OPERATION
    //

    #100
    resetn = 1'b1; // no more reset. Lets start with normal operation

    // write transfer (burst: SINGLE_TRANSFER)

    // address phase
    #100
    haddr = 'h00000000;
    hwrite = 1'b1; // write transfer (= HIGH)
    // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    htrans = 2'b10; // NONSEQ
    hwdata = 'hCAFEBABE;

    #100
    haddr = 'h00000000;
    hwrite = 1'b1; // write transfer (= HIGH)
    // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    htrans = 2'b11; // SEQ
    hwdata = 'hCAFEBABE;

    // #100
    // haddr = 'h00000000;
    // hwrite = 1'b1; // write transfer (= HIGH)
    // // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    // htrans = 2'b10;
    // hwdata = 'hCAFEBABE;

    // #100
    // haddr = 'h00000000;
    // hwrite = 1'b1; // write transfer (= HIGH)
    // // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    // htrans = 2'b10;
    // hwdata = 'hCAFEBABE;

    //
    // Read transfer
    //

    #100
    haddr = 'h00000000;
    hwrite = 1'b0; // read transfer (= LOW)
    // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    htrans = 2'b10; // NONSEQ
    //hwdata = 'h00000000;

    #100
    haddr = 'h00000000;
    hwrite = 1'b0; // read transfer (= LOW)
    // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    htrans = 2'b11; // SEQ
    //hwdata = 'h00000000;

    // #100
    // haddr = 'h00000000;
    // hwrite = 1'b0; // read transfer (= LOW)
    // // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    // htrans = 2'b11;
    // //hwdata = 'h00000000;

    // #100
    // haddr = 'h00000000;
    // hwrite = 1'b0; // read transfer (= LOW)
    // // Indicates the transfer type, i.e. IDLE (b00), BUSY (b01), NONSEQUENTIAL (b10), SEQUENTIAL (b11) for Master 0
    // htrans = 2'b11;
    // //hwdata = 'h00000000;

    //#200
    // Initiate transfer task :
    // issues m consecutive transfers with randomized parameters (addr, size, width, etc.)
    //initiate_transfer(5000);

    //#1000
    //$display("\n -----------------------");
    //$display("\n ALL tests have passed - Hallelujah!");
end

endmodule