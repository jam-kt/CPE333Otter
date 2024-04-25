`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/08/2023 09:13:25 PM
// Design Name: 
// Module Name: OTTER_MCU_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module OTTER_tb(

    );
    
    logic RST, INTR, CLK, IOBUS_WR;
    logic [31:0] IOBUS_IN, IOBUS_OUT, IOBUS_ADDR;
    
    OTTER UUT(.RST(RST), .IOBUS_IN(IOBUS_IN), .CLK(CLK), .IOBUS_WR(IOBUS_WR), .IOBUS_OUT(IOBUS_OUT), .IOBUS_ADDR(IOBUS_ADDR));
    
    always #5 CLK = !CLK;
    
    initial begin
        CLK = 1;
        RST = 1;
        IOBUS_IN = 32'b00;
//        IOBUS_IN = 32'b00;
        #10
        RST = 0;
    end
 endmodule