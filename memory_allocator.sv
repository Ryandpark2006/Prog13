module memory(
    input wire [63:0] pc,    // instruction address
    input wire clk, reset,
    input wire mem_write_enable, // control signal for write enable
    input wire [63:0] rw_val,       // the value that you want to read/write
    input wire [31:0] rw_addr,      // read/write address 
    output wire [31:0] instruction, 
    output wire [63:0] r_out        // 8-byte data output
);
    reg [7:0] bytes [524288-1:0];

    // read instruction
    assign instruction = {bytes[pc+3], bytes[pc+2], bytes[pc+1], bytes[pc]};
    
    // read data
    assign r_out = {bytes[rw_addr+7], bytes[rw_addr+6], bytes[rw_addr+5], bytes[rw_addr+4], bytes[rw_addr+3], 
                   bytes[rw_addr+2], bytes[rw_addr+1], bytes[rw_addr]};
    
    // write data
    integer i;
    always @(posedge clk or posedge reset) begin
        if(reset) begin 
            for (i = 0; i < 524288; i = i + 1) begin
                bytes[i] <= 8'b0;  // Reset each byte in memory to 0s
            end
        end
        if (mem_write_enable) begin
            // Write all 8 bytes
            bytes[rw_addr+7] <= rw_val[63:56];
            bytes[rw_addr+6] <= rw_val[55:48];
            bytes[rw_addr+5] <= rw_val[47:40];
            bytes[rw_addr+4] <= rw_val[39:32];
            bytes[rw_addr+3] <= rw_val[31:24];
            bytes[rw_addr+2] <= rw_val[23:16];
            bytes[rw_addr+1] <= rw_val[15:8];
            bytes[rw_addr] <= rw_val[7:0];
        end
    end
endmodule