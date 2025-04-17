module instruction_decoder(
    input wire[31:0] instruction,
    output wire[4:0] controlSignal,
    output wire[4:0] rd,
    output wire[4:0] rs,
    output wire[4:0] rt,
    output wire[11:0] L,
    output wire rtPassed
    // output wire hlt
    // output wire isFloatingPoint
);

// control signal 
assign controlSignal = instruction[31:27];    // big endian so this is fine 
assign rd = instruction[26:22];
assign rs = instruction[21:17];
assign rt = instruction[16:12];
assign L = instruction[11:0];

// assign hlt = (instruction[31:27] == 5'b01111 && instruction[11:0] == 11'b00000000000) ? 1'b1 : 1'b0;


// need to have an if statement or multiplexor on whether rt or L need to be passed 
// assign rtPassed = 0x19 0x1b 0x5 0x7 0x12
assign rtPassed = (controlSignal == 5'b11001 || controlSignal == 5'b11011 || 
    controlSignal == 5'b00101 || controlSignal == 5'b00111 || controlSignal == 5'b10010 || controlSignal == 5'b01010 || controlSignal == 5'b10011 || controlSignal == 5'b10000) ? 0 : 1;
// $display("%32h", controlSignal);  
endmodule

