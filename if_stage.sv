module if_stage(
    input  wire         clk,
    input  wire         reset,
    input  wire [63:0]  pc_in,       // current PC from pipeline or branch logic
    output reg  [63:0]  pc_out,      // next PC to pass along
    output reg  [31:0]  instruction, // or more if you fetch multiple instructions
    // ...
);
// Typically you’d do:
// 1) pc_out <= pc_in + 4 (for a single 32-bit instruction)
// 2) instruction <= i_cache[pc_in] or memory module output
// Pipeline register (IF/ID) would store instruction, pc_out, etc.


endmodule