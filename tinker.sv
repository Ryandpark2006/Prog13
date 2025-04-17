// Fully Edited Tinker Core and Associated Modules (FSM Style)
// Includes fixes for CALL and MOV_L_TO_REG

//------------------------------------------------------
// ALU Module
//------------------------------------------------------
module ALU(
    input wire [63:0] pc,
    input wire [63:0] rdVal,
    input wire [63:0] operand1,
    input wire [63:0] operand2,
    input wire [4:0] opcode,
    input wire [63:0] r_out,
    input wire [63:0] r31_val,
    output reg [63:0] result,
    output reg writeEnable,
    output reg mem_write_enable,
    output reg [31:0] rw_addr,
    output reg [63:0] rw_val,
    output reg [63:0] updated_next,
    output reg changing_pc
);
    always @(*) begin
        writeEnable      = 1'b1;
        mem_write_enable = 1'b0;
        changing_pc      = 1'b0;
        updated_next     = pc + 4;
        case (opcode)
            // Arithmetic
            5'b11000: result = operand1 + operand2;  // add
            5'b11001: result = operand1 + operand2;  // addi
            5'b11010: result = operand1 - operand2;  // sub
            5'b11011: result = operand1 - operand2;  // subi
            5'b11100: result = operand1 * operand2;  // mul
            5'b11101: result = operand1 / operand2;  // div

            // Logic
            5'b00000: result = operand1 & operand2;  // and
            5'b00001: result = operand1 | operand2;  // or
            5'b00010: result = operand1 ^ operand2;  // xor
            5'b00011: result = ~operand1;            // not
            5'b00100: result = operand1 >> operand2;
            5'b00101: result = operand1 >> operand2;
            5'b00110: result = operand1 << operand2;
            5'b00111: result = operand1 << operand2;

            // Data movement
            5'b10001: result = operand1;             // mov rd, rs
            5'b10010: result = operand2;             // mov rd, L (literal)
            5'b10000: begin                          // load
                rw_addr = operand1 + operand2;
                result  = r_out;
            end
            5'b10011: begin                          // store
                writeEnable      = 1'b0;
                mem_write_enable = 1'b1;
                rw_addr          = rdVal + operand2;
                rw_val           = operand1;
            end

            // Control
            5'b01000: begin // jump
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = rdVal;
            end
            5'b01001: begin // jump rel
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = pc + rdVal;
            end
            5'b01010: begin // jump rel2
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = pc + operand2;
            end
            5'b01011: begin // brnz
                writeEnable = 1'b0;
                if (operand1 != 64'd0) begin
                    changing_pc = 1'b1;
                    updated_next = rdVal;
                end
            end
            5'b01101: begin // return
                writeEnable      = 1'b0;
                changing_pc      = 1'b1;
                mem_write_enable = 1'b1;
                rw_addr          = r31_val - 8;
                updated_next     = r_out;
            end
            5'b01100: begin // call
                writeEnable      = 1'b0;
                changing_pc      = 1'b1;
                mem_write_enable = 1'b1;
                rw_val           = pc + 4;
                rw_addr          = r31_val - 8;
                updated_next     = rdVal;
            end
            5'b01110: begin // brgt
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = ($signed(operand1) > $signed(operand2)) ? rdVal : pc + 4;
            end

            // Floating
            5'b10100: result = $realtobits($bitstoreal(operand1) + $bitstoreal(operand2));
            5'b10101: result = $realtobits($bitstoreal(operand1) - $bitstoreal(operand2));
            5'b10110: result = $realtobits($bitstoreal(operand1) * $bitstoreal(operand2));
            5'b10111: result = $realtobits($bitstoreal(operand1) / $bitstoreal(operand2));
            default: begin
                writeEnable      = 1'b0;
                mem_write_enable = 1'b0;
            end
        endcase
    end
endmodule

//------------------------------------------------------
// FPU Module
//------------------------------------------------------
module FPU(
    input wire [63:0] operand1,
    input wire [63:0] operand2,
    input wire [4:0] opcode,
    output reg [63:0] result,
    output reg writeEnable
);
    always @(*) begin
        case (opcode)
            5'b10100: result = $realtobits($bitstoreal(operand1) + $bitstoreal(operand2));
            5'b10101: result = $realtobits($bitstoreal(operand1) - $bitstoreal(operand2));
            5'b10110: result = $realtobits($bitstoreal(operand1) * $bitstoreal(operand2));
            5'b10111: result = $realtobits($bitstoreal(operand1) / $bitstoreal(operand2));
            default: result = 64'b0;
        endcase
        writeEnable = 1;
    end
endmodule

//------------------------------------------------------
// Instruction Decoder
//------------------------------------------------------
module instruction_decoder(
    input  wire [31:0] instruction,
    output wire [4:0]  controlSignal,
    output wire [4:0]  rd,
    output wire [4:0]  rs,
    output wire [4:0]  rt,
    output wire [11:0] L,
    output wire        rtPassed
);
    assign controlSignal = instruction[31:27];
    assign rd            = instruction[26:22];
    assign rs            = instruction[21:17];
    assign rt            = instruction[16:12];
    assign L             = instruction[11:0];
    assign rtPassed      = (controlSignal == 5'b11001 || controlSignal == 5'b11011 ||
                            controlSignal == 5'b00101 || controlSignal == 5'b00111 ||
                            controlSignal == 5'b10010 || controlSignal == 5'b01010 ||
                            controlSignal == 5'b10011 || controlSignal == 5'b10000)
                           ? 0 : 1;
endmodule

//------------------------------------------------------
// Memory
//------------------------------------------------------
module memory(
    input  wire [63:0] pc,
    input  wire       clk,
    input  wire       reset,
    input  wire       mem_write_enable,
    input  wire [63:0] rw_val,
    input  wire [31:0] rw_addr,
    output wire [31:0] instruction,
    output wire [63:0] r_out
);
    reg [7:0] bytes [524287:0];
    assign instruction = {bytes[pc+3], bytes[pc+2], bytes[pc+1], bytes[pc]};
    assign r_out       = {bytes[rw_addr+7], bytes[rw_addr+6], bytes[rw_addr+5], bytes[rw_addr+4],
                          bytes[rw_addr+3], bytes[rw_addr+2], bytes[rw_addr+1], bytes[rw_addr]};
    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 524288; i = i + 1)
                bytes[i] <= 8'b0;
        end else if (mem_write_enable) begin
            bytes[rw_addr+7] <= rw_val[63:56];
            bytes[rw_addr+6] <= rw_val[55:48];
            bytes[rw_addr+5] <= rw_val[47:40];
            bytes[rw_addr+4] <= rw_val[39:32];
            bytes[rw_addr+3] <= rw_val[31:24];
            bytes[rw_addr+2] <= rw_val[23:16];
            bytes[rw_addr+1] <= rw_val[15:8];
            bytes[rw_addr  ] <= rw_val[7:0];
        end
    end
endmodule

//------------------------------------------------------
// Register File
//------------------------------------------------------
module register_file(
    input  wire        clk,
    input  wire        reset,
    input  wire        write_enable,
    input  wire [63:0] dataInput,
    input  wire [4:0]  readAddress1,
    input  wire [4:0]  readAddress2,
    input  wire [4:0]  writeAddress,
    input  wire        lPassed,
    input  wire [11:0] L,
    output wire [63:0] value1,
    output wire [63:0] value2,
    output wire [63:0] rdVal,
    output wire [63:0] r31_val
);
    reg [63:0] registers [31:0];
    assign value1 = registers[readAddress1];
    assign value2 = lPassed ? {{52{L[11]}},L} : registers[readAddress2];
    assign rdVal  = registers[writeAddress];
    assign r31_val= registers[31];
    integer j;
    always @(posedge clk) begin
        if (reset) begin
            for (j = 0; j < 31; j = j + 1)
                registers[j] <= 64'b0;
            registers[31] <= 64'h80000;
        end else if (write_enable) begin
            registers[writeAddress] <= dataInput;
        end
    end
endmodule


// Vanilla 5‑Stage Pipeline Tinker Core
// IF, ID, EX, MEM, WB stages in parallel with pipeline registers

//------------------------------------------------------
// ALU, FPU, Decoder, Memory, Register File (unchanged)
// ... include your existing ALU, FPU, instruction_decoder,
// memory, register_file modules here ...
//------------------------------------------------------
//------------------------------------------------------
// tinker.sv: Top-level 5‑stage vanilla pipeline
//------------------------------------------------------
module tinker(
    input  wire        clk,
    input  wire        reset,
    output wire        hlt
);
    // --- IF/ID pipeline regs ---
    reg [63:0] IF_ID_PC;
    reg [31:0] IF_ID_IR;

    // --- ID/EX pipeline regs ---
    reg [63:0] ID_EX_PC;
    reg [63:0] ID_EX_A;
    reg [63:0] ID_EX_B;
    reg [63:0] ID_EX_rdVal;
    reg [4:0]  ID_EX_ctrl;
    reg [4:0]  ID_EX_rd;
    wire [4:0] id_ctrl, id_rd, id_rs, id_rt;
    wire [11:0] id_L;
    wire       id_rtPassed;
    wire [63:0] rf_val1, rf_val2, rf_rdVal, rf_r31;

    // --- EX/MEM pipeline regs ---
    reg [63:0] EX_MEM_aluResult;
    reg [4:0]  EX_MEM_ctrl;
    reg [4:0]  EX_MEM_rd;
    reg        EX_MEM_aluWE;
    reg        EX_MEM_memToReg;
    reg [63:0] EX_MEM_memData;

    // --- MEM/WB pipeline regs ---
    reg [63:0] MEM_WB_val;
    reg [4:0]  MEM_WB_ctrl;
    reg [4:0]  MEM_WB_rd;
    reg        MEM_WB_we;

    // Program counter
    wire [63:0] next_pc;
    wire        ex_chgPC;

    // --- Instantiate IF stage (fetch instruction & PC update) ---
    // Memory module also lives in IF stage to fetch instruction
    memory memory(
        .pc               (IF_ID_PC),
        .clk              (clk),
        .reset            (reset),
        .mem_write_enable (EX_MEM_memToReg && !EX_MEM_ctrl==5'b10011), // store from EX stage
        .rw_val           (/* from EX_MEM */),
        .rw_addr          (/* from EX_MEM */),
        .instruction      (IF_ID_IR),
        .r_out            (EX_MEM_memData)   // routed through pipeline regs
    );
    fetch_control fc(
        .clk(clk), .reset(reset),
        .changing_pc(ex_chgPC), .next_pc(next_pc), .pc(IF_ID_PC)
    );

    // --- Instantiate ID stage ---
    instruction_decoder idec(
        .instruction(IF_ID_IR),
        .controlSignal(id_ctrl),
        .rd(id_rd), .rs(id_rs), .rt(id_rt),
        .L(id_L), .rtPassed(id_rtPassed)
    );
    register_file reg_file(
        .clk(clk), .reset(reset), .write_enable(MEM_WB_we),
        .dataInput    (MEM_WB_val),
        .readAddress1 (id_rs), .readAddress2(id_rt),
        .writeAddress (MEM_WB_rd),
        .lPassed      (~id_rtPassed),
        .L            (id_L),
        .value1       (rf_val1), .value2(rf_val2),
        .rdVal        (rf_rdVal), .r31_val(rf_r31)
    );

    // --- Instantiate EX stage ---
    ALU alu(
        .pc(ID_EX_PC), .rdVal(ID_EX_rdVal),
        .operand1(ID_EX_A), .operand2(ID_EX_B),
        .opcode(ID_EX_ctrl), .r_out(ID_EX_A /* dummy */), .r31_val(rf_r31),
        .result(EX_MEM_aluResult), .writeEnable(EX_MEM_aluWE),
        .mem_write_enable(/*unused*/), .rw_addr(/*unused*/), .rw_val(/*unused*/),
        .updated_next(next_pc), .changing_pc(ex_chgPC)
    );

    // --- WB stage (simple writeback only) ---
    assign MEM_WB_val  = (MEM_WB_ctrl == 5'b10000) ? EX_MEM_memData : EX_MEM_aluResult;
    assign MEM_WB_we   = EX_MEM_aluWE;
    assign hlt         = (id_ctrl == 5'h0f) && (id_L[3:0] == 4'h0);

    // Pipeline register updates
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IF_ID_PC <= 64'h2000;
            IF_ID_IR <= 32'b0;
            ID_EX_CTRL <= 5'b0; ID_EX_rd <= 5'b0;
            EX_MEM_ctrl <= 5'b0; EX_MEM_rd <= 5'b0; MEM_WB_ctrl <= 5'b0;
        end else begin
            // IF -> ID
            IF_ID_PC <= next_pc;
            IF_ID_IR <= memory_inst.instruction;
            // ID -> EX
            ID_EX_PC    <= IF_ID_PC;
            ID_EX_ctrl  <= id_ctrl;
            ID_EX_rd    <= id_rd;
            ID_EX_A     <= rf_val1;
            ID_EX_B     <= rf_val2;
            ID_EX_rdVal <= rf_rdVal;
            // EX -> MEM
            EX_MEM_ctrl      <= ID_EX_ctrl;
            EX_MEM_rd        <= ID_EX_rd;
            EX_MEM_aluWE     <= alu.writeEnable;
            EX_MEM_memToReg  <= (ID_EX_ctrl == 5'b10000);
            // MEM -> WB
            MEM_WB_ctrl <= EX_MEM_ctrl;
            MEM_WB_rd   <= EX_MEM_rd;
        end
    end
endmodule
