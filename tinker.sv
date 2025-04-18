module alu(
    input  [4:0]  opcode,
    input  [63:0] inputDataOne,
    input  [63:0] inputDataTwo,
    input  [63:0] inputDataThree,
    input  [63:0] signExtendedLiteral,
    input  [63:0] programCounter,
    input  [63:0] stackPointer,
    input  [63:0] readMemory,
    output reg [63:0] result,
    output reg [63:0] readWriteAddress,
    output reg [63:0] newProgramCounter,
    output reg       branchTaken,
    output reg       writeToRegister,
    output reg       writeToMemory,
    output reg       hlt
);
    always @(*) begin
        result            = 64'd0;
        readWriteAddress  = 64'd0;
        newProgramCounter = programCounter + 64'd4;
        writeToRegister   = 1'b1;
        writeToMemory     = 1'b0;
        branchTaken       = 1'b0;
        hlt               = 1'b0;
        case (opcode)
            5'h00: result = inputDataOne & inputDataTwo;                       // and
            5'h01: result = inputDataOne | inputDataTwo;                       // or
            5'h02: result = inputDataOne ^ inputDataTwo;                       // xor
            5'h03: result = ~inputDataOne;                                      // not
            5'h04: result = inputDataOne >> inputDataTwo[5:0];                 // shftr
            5'h05: result = inputDataThree >> signExtendedLiteral;             // shftri
            5'h06: result = inputDataOne << inputDataTwo[5:0];                 // shftl
            5'h07: result = inputDataThree << signExtendedLiteral;             // shftli
            5'h08: begin                                                       // br
                branchTaken     = 1;
                writeToRegister = 0;
                newProgramCounter = inputDataThree;
            end
            5'h09: begin                                                       // brr rd
                branchTaken     = 1;
                writeToRegister = 0;
                newProgramCounter = programCounter + inputDataThree;
            end
            5'h0a: begin                                                       // brr L
                branchTaken     = 1;
                writeToRegister = 0;
                newProgramCounter = programCounter + signExtendedLiteral;
            end
            5'h0b: begin                                                       // brnz
                writeToRegister = 0;
                if (inputDataOne != 0) begin
                    newProgramCounter = inputDataThree;
                    branchTaken       = 1;
                end
            end
            5'h0c: begin                                                       // call
                writeToRegister = 0;
                writeToMemory   = 1;
                branchTaken     = 1;
                newProgramCounter = inputDataThree;
                readWriteAddress  = stackPointer - 64'd8;
                result            = programCounter + 64'd4;
            end
            5'h0d: begin                                                       // return
                writeToRegister = 0;
                branchTaken     = 1;
                readWriteAddress  = stackPointer - 64'd8;
                newProgramCounter = readMemory;
            end
            5'h0e: begin                                                       // brgt
                writeToRegister = 0;
                if ($signed(inputDataOne) > $signed(inputDataTwo)) begin
                    newProgramCounter = inputDataThree;
                    branchTaken       = 1;
                end
            end
            5'h0f: begin                                                       // halt/priv
                writeToRegister = 0;
                if (signExtendedLiteral[3:0] == 4'h0) hlt = 1;
            end
            5'h10: begin                                                       // load
                readWriteAddress = inputDataOne + signExtendedLiteral;
                result           = readMemory;
            end
            5'h11: result = inputDataOne;                                      // mov_reg_to_reg
            5'h12: result = {inputDataThree[63:12], signExtendedLiteral[11:0]}; // mov_L_to_reg (upper)
            5'h13: begin                                                       // store
                writeToRegister = 0;
                writeToMemory   = 1;
                readWriteAddress = inputDataThree + signExtendedLiteral;
                result          = inputDataOne;
            end
            5'h14: result = $realtobits($bitstoreal(inputDataOne) + $bitstoreal(inputDataTwo)); // addf
            5'h15: result = $realtobits($bitstoreal(inputDataOne) - $bitstoreal(inputDataTwo)); // subf
            5'h16: result = $realtobits($bitstoreal(inputDataOne) * $bitstoreal(inputDataTwo)); // mulf
            5'h17: result = $realtobits($bitstoreal(inputDataOne) / $bitstoreal(inputDataTwo)); // divf
            5'h18: result = inputDataOne + inputDataTwo;                        // add
            5'h19: result = inputDataThree + signExtendedLiteral;               // addi
            5'h1a: result = inputDataOne - inputDataTwo;                        // sub
            5'h1b: result = inputDataThree - signExtendedLiteral;               // subi
            5'h1c: result = inputDataOne * inputDataTwo;                        // mul
            5'h1d: result = (inputDataTwo == 0) ? 64'd0 : (inputDataOne / inputDataTwo); // div
            default: begin
                writeToRegister = 0;
                writeToMemory   = 0;
                branchTaken     = 0;
            end
        endcase
    end
endmodule


module registers(
    input        clk,
    input        reset,
    input        write,
    input  [63:0] data_input,
    input  [4:0]  registerOne,
    input  [4:0]  registerTwo,
    input  [4:0]  registerThree,
    input  [4:0]  writeAddress,
    output [63:0] data_outputOne,
    output [63:0] data_outputTwo,
    output [63:0] data_outputThree,
    output [63:0] stackPointer
);
    reg [63:0] registers [0:31];

    assign data_outputOne   = registers[registerOne];
    assign data_outputTwo   = registers[registerTwo];
    assign data_outputThree = registers[registerThree];
    assign stackPointer     = registers[31];

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) registers[i] <= 64'd0;
            registers[31] <= 64'd524288;
        end else if (write) begin
            registers[writeAddress] <= data_input;
        end
    end
endmodule


module memory(
    input        clk,
    input  [63:0] programCounter,
    input  [63:0] dataAddress,
    input        writeEnable,
    input  [63:0] writeData,
    output [31:0] readInstruction,
    output [63:0] readData
);
    reg [7:0] bytes [0:524287];   // 512 KiB for both instructions & data

    assign readInstruction = {bytes[programCounter+3], bytes[programCounter+2],
                              bytes[programCounter+1], bytes[programCounter]};

    assign readData = {bytes[dataAddress+7], bytes[dataAddress+6],
                       bytes[dataAddress+5], bytes[dataAddress+4],
                       bytes[dataAddress+3], bytes[dataAddress+2],
                       bytes[dataAddress+1], bytes[dataAddress]};

    always @(posedge clk) begin
        if (writeEnable) begin
            {bytes[dataAddress+7], bytes[dataAddress+6], bytes[dataAddress+5],
             bytes[dataAddress+4], bytes[dataAddress+3], bytes[dataAddress+2],
             bytes[dataAddress+1], bytes[dataAddress]} <= writeData;
        end
    end
endmodule


module decoder(
    input  [31:0] instruction,
    output [4:0]  opcode,
    output [4:0]  rd,
    output [4:0]  rs,
    output [4:0]  rt,
    output [11:0] l
);
    assign opcode = instruction[31:27];
    assign rd     = instruction[26:22];
    assign rs     = instruction[21:17];
    assign rt     = instruction[16:12];
    assign l      = instruction[11:0];
endmodule


/* ================================================================ *
 *  TINKER‑CORE  — refactored to “PC / IF_ID / ID_EX / EX_MEM” style *
 * ================================================================ */
module tinker_core (
    input  wire clk,
    input  wire reset,
    output wire hlt
);

/* ───────────── 0. FETCH ───────────────────────────────────────── */
reg  [63:0] PC;                      // program counter
wire [63:0] PC_plus4 = PC + 64'd4;   // next sequential address

/* ───────────── RAW‑hazard stall & branch flush flags ──────────── */
wire stall_IF  ;   // set by hazard detector
wire flush_IF  ;   // asserted as soon as the EX‑stage ALU redirects the PC

/* ================================================================ *
 *  1. IF → ID  (pipeline register)                                 *
 * ================================================================ */
reg  [31:0] IF_ID_IR;      // fetched instruction
reg  [63:0] IF_ID_PC4;     // PC+4 accompanying that instruction

/* ================================================================ *
 *  2. ID → EX  (pipeline register)                                 *
 * ================================================================ */
reg  [4:0]  ID_EX_op  , ID_EX_rd , ID_EX_rs , ID_EX_rt;
reg  [63:0] ID_EX_A   , ID_EX_B  , ID_EX_C;
reg  [63:0] ID_EX_SExt, ID_EX_PC4, ID_EX_SP;

/* ================================================================ *
 *  3. EX → MEM (pipeline register)                                 *
 * ================================================================ */
reg  [4:0]  EX_MEM_rd;
reg  [63:0] EX_MEM_ALU   , EX_MEM_addr , EX_MEM_PCtarget;
reg         EX_MEM_brTkn , EX_MEM_regW , EX_MEM_memW , EX_MEM_hlt;

/* ================================================================ *
 *  4. MEM → WB (pipeline register)                                 *
 * ================================================================ */
reg  [4:0]  MEM_WB_rd;
reg  [63:0] MEM_WB_val;
reg         MEM_WB_regW , MEM_WB_hlt;

/* ================================================================ *
 *  5. Combinational modules shared with the original design        *
 * ================================================================ */
wire [31:0] instr_mem_out;
wire [63:0] data_mem_out;

memory MEM (
    .clk            (clk),
    .programCounter (PC),
    .dataAddress    (EX_MEM_addr),
    .writeEnable    (EX_MEM_memW),
    .writeData      (EX_MEM_ALU),
    .readInstruction(instr_mem_out),
    .readData       (data_mem_out)
);

wire [4:0] op, rd, rs, rt;  wire [11:0] L;
decoder DEC (.instruction(IF_ID_IR), .opcode(op), .rd(rd), .rs(rs), .rt(rt), .l(L));

wire [63:0] A_reg, B_reg, C_reg, SP_reg;
registers REGS (
    .clk(clk), .reset(reset),
    .write(MEM_WB_regW),
    .data_input(MEM_WB_val),
    .registerOne(rs), .registerTwo(rt), .registerThree(rd),
    .writeAddress(MEM_WB_rd),
    .data_outputOne(A_reg), .data_outputTwo(B_reg),
    .data_outputThree(C_reg), .stackPointer(SP_reg)
);

wire [63:0] SExt = {{52{L[11]}}, L};

/* ───────────── Simple hazard detector (unchanged logic) ───────── */
wire idex_write = (ID_EX_op != 5'h1F) && (ID_EX_op != 5'h08) && (ID_EX_op != 5'h09) &&
                  (ID_EX_op != 5'h0a) && (ID_EX_op != 5'h0b) && (ID_EX_op != 5'h0c) &&
                  (ID_EX_op != 5'h0d) && (ID_EX_op != 5'h0e) && (ID_EX_op != 5'h0f) &&
                  (ID_EX_op != 5'h13);

wire RAW_IDEX = idex_write     && ((ID_EX_rd == rs)  || (ID_EX_rd == rt) || (ID_EX_rd == rd));
wire RAW_EX   = EX_MEM_regW    && ((EX_MEM_rd == rs) || (EX_MEM_rd == rt) || (EX_MEM_rd == rd));
wire RAW_MEM  = MEM_WB_regW    && ((MEM_WB_rd == rs) || (MEM_WB_rd == rt) || (MEM_WB_rd == rd));

assign stall_IF = RAW_IDEX | RAW_EX | RAW_MEM;

/* ───────────── Bubble / flush muxes for IF→ID stage ───────────── */
localparam [4:0] BUBBLE = 5'h1F;
wire bubble = stall_IF | flush_IF;

wire [31:0] IF_ID_IR_next = bubble ? 32'd0  : instr_mem_out;
wire [63:0] IF_ID_PC4_next= bubble ? 64'd0  : PC_plus4;

/* ───────────── ALU (identical to original) ────────────────────── */
wire [63:0] alu_Y , alu_addr , alu_PCout;
wire        alu_taken , alu_regW , alu_memW , alu_H;

alu ALU (
    .opcode(ID_EX_op),
    .inputDataOne(ID_EX_A), .inputDataTwo(ID_EX_B),
    .inputDataThree(ID_EX_C),
    .signExtendedLiteral(ID_EX_SExt),
    .programCounter(ID_EX_PC4 - 64'd4),
    .stackPointer(ID_EX_SP),
    .readMemory(data_mem_out),
    .result(alu_Y), .readWriteAddress(alu_addr),
    .newProgramCounter(alu_PCout),
    .branchTaken(alu_taken),
    .writeToRegister(alu_regW),
    .writeToMemory(alu_memW),
    .hlt(alu_H)
);

assign flush_IF = alu_taken;   // flush as soon as branch resolved

/* ================================================================ *
 *  6.  SEQ LOGIC — clocked registers                               *
 * ================================================================ */
always @(posedge clk or posedge reset) begin
    /* ---------- RESET ---------- */
    if (reset) begin
        PC           <= 64'h2000;

        IF_ID_IR  <= 0; IF_ID_PC4 <= 0;

        ID_EX_op  <= BUBBLE; ID_EX_rd <= 0; ID_EX_rs <= 0; ID_EX_rt <= 0;
        ID_EX_A   <= 0; ID_EX_B <= 0; ID_EX_C <= 0;
        ID_EX_SExt<= 0; ID_EX_PC4<= 0; ID_EX_SP <= 0;

        EX_MEM_rd <= 0; EX_MEM_ALU <= 0; EX_MEM_addr <= 0; EX_MEM_PCtarget <= 0;
        EX_MEM_brTkn <= 0; EX_MEM_regW <= 0; EX_MEM_memW <= 0; EX_MEM_hlt <= 0;

        MEM_WB_rd <= 0; MEM_WB_val <= 0; MEM_WB_regW <= 0; MEM_WB_hlt <= 0;

    end else begin
        /* ---------- 0. PC update ---------- */
        if (alu_taken)        PC <= alu_PCout;
        else if (!stall_IF)   PC <= PC_plus4;

        /* ---------- 1. IF→ID ---------- */
        IF_ID_IR  <= IF_ID_IR_next;
        IF_ID_PC4 <= IF_ID_PC4_next;

        /* ---------- 2. ID→EX ---------- */
        ID_EX_op   <= bubble ? BUBBLE : op;
        ID_EX_rd   <= bubble ? 5'd0 : rd;
        ID_EX_rs   <= bubble ? 5'd0 : rs;
        ID_EX_rt   <= bubble ? 5'd0 : rt;
        ID_EX_A    <= bubble ? 0 : A_reg;
        ID_EX_B    <= bubble ? 0 : B_reg;
        ID_EX_C    <= bubble ? 0 : C_reg;
        ID_EX_SExt <= bubble ? 0 : SExt;
        ID_EX_PC4  <= bubble ? 0 : IF_ID_PC4;
        ID_EX_SP   <= bubble ? 0 : SP_reg;

        /* ---------- 3. EX→MEM ---------- */
        EX_MEM_rd      <= ID_EX_rd;
        EX_MEM_ALU     <= alu_Y;
        EX_MEM_addr    <= alu_addr;
        EX_MEM_PCtarget<= alu_PCout;
        EX_MEM_brTkn   <= alu_taken;
        EX_MEM_regW    <= alu_regW;
        EX_MEM_memW    <= alu_memW;
        EX_MEM_hlt     <= alu_H;

        /* ---------- 4. MEM→WB ---------- */
        MEM_WB_rd    <= EX_MEM_rd;
        MEM_WB_val   <= EX_MEM_ALU;
        MEM_WB_regW  <= EX_MEM_regW;
        MEM_WB_hlt   <= EX_MEM_hlt;
    end
end

/* ================================================================ *
 *  7. HALT flag (unchanged)                                        *
 * ================================================================ */
assign hlt = MEM_WB_hlt;

endmodule
