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


module register_file(
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


module memory (
    input        clk,
    input  [63:0] pc,           // 8‑byte aligned
    input  [63:0] dataAddress,
    input        writeEnable,
    input  [63:0] writeData,
    output [31:0] instr0,
    output [31:0] instr1,
    output [63:0] readData
);
    reg [7:0] bytes [0:524287];  // 512 KiB I+D

    /* dual instruction read */
    assign instr0 = {bytes[pc+3], bytes[pc+2], bytes[pc+1], bytes[pc  ]};
    assign instr1 = {bytes[pc+7], bytes[pc+6], bytes[pc+5], bytes[pc+4]};

    /* data read */
    assign readData = {bytes[dataAddress+7], bytes[dataAddress+6],
                       bytes[dataAddress+5], bytes[dataAddress+4],
                       bytes[dataAddress+3], bytes[dataAddress+2],
                       bytes[dataAddress+1], bytes[dataAddress]};

    /* store */
    always @(posedge clk) begin
        if (writeEnable)
            {bytes[dataAddress+7], bytes[dataAddress+6], bytes[dataAddress+5],
             bytes[dataAddress+4], bytes[dataAddress+3], bytes[dataAddress+2],
             bytes[dataAddress+1], bytes[dataAddress]} <= writeData;
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

/* ======================================================================= *
 *  Tinker‑Core  ▸  2‑wide in‑order issue                                  *
 *  full EX/MEM/WB forwarding  +  WB‑stage HALT                            *
 * ======================================================================= */
module tinker_core (
    input  wire clk,
    input  wire reset,
    output wire hlt
);

/* ─────  0. Program‑Counter & Dual‑Fetch  ───────────────────────────── */
reg  [63:0] PC;                       // 8‑byte aligned
wire [63:0] PC_plus8 = PC + 64'd8;

/* unified I‑/D‑memory (dual‑word fetch) */
wire [31:0] IF_instr0 , IF_instr1 ;
wire [63:0] dmem_rdata;
memory64 MEM (
    .clk(clk),
    .pc(PC),
    .dataAddress(EX_MEM_addr),
    .writeEnable(EX_MEM_memW),
    .writeData(EX_MEM_ALU),
    .instr0(IF_instr0), .instr1(IF_instr1),
    .readData(dmem_rdata)
);

/* ─────  1. IF → ID  ────────────────────────────────────────────────── */
reg  [31:0] IF_ID_IR [1:0];
reg  [63:0] IF_ID_PC;

wire flush = EX_MEM_brTkn;            // branch resolved in MEM
wire stall;                           // RAW hazard (slot‑0)

/* decode slot‑0 / slot‑1 */
wire [4:0] op0, rd0, rs0, rt0;  wire [11:0] L0;
wire [4:0] op1, rd1, rs1, rt1;  wire [11:0] L1;
decoder D0(.instruction(IF_ID_IR[0]), .opcode(op0), .rd(rd0),
           .rs(rs0), .rt(rt0), .l(L0));
decoder D1(.instruction(IF_ID_IR[1]), .opcode(op1), .rd(rd1),
           .rs(rs1), .rt(rt1), .l(L1));

wire dep10 = (rd0!=5'd0)&&((rd0==rs1)||(rd0==rt1)||(rd0==rd1));

wire bubble0 = flush | stall;
wire bubble1 = flush | stall | dep10;

wire [31:0] IF_IR0_n = bubble0 ? 32'd0 : IF_instr0;
wire [31:0] IF_IR1_n = bubble1 ? 32'd0 : IF_instr1;
wire [63:0] IF_PC_n  = flush ? EX_MEM_PCtarget :
                       stall ? IF_ID_PC      : PC_plus8;

/* ─────  2. Register File  ──────────────────────────────────────────── */
wire [63:0] regA, regB, regC, regSP;
register_file RF (
    .clk(clk), .reset(reset),
    .write     (MEM_WB_regW),
    .data_input(MEM_WB_val),
    .registerOne(rs0), .registerTwo(rt0), .registerThree(rd0),
    .writeAddress(MEM_WB_rd),
    .data_outputOne(regA), .data_outputTwo(regB),
    .data_outputThree(regC), .stackPointer(regSP)
);
wire [63:0] SE0 = {{52{L0[11]}},L0};
wire [63:0] SE1 = {{52{L1[11]}},L1};

/* ─────  3. ID → EX  ───────────────────────────────────────────────── */
reg [4:0]  ID_EX_op  [1:0], ID_EX_rd [1:0], ID_EX_rs[1:0], ID_EX_rt[1:0];
reg [63:0] ID_EX_A   [1:0], ID_EX_B [1:0], ID_EX_C [1:0];
reg [63:0] ID_EX_SE  [1:0], ID_EX_PC;

wire [4:0] rd0_nxt = bubble0 ? 5'd0 : rd0;

/* RAW against EX/MEM/WB (slot‑0) */
wire RAW_EX0  = EX_MEM_regW && (EX_MEM_rd!=0) &&
                ((EX_MEM_rd==rs0)||(EX_MEM_rd==rt0)||(EX_MEM_rd==rd0));
wire RAW_MEM0 = MEM_WB_regW && (MEM_WB_rd!=0) &&
                ((MEM_WB_rd==rs0)||(MEM_WB_rd==rt0)||(MEM_WB_rd==rd0));
assign stall  = RAW_EX0 | RAW_MEM0;

/* latch */
integer s;
always @(posedge clk or posedge reset) begin
    if(reset) begin
        PC<=64'h2000; IF_ID_IR[0]<=0; IF_ID_IR[1]<=0; IF_ID_PC<=0;
        for(s=0;s<2;s=s+1) begin ID_EX_op[s]<=5'h1F; ID_EX_rd[s]<=0; end
    end else begin
        /* PC */
        if(flush) PC<=EX_MEM_PCtarget;
        else if(!stall) PC<=PC_plus8;

        /* IF→ID */
        IF_ID_IR[0]<=IF_IR0_n; IF_ID_IR[1]<=IF_IR1_n; IF_ID_PC<=IF_PC_n;

        /* slot‑0 ID→EX */
        ID_EX_op[0]<=bubble0?5'h1F:op0; ID_EX_rd[0]<=rd0_nxt;
        ID_EX_rs[0]<=rs0; ID_EX_rt[0]<=rt0;
        ID_EX_A [0]<=regA; ID_EX_B[0]<=regB; ID_EX_C[0]<=regC; ID_EX_SE[0]<=SE0;

        /* slot‑1 ID→EX */
        ID_EX_op[1]<=bubble1?5'h1F:op1; ID_EX_rd[1]<=bubble1?5'd0:rd1;
        ID_EX_rs[1]<=rs1; ID_EX_rt[1]<=rt1;
        ID_EX_A [1]<=regA; ID_EX_B[1]<=regB; ID_EX_C[1]<=regC; ID_EX_SE[1]<=SE1;

        ID_EX_PC<=IF_ID_PC;
    end
end

/* ─────  4. Forwarding (WB → EX) for slot‑0  ───────────────────────── */
wire fA_WB0 = MEM_WB_regW && (MEM_WB_rd!=0) && (MEM_WB_rd==ID_EX_rs[0]);
wire fB_WB0 = MEM_WB_regW && (MEM_WB_rd!=0) && (MEM_WB_rd==ID_EX_rt[0]);
wire [63:0] ALU_A0 = fA_WB0 ? MEM_WB_val : ID_EX_A[0];
wire [63:0] ALU_B0 = fB_WB0 ? MEM_WB_val : ID_EX_B[0];

/* ─────  5. Execution units ────────────────────────────────────────── */
wire [63:0] aluY0, aluAddr0, aluPC0;  wire aluTaken0, aluRegW0, aluMemW0, aluHlt0;
alu ALU0 (
    .opcode(ID_EX_op[0]), .inputDataOne(ALU_A0), .inputDataTwo(ALU_B0),
    .inputDataThree(ID_EX_C[0]), .signExtendedLiteral(ID_EX_SE[0]),
    .programCounter(ID_EX_PC-64'd8), .stackPointer(regSP), .readMemory(dmem_rdata),
    .result(aluY0), .readWriteAddress(aluAddr0), .newProgramCounter(aluPC0),
    .branchTaken(aluTaken0), .writeToRegister(aluRegW0),
    .writeToMemory(aluMemW0), .hlt(aluHlt0)
);

wire [63:0] aluY1, aluAddr1, aluPC1;  wire aluTaken1, aluRegW1, aluMemW1, aluHlt1;
alu ALU1 (
    .opcode(ID_EX_op[1]), .inputDataOne(ID_EX_A[1]), .inputDataTwo(ID_EX_B[1]),
    .inputDataThree(ID_EX_C[1]), .signExtendedLiteral(ID_EX_SE[1]),
    .programCounter(ID_EX_PC-64'd4), .stackPointer(regSP), .readMemory(dmem_rdata),
    .result(aluY1), .readWriteAddress(aluAddr1), .newProgramCounter(aluPC1),
    .branchTaken(aluTaken1), .writeToRegister(aluRegW1),
    .writeToMemory(aluMemW1), .hlt(aluHlt1)
);

/* ─────  6. Branch target (slot‑0 wins) ─────────────────────────────── */
wire        EX_MEM_brTkn    = aluTaken0 | aluTaken1;
wire [63:0] EX_MEM_PCtarget = aluTaken0 ? aluPC0 : aluPC1;

/* ─────  7. EX → MEM  (latch + new hlt flag)  ───────────────────────── */
reg [4:0]  EX_MEM_rd;
reg [63:0] EX_MEM_ALU, EX_MEM_addr;
reg        EX_MEM_regW, EX_MEM_memW, EX_MEM_memToReg;
reg        EX_MEM_hlt;                       // ★ new pipe‑stage flag
always @(posedge clk or posedge reset) begin
    if(reset) begin
        EX_MEM_rd<=0; EX_MEM_ALU<=0; EX_MEM_addr<=0;
        EX_MEM_regW<=0; EX_MEM_memW<=0; EX_MEM_memToReg<=0; EX_MEM_hlt<=0;
    end else begin
        EX_MEM_rd       <= rd0_nxt;
        EX_MEM_ALU      <= aluY0;
        EX_MEM_addr     <= aluAddr0;
        EX_MEM_regW     <= aluRegW0;
        EX_MEM_memW     <= aluMemW0;
        EX_MEM_memToReg <= (ID_EX_op[0]==5'h10);   // load?
        EX_MEM_hlt      <= aluHlt0 | aluHlt1;      // propagate HALT
    end
end

/* ─────  8. MEM → WB  (commit)  ─────────────────────────────────────── */
reg [4:0]  MEM_WB_rd;  reg [63:0] MEM_WB_val;
reg        MEM_WB_regW, MEM_WB_hlt;
always @(posedge clk or posedge reset) begin
    if(reset) begin
        MEM_WB_rd<=0; MEM_WB_val<=0; MEM_WB_regW<=0; MEM_WB_hlt<=0;
    end else begin
        MEM_WB_rd   <= EX_MEM_rd;
        MEM_WB_val  <= EX_MEM_memToReg ? dmem_rdata : EX_MEM_ALU;
        MEM_WB_regW <= EX_MEM_regW;
        MEM_WB_hlt  <= EX_MEM_hlt;            // ★ HALT only when instruction commits
    end
end

assign hlt = MEM_WB_hlt;

/* ─────  end module  ─────────────────────────────────────────────── */
endmodule
