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


module memory(
    input        clk,
    input  [63:0] programCounter,
    input  [63:0] dataAddress,
    input        writeEnable,
    input  [63:0] writeData,
    output [31:0] readInstruction,
    output [63:0] readData
);
    reg [7:0] bytes [0:524287];   // 512 KiB for both instructions & data

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



/* ================================================================== *
 *  Tinker‑Core  – 5‑stage, single‑issue, full operand forwarding     *
 * ================================================================== */
module tinker_core (
    input  wire clk,
    input  wire reset,
    output wire hlt
);

/* ───── 0. PC & FETCH ───────────────────────────────────────────── */
reg  [63:0] PC               = 64'h2000;           // start address
wire [63:0] PC_plus4         = PC + 64'd4;

/* ───── IF/ID pipeline latches ─────────────────────────────────── */
reg  [31:0] IF_ID_IR         = 32'd0;
reg  [63:0] IF_ID_PC4        = 64'd0;

/* ───── ID/EX pipeline latches ─────────────────────────────────── */
reg  [4:0]  ID_EX_op         = 5'h1F;
reg  [4:0]  ID_EX_rd         = 5'd0,
            ID_EX_rs         = 5'd0,
            ID_EX_rt         = 5'd0;
reg  [63:0] ID_EX_D1         = 64'd0,   // operand A (rs)
            ID_EX_D2         = 64'd0,   // operand B (rt)
            ID_EX_D3         = 64'd0;   // operand C (rd)
reg  [63:0] ID_EX_SExt       = 64'd0,
            ID_EX_PC4        = 64'd0,
            ID_EX_SP         = 64'd0;

/* ───── EX/MEM pipeline latches ────────────────────────────────── */
reg  [4:0]  EX_MEM_rd        = 5'd0;
reg  [63:0] EX_MEM_ALU       = 64'd0,
            EX_MEM_addr      = 64'd0,
            EX_MEM_target    = 64'd0;
reg         EX_MEM_writeReg  = 1'b0,
            EX_MEM_writeMem  = 1'b0,
            EX_MEM_brTaken   = 1'b0,
            EX_MEM_hlt       = 1'b0,
            EX_MEM_isLoad    = 1'b0;   // remembers opcode 0x10

/* ───── MEM/WB pipeline latches ────────────────────────────────── */
reg  [4:0]  MEM_WB_rd        = 5'd0;
reg  [63:0] MEM_WB_val       = 64'd0;
reg         MEM_WB_writeReg  = 1'b0,
            MEM_WB_hlt       = 1'b0;   // <— drives top‑level hlt

assign hlt = MEM_WB_hlt;

/* ───── unified memory (dual‑ported internally) ────────────────── */
wire [31:0] fetchedInstr;
wire [63:0] dmem_rdata;
memory memory (
    .clk            (clk),
    .programCounter (PC),
    .dataAddress    (EX_MEM_addr),
    .writeEnable    (EX_MEM_writeMem),
    .writeData      (EX_MEM_ALU),
    .readInstruction(fetchedInstr),
    .readData       (dmem_rdata)
);

/* ───── Decode & Register File read ────────────────────────────── */
wire [4:0] op, rd, rs, rt;  wire [11:0] L;
decoder DEC (.instruction(IF_ID_IR), .opcode(op),
             .rd(rd), .rs(rs), .rt(rt), .l(L));

wire [63:0] SExt = {{52{L[11]}}, L};

wire [63:0] regA, regB, regC, regSP;
register_file reg_file (
    .clk         (clk),
    .reset       (reset),
    .write       (MEM_WB_writeReg),
    .data_input  (MEM_WB_val),
    .registerOne (rs),
    .registerTwo (rt),
    .registerThree(rd),
    .writeAddress(MEM_WB_rd),
    .data_outputOne  (regA),
    .data_outputTwo  (regB),
    .data_outputThree(regC),
    .stackPointer    (regSP)
);

/* ───── Hazard detection (only load‑use) ───────────────────────── */
wire ID_EX_isLoad  = (ID_EX_op == 5'h10);          // load opcode
wire load_use_hazard =
        ID_EX_isLoad && (ID_EX_rd != 0) &&
       ((ID_EX_rd == rs) || (ID_EX_rd == rt) || (ID_EX_rd == rd));

/* ───── Flush flag (branch taken) ───────────────────────────────── */
wire flushIF = EX_MEM_brTaken;

/* ───── IF‑stage bubbles & next values ─────────────────────────── */
wire bubbleIF = flushIF | load_use_hazard;
wire [31:0] IF_nextIR  = bubbleIF ? 32'd0 : fetchedInstr;
wire [63:0] IF_nextPC4 = bubbleIF ? 64'd0 : PC_plus4;

/* ───── ID‑stage bubbles & next values ─────────────────────────── */
wire bubbleID = bubbleIF;
wire [4:0]  ID_nextOp   = bubbleID ? 5'h1F : op;
wire [4:0]  ID_nextRd   = bubbleID ? 5'd0  : rd;
wire [4:0]  ID_nextRs   = bubbleID ? 5'd0  : rs;
wire [4:0]  ID_nextRt   = bubbleID ? 5'd0  : rt;
wire [63:0] ID_nextD1   = bubbleID ? 64'd0 : regA;
wire [63:0] ID_nextD2   = bubbleID ? 64'd0 : regB;
wire [63:0] ID_nextD3   = bubbleID ? 64'd0 : regC;
wire [63:0] ID_nextSExt = bubbleID ? 64'd0 : SExt;
wire [63:0] ID_nextSP   = bubbleID ? 64'd0 : regSP;
wire [63:0] ID_nextPC4  = bubbleID ? 64'd0 : IF_ID_PC4;

/* ───── Forwarding (EX/MEM & MEM/WB) ───────────────────────────── */
wire fwdA_EX  = EX_MEM_writeReg && (EX_MEM_rd!=0) && (EX_MEM_rd == ID_EX_rs);
wire fwdA_MEM = MEM_WB_writeReg && (MEM_WB_rd!=0) && (MEM_WB_rd == ID_EX_rs);

wire fwdB_EX  = EX_MEM_writeReg && (EX_MEM_rd!=0) && (EX_MEM_rd == ID_EX_rt);
wire fwdB_MEM = MEM_WB_writeReg && (MEM_WB_rd!=0) && (MEM_WB_rd == ID_EX_rt);

wire fwdC_EX  = EX_MEM_writeReg && (EX_MEM_rd!=0) && (EX_MEM_rd == ID_EX_rd);
wire fwdC_MEM = MEM_WB_writeReg && (MEM_WB_rd!=0) && (MEM_WB_rd == ID_EX_rd);

wire [63:0] ALU_D1 = fwdA_EX  ? EX_MEM_ALU :
                     fwdA_MEM ? MEM_WB_val : ID_EX_D1;

wire [63:0] ALU_D2 = fwdB_EX  ? EX_MEM_ALU :
                     fwdB_MEM ? MEM_WB_val : ID_EX_D2;

wire [63:0] ALU_D3 = fwdC_EX  ? EX_MEM_ALU :
                     fwdC_MEM ? MEM_WB_val : ID_EX_D3;

/* ───── ALU ────────────────────────────────────────────────────── */
wire [63:0] aluY, aluAddr, aluPC;
wire aluTaken, aluRegW, aluMemW, aluHlt;
alu ALU (
    .opcode              (ID_EX_op),
    .inputDataOne        (ALU_D1),
    .inputDataTwo        (ALU_D2),
    .inputDataThree      (ALU_D3),
    .signExtendedLiteral (ID_EX_SExt),
    .programCounter      (ID_EX_PC4 - 64'd4),
    .stackPointer        (ID_EX_SP),
    .readMemory          (dmem_rdata),
    .result              (aluY),
    .readWriteAddress    (aluAddr),
    .newProgramCounter   (aluPC),
    .branchTaken         (aluTaken),
    .writeToRegister     (aluRegW),
    .writeToMemory       (aluMemW),
    .hlt                 (aluHlt)
);

/* ───── Sequential logic for all latches + PC ─────────────────── */
always @(posedge clk or posedge reset) begin
    if (reset) begin
        /* all regs already initialised to 0/starting addr above */
        PC <= 64'h2000;
    end else begin
        /* 0) PC update */
        if (aluTaken) PC <= aluPC;
        else          PC <= PC_plus4;

        /* 1) IF/ID */
        IF_ID_IR  <= IF_nextIR;
        IF_ID_PC4 <= IF_nextPC4;

        /* 2) ID/EX */
        ID_EX_op   <= ID_nextOp;  ID_EX_rd <= ID_nextRd;
        ID_EX_rs   <= ID_nextRs;  ID_EX_rt <= ID_nextRt;
        ID_EX_D1   <= ID_nextD1;  ID_EX_D2 <= ID_nextD2; ID_EX_D3 <= ID_nextD3;
        ID_EX_SExt <= ID_nextSExt;ID_EX_SP <= ID_nextSP; ID_EX_PC4<=ID_nextPC4;

        /* 3) EX/MEM */
        EX_MEM_rd        <= ID_EX_rd;
        EX_MEM_ALU       <= aluY;
        EX_MEM_addr      <= aluAddr;
        EX_MEM_target    <= aluPC;
        EX_MEM_writeReg  <= aluRegW;
        EX_MEM_writeMem  <= aluMemW;
        EX_MEM_brTaken   <= aluTaken;
        EX_MEM_hlt       <= aluHlt;
        EX_MEM_isLoad    <= (ID_EX_op == 5'h10);

        /* 4) MEM/WB */
        MEM_WB_rd       <= EX_MEM_rd;
        MEM_WB_val      <= EX_MEM_isLoad ? dmem_rdata : EX_MEM_ALU;
        MEM_WB_writeReg <= EX_MEM_writeReg;
        MEM_WB_hlt      <= EX_MEM_hlt;
    end
end

endmodule
