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

module tinker_core (
    input  wire clk,
    input  wire reset,
    output wire hlt
);

/* ================================================================
   0. Program Counter & FETCH ------------------------------------*/
reg  [63:0] programCounter;
wire [63:0] pcPlus4 = programCounter + 64'd4;

/* --------------------------------------------------  Flush / Stall */
wire fetchStall;            // RAW‑hazard detector output
/*  ►► FIX ◄◄  flush as soon as ALU resolves a branch */
wire flushFetch = aluBranchTaken;   // use ALU’s combinational flag

/* ================================================================
   1. IF / ID  ----------------------------------------------------*/
reg  [31:0] fetchToDecodeInstr;
reg  [63:0] fetchToDecodePCPlus4;

/* ================================================================
   2. ID / EX  ----------------------------------------------------*/
reg  [4:0]  decoderToAluOpcode, decoderToAluRd, decoderToAluRs, decoderToAluRt;
reg  [63:0] decoderToAluData1,  decoderToAluData2,  decoderToAluData3;
reg  [63:0] decoderToAluSignExtLit, decoderToAluPCPlus4, decoderToAluStackPtr;

/* ================================================================
   3. EX / MEM  ---------------------------------------------------*/
reg  [4:0]  aluToMemRd;
reg  [63:0] aluToMemResult, aluToMemAddr, aluToMemNewPC;
reg         aluToMemBranchTaken, aluToMemWriteToReg, aluToMemWriteToMem, aluToMemHlt;

/* ================================================================
   4. MEM / WB  ---------------------------------------------------*/
reg  [4:0]  memToWritebackRd;
reg  [63:0] memToWritebackResult;
reg         memToWritebackWriteReg, memToWritebackHlt;

assign hlt = memToWritebackHlt;

/* ================================================================
   5. Instruction / Data Memory ----------------------------------*/
wire [31:0] fetchedInstr;
wire [63:0] memoryReadData;

memory memory (
    .clk            (clk),
    .programCounter (programCounter),
    .dataAddress    (aluAddr),
    .writeEnable    (aluWriteMem),
    .writeData      (aluResult),
    .readInstruction(fetchedInstr),
    .readData       (memoryReadData)
);

/* ================================================================
   6. Decode stage wiring ----------------------------------------*/
wire [4:0] opcode, rd, rs, rt;  wire [11:0] l;

decoder dec_inst (.instruction(fetchToDecodeInstr),
                  .opcode(opcode), .rd(rd), .rs(rs), .rt(rt), .l(l));

wire [63:0] signExtLiteral = {{52{l[11]}}, l};

/* ---------- Register file -------------------------------------*/
wire [63:0] regData1, regData2, regData3, stackPtr;

registers reg_file (
    .clk(clk), .reset(reset),
    .write(memToWritebackWriteReg),
    .data_input(memToWritebackResult),
    .registerOne(rs), .registerTwo(rt), .registerThree(rd),
    .writeAddress(memToWritebackRd),
    .data_outputOne(regData1), .data_outputTwo(regData2),
    .data_outputThree(regData3), .stackPointer(stackPtr));

/* ================================================================
   7. RAW‑hazard detector (Decode) -------------------------------*/
/* Does the ID/EX instruction write a register? */
wire idExWriteReg = (decoderToAluOpcode != 5'h1F) &&           // not a bubble
                    (decoderToAluOpcode != 5'h08) &&           // br
                    (decoderToAluOpcode != 5'h09) &&           // brr rd
                    (decoderToAluOpcode != 5'h0a) &&           // brr L
                    (decoderToAluOpcode != 5'h0b) &&           // brnz
                    (decoderToAluOpcode != 5'h0c) &&           // call
                    (decoderToAluOpcode != 5'h0d) &&           // return
                    (decoderToAluOpcode != 5'h0e) &&           // brgt
                    (decoderToAluOpcode != 5'h0f) &&           // halt/priv
                    (decoderToAluOpcode != 5'h13);             // store

wire hazardIDEX = idExWriteReg &&
                  ((decoderToAluRd == rs) || (decoderToAluRd == rt) || (decoderToAluRd == rd));

wire hazardEX  = aluToMemWriteToReg &&
                 ((aluToMemRd == rs) || (aluToMemRd == rt) || (aluToMemRd == rd));

wire hazardMEM = memToWritebackWriteReg &&
                 ((memToWritebackRd == rs) || (memToWritebackRd == rt) || (memToWritebackRd == rd));

assign fetchStall = hazardIDEX | hazardEX | hazardMEM;

/* ================================================================
   8. IF/ID & ID/EX next‑state calc ------------------------------*/
wire [31:0] nextFetchInstr   = flushFetch ? 32'd0 :
                               fetchStall ? fetchToDecodeInstr : fetchedInstr;
wire [63:0] nextFetchPCPlus4 = flushFetch ? 64'd0 :
                               fetchStall ? fetchToDecodePCPlus4 : pcPlus4;

localparam [4:0] BUBBLE_OP = 5'h1F;  wire bubble = flushFetch|fetchStall;

wire [4:0]  nextOpcode     = bubble ? BUBBLE_OP : opcode;
wire [4:0]  nextRd         = bubble ? 5'd0      : rd;
wire [4:0]  nextRs         = bubble ? 5'd0      : rs;
wire [4:0]  nextRt         = bubble ? 5'd0      : rt;

wire [63:0] nextData1      = bubble ? 64'd0 : regData1;
wire [63:0] nextData2      = bubble ? 64'd0 : regData2;
wire [63:0] nextData3      = bubble ? 64'd0 : regData3;
wire [63:0] nextSignExtLit = bubble ? 64'd0 : signExtLiteral;
wire [63:0] nextPCPlus4ID  = bubble ? 64'd0 : fetchToDecodePCPlus4;
wire [63:0] nextStackPtr   = bubble ? 64'd0 : stackPtr;

/* ================================================================
   9. Execute stage (ALU) ----------------------------------------*/
wire [63:0] aluResult, aluAddr, aluNewPC;
wire        aluBranchTaken, aluWriteReg, aluWriteMem, aluHalt;

alu alu_inst (
    .opcode(decoderToAluOpcode),
    .inputDataOne(decoderToAluData1), .inputDataTwo(decoderToAluData2),
    .inputDataThree(decoderToAluData3),
    .signExtendedLiteral(decoderToAluSignExtLit),
    .programCounter(decoderToAluPCPlus4 - 64'd4),
    .stackPointer(decoderToAluStackPtr),
    .readMemory(memoryReadData),
    .result(aluResult), .readWriteAddress(aluAddr),
    .newProgramCounter(aluNewPC),
    .branchTaken(aluBranchTaken),
    .writeToRegister(aluWriteReg),
    .writeToMemory(aluWriteMem),
    .hlt(aluHalt));

/* EX/MEM next values -------------------------------------------*/
wire [4:0]  nextAluToMemRd          = decoderToAluRd;
wire [63:0] nextAluToMemResult      = aluResult;
wire [63:0] nextAluToMemAddr        = aluAddr;
wire [63:0] nextAluToMemNewPC       = aluNewPC;
wire        nextAluToMemBranchTaken = aluBranchTaken;
wire        nextAluToMemWriteReg    = aluWriteReg;
wire        nextAluToMemWriteMem    = aluWriteMem;
wire        nextAluToMemHlt         = aluHalt;

/* ================================================================
   10. MEM stage forwarding --------------------------------------*/
wire [4:0]  nextMemToWB_Rd        = aluToMemRd;
wire [63:0] nextMemToWB_Result    = aluToMemResult;
wire        nextMemToWB_WriteReg  = aluToMemWriteToReg;
wire        nextMemToWB_Hlt       = aluToMemHlt;

/* ================================================================
   11. Sequential logic ------------------------------------------*/
always @(posedge clk or posedge reset) begin
    if (reset) begin
        programCounter <= 64'h2000;
        fetchToDecodeInstr <= 32'd0; fetchToDecodePCPlus4 <= 64'd0;

        decoderToAluOpcode <= BUBBLE_OP; decoderToAluRd <= 0;
        decoderToAluRs <= 0; decoderToAluRt <= 0;
        decoderToAluData1 <= 0; decoderToAluData2 <= 0; decoderToAluData3 <= 0;
        decoderToAluSignExtLit <= 0; decoderToAluPCPlus4 <= 0; decoderToAluStackPtr <= 0;

        aluToMemRd <= 0; aluToMemResult <= 0; aluToMemAddr <= 0; aluToMemNewPC <= 0;
        aluToMemBranchTaken <= 0; aluToMemWriteToReg <= 0; aluToMemWriteToMem <= 0; aluToMemHlt <= 0;

        memToWritebackRd <= 0; memToWritebackResult <= 0;
        memToWritebackWriteReg <= 0; memToWritebackHlt <= 0;
    end else begin
        /* PC update */
        if (flushFetch)          programCounter <= aluNewPC;
        else if (!fetchStall)    programCounter <= pcPlus4;

        /* IF/ID */
        fetchToDecodeInstr   <= nextFetchInstr;
        fetchToDecodePCPlus4 <= nextFetchPCPlus4;

        /* ID/EX */
        decoderToAluOpcode     <= nextOpcode;
        decoderToAluRd         <= nextRd;
        decoderToAluRs         <= nextRs;
        decoderToAluRt         <= nextRt;
        decoderToAluData1      <= nextData1;
        decoderToAluData2      <= nextData2;
        decoderToAluData3      <= nextData3;
        decoderToAluSignExtLit <= nextSignExtLit;
        decoderToAluPCPlus4    <= nextPCPlus4ID;
        decoderToAluStackPtr   <= nextStackPtr;

        /* EX/MEM */
        aluToMemRd              <= nextAluToMemRd;
        aluToMemResult          <= nextAluToMemResult;
        aluToMemAddr            <= nextAluToMemAddr;
        aluToMemNewPC           <= nextAluToMemNewPC;
        aluToMemBranchTaken     <= nextAluToMemBranchTaken;
        aluToMemWriteToReg      <= nextAluToMemWriteReg;
        aluToMemWriteToMem      <= nextAluToMemWriteMem;
        aluToMemHlt             <= nextAluToMemHlt;

        /* MEM/WB */
        memToWritebackRd        <= nextMemToWB_Rd;
        memToWritebackResult    <= nextMemToWB_Result;
        memToWritebackWriteReg  <= nextMemToWB_WriteReg;
        memToWritebackHlt       <= nextMemToWB_Hlt;
    end
end

endmodule