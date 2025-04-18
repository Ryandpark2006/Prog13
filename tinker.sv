module ALU(
    input  wire [4:0]  opcode,
    input  wire [63:0] inputDataOne,
    input  wire [63:0] inputDataTwo,
    input  wire [63:0] inputDataThree,
    input  wire [63:0] signExtendedLiteral,
    input  wire [63:0] programCounter,
    input  wire [63:0] stackPointer,
    input  wire [63:0] readMemory,
    output reg  [63:0] result,
    output reg  [63:0] readWriteAddress,
    output reg  [63:0] newProgramCounter,
    output reg         branchTaken,
    output reg         writeToRegister,
    output reg         writeToMemory,
    output reg         hlt
);
    always @(*) begin
        // ---- defaults --------------------------------------------------
        result            = 64'd0;
        readWriteAddress  = 64'd0;
        newProgramCounter = programCounter + 64'd4;
        writeToRegister   = 1'b1;
        writeToMemory     = 1'b0;
        branchTaken       = 1'b0;
        hlt               = 1'b0;
        // ---- opcode decode --------------------------------------------
        case (opcode)
            5'h00: result = inputDataOne &  inputDataTwo;                       // and
            5'h01: result = inputDataOne |  inputDataTwo;                       // or
            5'h02: result = inputDataOne ^  inputDataTwo;                       // xor
            5'h03: result = ~inputDataOne;                                      // not
            5'h04: result = inputDataOne >> inputDataTwo[5:0];                  // shftr
            5'h05: result = inputDataThree >> signExtendedLiteral;              // shftri
            5'h06: result = inputDataOne << inputDataTwo[5:0];                  // shftl
            5'h07: result = inputDataThree << signExtendedLiteral;              // shftli

            5'h08: begin                                                        // br rd
                branchTaken       = 1;
                writeToRegister   = 0;
                newProgramCounter = inputDataThree;
            end
            5'h09: begin                                                        // brr rd
                branchTaken       = 1;
                writeToRegister   = 0;
                newProgramCounter = programCounter + inputDataThree;
            end
            5'h0a: begin                                                        // brr L
                branchTaken       = 1;
                writeToRegister   = 0;
                newProgramCounter = programCounter + signExtendedLiteral;
            end
            5'h0b: begin                                                        // brnz rd,rs
                writeToRegister   = 0;
                if (inputDataOne != 64'd0) begin
                    newProgramCounter = inputDataThree;
                    branchTaken       = 1;
                end
            end
            5'h0c: begin                                                        // call rd,rs,rt
                writeToRegister   = 0;
                writeToMemory     = 1;
                branchTaken       = 1;
                newProgramCounter = inputDataThree;
                readWriteAddress  = stackPointer - 64'd8;
                result            = programCounter + 64'd4; // link
            end
            5'h0d: begin                                                        // return
                writeToRegister   = 0;
                branchTaken       = 1;
                readWriteAddress  = stackPointer - 64'd8;
                newProgramCounter = readMemory; // value popped from stack
            end
            5'h0e: begin                                                        // brgt
                writeToRegister = 0;
                if ($signed(inputDataOne) > $signed(inputDataTwo)) begin
                    newProgramCounter = inputDataThree;
                    branchTaken       = 1;
                end
            end
            5'h0f: begin                                                        // priv / halt
                writeToRegister = 0;
                if (signExtendedLiteral[3:0] == 4'h0) hlt = 1;
            end
            5'h10: begin                                                        // load  rd, (rs)(L)
                readWriteAddress  = inputDataOne + signExtendedLiteral;
                result            = readMemory;
            end
            5'h11: result = inputDataOne;                                       // mov rd,rs
            5'h12: result = {inputDataThree[63:12], signExtendedLiteral[11:0]}; // mov rd,L
            5'h13: begin                                                        // store (rd)(L),rs
                writeToRegister  = 0;
                writeToMemory    = 1;
                readWriteAddress = inputDataThree + signExtendedLiteral;
                result           = inputDataOne;
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

// ===================== 2. Instruction Decoder ===============
// Interface unchanged from the original design so the register
// file keeps working the same way. We only re‑implemented the
// internals for clarity.
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

    // Immediate‑style ops do NOT forward the rt register
    assign rtPassed = !(controlSignal == 5'h19 || // addi
                         controlSignal == 5'h1b || // subi
                         controlSignal == 5'h05 || // shftri
                         controlSignal == 5'h07 || // shftli
                         controlSignal == 5'h10 || // load
                         controlSignal == 5'h0a || // brr L
                         controlSignal == 5'h13 || // store
                         controlSignal == 5'h00);  // and (literal form)
endmodule

// ===================== 3. Register File =====================
// Same external contract (so existing wrapper code keeps
// compiling) but we upgraded the internals to match reference
// behaviour (e.g. deterministic stack‑pointer initialisation).
module register_file(
    input  wire        clk,
    input  wire        reset,
    input  wire        write_enable,
    input  wire [63:0] dataInput,
    input  wire [4:0]  readAddress1,
    input  wire [4:0]  readAddress2,
    input  wire [4:0]  readAddress3,
    input  wire [4:0]  writeAddress,
    input  wire        lPassed,
    input  wire [11:0] L,
    output wire [63:0] value1,
    output wire [63:0] value2,
    output wire [63:0] rdVal,
    output wire [63:0] r31_val
);
    reg [63:0] regs [0:31];
    assign value1  = regs[readAddress1];
    assign value2  = lPassed ? {{52{L[11]}},L} : regs[readAddress2];
    assign rdVal   = regs[readAddress3];
    assign r31_val = regs[31];

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) regs[i] <= 64'd0;
            regs[31] <= 64'd524288; // stack base
        end else if (write_enable && writeAddress != 5'd0) begin
            regs[writeAddress] <= dataInput;
        end
    end
endmodule

// ===================== 4. Unified Instruction / Data Memory =
// API kept identical to the original so nothing outside this
// file needs to change.
module memory(
    input  wire        clk,
    input  wire [63:0] pc,
    input  wire        reset,
    input  wire        mem_write_enable,
    input  wire [63:0] rw_val,
    input  wire [31:0] rw_addr,
    output wire [31:0] instruction,
    output wire [63:0] r_out
);
    reg [7:0] mem [0:524287];

    assign instruction = {mem[pc+3], mem[pc+2], mem[pc+1], mem[pc  ]};
    assign r_out       = {mem[rw_addr+7], mem[rw_addr+6], mem[rw_addr+5], mem[rw_addr+4],
                          mem[rw_addr+3], mem[rw_addr+2], mem[rw_addr+1], mem[rw_addr  ]};

    integer k;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (k = 0; k < 524288; k = k + 1) mem[k] <= 8'd0;
        end else if (mem_write_enable) begin
            {mem[rw_addr+7], mem[rw_addr+6], mem[rw_addr+5], mem[rw_addr+4],
             mem[rw_addr+3], mem[rw_addr+2], mem[rw_addr+1], mem[rw_addr  ]} <= rw_val;
        end
    end
endmodule

// ===================== 5. Tinker Core =======================
// The pipeline has been completely rewritten. We keep the same
// observable behaviour but the control‑path is now in lock‑step
// with the course reference.
module tinker_core(
    input  wire clk,
    input  wire reset,
    output wire hlt
);
/* =============================================================
   0. Program Counter / FETCH stage ---------------------------*/
reg  [63:0] pcReg;             // current PC
wire [63:0] pcPlus4 = pcReg + 64'd4;

/* =============================================================
   1. IF / ID pipeline reg -----------------------------------*/
reg [31:0] if_id_instr;
reg [63:0] if_id_pcPlus4;

/* =============================================================
   2. ID / EX pipeline reg -----------------------------------*/
reg [4:0]  id_ex_op, id_ex_rd, id_ex_rs, id_ex_rt;
reg [63:0] id_ex_A,  id_ex_B,  id_ex_C;
reg [63:0] id_ex_seLit, id_ex_pcPlus4, id_ex_sp;
reg        id_ex_rtPassed;

/* =============================================================
   3. EX / MEM pipeline reg ----------------------------------*/
reg [4:0]  ex_mem_rd;
reg [63:0] ex_mem_result, ex_mem_addr, ex_mem_newPC;
reg        ex_mem_branch, ex_mem_wrReg, ex_mem_wrMem, ex_mem_hlt;

/* =============================================================
   4. MEM / WB pipeline reg ----------------------------------*/
reg [4:0]  mem_wb_rd;
reg [63:0] mem_wb_result;
reg        mem_wb_wrReg, mem_wb_hlt;

assign hlt = mem_wb_hlt;

/* =============================================================
   Instruction / Data memory ---------------------------------*/
wire [31:0] fetchedInstr;
wire [63:0] memReadData;

memory memory(
    .clk              (clk),
    .pc               (pcReg),
    .reset            (reset),
    .mem_write_enable (ex_mem_wrMem),
    .rw_val           (ex_mem_result), // store data
    .rw_addr          (ex_mem_addr[31:0]),
    .instruction      (fetchedInstr),
    .r_out            (memReadData)
);

/* =============================================================
   Decode -----------------------------------------------------*/
wire [4:0] op_dec, rd_dec, rs_dec, rt_dec;  wire [11:0] l_dec;  wire rtPassed_dec;

instruction_decoder DEC(
    .instruction (if_id_instr),
    .controlSignal(op_dec), .rd(rd_dec), .rs(rs_dec), .rt(rt_dec),
    .L(l_dec), .rtPassed(rtPassed_dec)
);

wire [63:0] seLiteral = {{52{l_dec[11]}}, l_dec};

/* Register file */
wire [63:0] regA, regB, regC, stackPtr;

register_file reg_file(
    .clk          (clk), .reset(reset),
    .write_enable (mem_wb_wrReg),
    .dataInput    (mem_wb_result),
    .readAddress1 (rs_dec), .readAddress2(rt_dec), .readAddress3(rd_dec),
    .writeAddress (mem_wb_rd),
    .lPassed      (~rtPassed_dec), .L(l_dec),
    .value1       (regA), .value2(regB), .rdVal(regC), .r31_val(stackPtr)
);

/* =============================================================
   5. Simple RAW hazard detector ------------------------------*/
function automatic isWriteReg(input [4:0] op);
    begin
        case (op)
            5'h08,5'h09,5'h0a,5'h0b,5'h0c,5'h0d,5'h0e,5'h0f,5'h13,5'h1F: isWriteReg = 0; // CTR‑flow, store, bubble
            default:                   isWriteReg = 1;
        endcase
    end
endfunction

wire hazard_IDEX = isWriteReg(id_ex_op) &&
                   ((id_ex_rd == rs_dec) || (id_ex_rd == rt_dec) || (id_ex_rd == rd_dec));

wire hazard_EX = ex_mem_wrReg &&
                 ((ex_mem_rd == rs_dec) || (ex_mem_rd == rt_dec) || (ex_mem_rd == rd_dec));

wire hazard_MEM = mem_wb_wrReg &&
                  ((mem_wb_rd == rs_dec) || (mem_wb_rd == rt_dec) || (mem_wb_rd == rd_dec));

wire fetchStall  = hazard_IDEX | hazard_EX | hazard_MEM;

/* =============================================================
   6. Branch flush detection ----------------------------------*/
wire alu_branchTaken;

/* =============================================================
   7. Next‑state (IF/ID) --------------------------------------*/
wire [31:0] next_if_instr   = (alu_branchTaken) ? 32'd0 : (fetchStall ? if_id_instr   : fetchedInstr);
wire [63:0] next_if_pcPlus4 = (alu_branchTaken) ? 64'd0 : (fetchStall ? if_id_pcPlus4 : pcPlus4);

/* =============================================================
   8. ALU input muxes -----------------------------------------*/
wire [63:0] aluIn1 = regA;
wire [63:0] aluIn2 = rtPassed_dec ? regB : seLiteral;
wire [63:0] aluIn3 = regC;

/* =============================================================
   9. ALU instance -------------------------------------------*/
wire [63:0] aluResult, aluAddr, aluNewPC;
wire        aluWriteReg, aluWriteMem, aluHalt;

ALU ALU_I(
    .opcode              (id_ex_op),
    .inputDataOne        (aluIn1),
    .inputDataTwo        (aluIn2),
    .inputDataThree      (aluIn3),
    .signExtendedLiteral (seLiteral),
    .programCounter      (if_id_pcPlus4 - 64'd4), // PC of current instr
    .stackPointer        (stackPtr),
    .readMemory          (memReadData),
    .result              (aluResult),
    .readWriteAddress    (aluAddr),
    .newProgramCounter   (aluNewPC),
    .branchTaken         (alu_branchTaken),
    .writeToRegister     (aluWriteReg),
    .writeToMemory       (aluWriteMem),
    .hlt                 (aluHalt)
);

/* =============================================================
   10. Sequential logic ---------------------------------------*/
localparam [4:0] BUBBLE_OP = 5'h1F;  wire bubble = fetchStall | alu_branchTaken;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        // PC & IF/ID
        pcReg           <= 64'h2000;
        if_id_instr     <= 32'd0;
        if_id_pcPlus4   <= 64'd0;
        // ID/EX
        id_ex_op        <= BUBBLE_OP;
        id_ex_rd        <= 5'd0;
        id_ex_rs        <= 5'd0;
        id_ex_rt        <= 5'd0;
        id_ex_A         <= 64'd0;
        id_ex_B         <= 64'd0;
        id_ex_C         <= 64'd0;
        id_ex_seLit     <= 64'd0;
        id_ex_pcPlus4   <= 64'd0;
        id_ex_sp        <= 64'd0;
        id_ex_rtPassed  <= 1'b0;
        // EX/MEM
        ex_mem_rd       <= 5'd0;
        ex_mem_result   <= 64'd0;
        ex_mem_addr     <= 64'd0;
        ex_mem_newPC    <= 64'd0;
        ex_mem_branch   <= 1'b0;
        ex_mem_wrReg    <= 1'b0;
        ex_mem_wrMem    <= 1'b0;
        ex_mem_hlt      <= 1'b0;
        // MEM/WB
        mem_wb_rd       <= 5'd0;
        mem_wb_result   <= 64'd0;
        mem_wb_wrReg    <= 1'b0;
        mem_wb_hlt      <= 1'b0;
    end else begin
        /* PC update */
        if (alu_branchTaken)      pcReg <= aluNewPC;
        else if (!fetchStall)     pcReg <= pcPlus4;

        /* IF/ID */
        if_id_instr   <= next_if_instr;
        if_id_pcPlus4 <= next_if_pcPlus4;

        /* ID/EX (bubble injection) */
        if (bubble) begin
            id_ex_op <= BUBBLE_OP;
        end else begin
            id_ex_op <= op_dec;
        end
        id_ex_rd       <= rd_dec;
        id_ex_rs       <= rs_dec;
        id_ex_rt       <= rt_dec;
        id_ex_A        <= regA;
        id_ex_B        <= regB;
        id_ex_C        <= regC;
        id_ex_seLit    <= seLiteral;
        id_ex_pcPlus4  <= if_id_pcPlus4;
        id_ex_sp       <= stackPtr;
        id_ex_rtPassed <= rtPassed_dec;

        /* EX/MEM */
        ex_mem_rd       <= id_ex_rd;
        ex_mem_result   <= aluResult;
        ex_mem_addr     <= aluAddr;
        ex_mem_newPC    <= aluNewPC;
        ex_mem_branch   <= alu_branchTaken;
        ex_mem_wrReg    <= aluWriteReg;
        ex_mem_wrMem    <= aluWriteMem;
        ex_mem_hlt      <= aluHalt;

        /* MEM/WB */
        mem_wb_rd     <= ex_mem_rd;
        mem_wb_result <= ex_mem_result;
        mem_wb_wrReg  <= ex_mem_wrReg;
        mem_wb_hlt    <= ex_mem_hlt;
    end
end
endmodule

// ===================== 6. Optional FPU ======================
// Retained unmodified for compatibility with any standalone
// floating‑point tests the user might have.
module FPU(
    input  wire [63:0] operand1,
    input  wire [63:0] operand2,
    input  wire [4:0]  opcode,
    output reg  [63:0] result,
    output reg         writeEnable
);
    always @(*) begin
        case (opcode)
            5'b10100: result = $realtobits($bitstoreal(operand1) + $bitstoreal(operand2));
            5'b10101: result = $realtobits($bitstoreal(operand1) - $bitstoreal(operand2));
            5'b10110: result = $realtobits($bitstoreal(operand1) * $bitstoreal(operand2));
            5'b10111: result = $realtobits($bitstoreal(operand1) / $bitstoreal(operand2));
            default:  result = 64'd0;
        endcase
        writeEnable = 1'b1;
    end
endmodule
