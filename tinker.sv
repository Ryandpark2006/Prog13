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
// Pipelined Tinker Core (Vanilla 5-Stage) with Branch Flush and WB-stage HALT
// + Branch‐target mux, immediate fix, rdVal wiring, forwarding, flush on mispredict, and delay HALT until after write-back
module tinker_core(
    input  wire        clk,
    input  wire        reset,
    output wire        hlt
);
    // ------------------------------------------------------------------
    // Program Counter & IF/ID registers
    // ------------------------------------------------------------------
    reg [63:0] PC;
    reg [63:0] IF_ID_PC;
    reg [31:0] IF_ID_IR;

    // ------------------------------------------------------------------
    // Decode signals in IF/ID
    // ------------------------------------------------------------------
    wire [4:0]  IF_ctrl     = IF_ID_IR[31:27];
    wire [11:0] IF_L        = IF_ID_IR[11:0];
    // lPassed=1 for opcodes using literal as operand2
    wire        IF_rtPassed = (IF_ctrl==5'b11001 || // addi
                               IF_ctrl==5'b11011 || // subi
                               IF_ctrl==5'b10010 || // mov rd, L
                               IF_ctrl==5'b10000 || // load
                               IF_ctrl==5'b10011 || // store
                               IF_ctrl==5'b01010)   // jump rel2
                              ? 1'b1 : 1'b0;

    // ------------------------------------------------------------------
    // ID/EX pipeline registers
    // ------------------------------------------------------------------
    reg [63:0] ID_EX_PC;
    reg [4:0]  ID_EX_ctrl;
    reg [4:0]  ID_EX_rd, ID_EX_rs, ID_EX_rt;
    reg [11:0] ID_EX_L;
    reg        ID_EX_rtPassed;
    reg [63:0] ID_EX_A, ID_EX_B;
    reg [63:0] ID_EX_r31;
    reg [63:0] ID_EX_rdVal;

    // ------------------------------------------------------------------
    // EX/MEM pipeline registers (+ branch info)
    // ------------------------------------------------------------------
    reg [4:0]  EX_MEM_ctrl;
    reg [4:0]  EX_MEM_rd;
    reg [63:0] EX_MEM_ALU;
    reg [63:0] EX_MEM_B;
    reg        EX_MEM_memWrite;
    reg        EX_MEM_regWrite;
    reg [31:0] EX_MEM_addr;
    reg [63:0] EX_MEM_wrData;
    reg        EX_MEM_changePC;
    reg [63:0] EX_MEM_target;

    // ------------------------------------------------------------------
    // MEM/WB pipeline registers
    // ------------------------------------------------------------------
    reg [4:0]  MEM_WB_ctrl;
    reg [4:0]  MEM_WB_rd;
    reg [63:0] MEM_WB_ALU;
    reg [63:0] MEM_WB_memData;
    reg        MEM_WB_regWrite;
    reg        MEM_WB_memToReg;

    // ------------------------------------------------------------------
    // Memory and Register File
    // ------------------------------------------------------------------
    wire [31:0] inst;
    wire [63:0] mem_rdata;
    memory MEM_INST(
        .pc(PC), .clk(clk), .reset(reset),
        .mem_write_enable(EX_MEM_memWrite),
        .rw_val(EX_MEM_wrData), .rw_addr(EX_MEM_addr),
        .instruction(inst), .r_out(mem_rdata)
    );

    wire [63:0] regOut1, regOut2, rdVal, r31Val;
    register_file RF(
        .clk(clk), .reset(reset),
        .write_enable(MEM_WB_regWrite),
        .dataInput(MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU),
        .readAddress1(IF_ID_IR[21:17]),
        .readAddress2(IF_ID_IR[16:12]),
        .writeAddress(MEM_WB_rd),
        .lPassed(IF_rtPassed), .L(IF_L),
        .value1(regOut1), .value2(regOut2),
        .rdVal(rdVal), .r31_val(r31Val)
    );

    // ------------------------------------------------------------------
    // Forwarding logic
    // ------------------------------------------------------------------
    wire [63:0] aluOp1 = (EX_MEM_regWrite && EX_MEM_rd!=0 && EX_MEM_rd==ID_EX_rs)
                         ? EX_MEM_ALU
                         : (MEM_WB_regWrite && MEM_WB_rd!=0 && MEM_WB_rd==ID_EX_rs)
                           ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU)
                           : ID_EX_A;

    wire [63:0] aluOp2_pre = ID_EX_B;
    wire [63:0] aluOp2 = ID_EX_rtPassed ? aluOp2_pre
                         : (EX_MEM_regWrite && EX_MEM_rd!=0 && EX_MEM_rd==ID_EX_rt)
                           ? EX_MEM_ALU
                           : (MEM_WB_regWrite && MEM_WB_rd!=0 && MEM_WB_rd==ID_EX_rt)
                             ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU)
                             : aluOp2_pre;

    // ------------------------------------------------------------------
    // ALU in EX stage
    // ------------------------------------------------------------------
    wire [63:0] aluResult, aluUpdatedNext;
    wire        aluRegWrite, aluMemWrite, aluChangePC;
    wire [31:0] aluAddr;
    wire [63:0] aluWrData;
    ALU ALU_INST(
        .pc(ID_EX_PC),
        .rdVal(ID_EX_rdVal),
        .operand1(aluOp1),
        .operand2(aluOp2),
        .opcode(ID_EX_ctrl),
        .r_out(mem_rdata),
        .r31_val(ID_EX_r31),
        .result(aluResult),
        .writeEnable(aluRegWrite),
        .mem_write_enable(aluMemWrite),
        .rw_addr(aluAddr),
        .rw_val(aluWrData),
        .updated_next(aluUpdatedNext),
        .changing_pc(aluChangePC)
    );

    // ==================================================================
    // IF stage: update PC and IF/ID with flush
    // ==================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC        <= 64'h2000;
            IF_ID_PC  <= 0;
            IF_ID_IR  <= 0;
        end else begin
            // redirect on branch
            if (EX_MEM_changePC) begin
                PC        <= EX_MEM_target;
                IF_ID_PC  <= 0;
                IF_ID_IR  <= 32'b0;       // squash into NOP
            end else begin
                PC        <= PC + 4;
                IF_ID_PC  <= PC;
                IF_ID_IR  <= inst;
            end
        end
    end

    // ==================================================================
    // ID stage: latch decode + registers with flush
    // ==================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ID_EX_ctrl     <= 0;
            ID_EX_rd       <= 0;
            ID_EX_rs       <= 0;
            ID_EX_rt       <= 0;
            ID_EX_L        <= 0;
            ID_EX_rtPassed <= 0;
            ID_EX_A        <= 0;
            ID_EX_B        <= 0;
            ID_EX_PC       <= 0;
            ID_EX_r31      <= 0;
            ID_EX_rdVal    <= 0;
        end else if (EX_MEM_changePC) begin
            // flush decode
            ID_EX_ctrl     <= 5'b00000;
            ID_EX_rd       <= 0;
            ID_EX_rs       <= 0;
            ID_EX_rt       <= 0;
            ID_EX_L        <= 0;
            ID_EX_rtPassed <= 0;
            ID_EX_A        <= 0;
            ID_EX_B        <= 0;
            ID_EX_PC       <= 0;
            ID_EX_r31      <= 0;
            ID_EX_rdVal    <= 0;
        end else begin
            ID_EX_ctrl     <= IF_ctrl;
            ID_EX_rd       <= IF_ID_IR[26:22];
            ID_EX_rs       <= IF_ID_IR[21:17];
            ID_EX_rt       <= IF_ID_IR[16:12];
            ID_EX_L        <= IF_L;
            ID_EX_rtPassed <= IF_rtPassed;
            ID_EX_A        <= regOut1;
            ID_EX_B        <= regOut2;
            ID_EX_PC       <= IF_ID_PC;
            ID_EX_r31      <= r31Val;
            ID_EX_rdVal    <= rdVal;
        end
    end

    // ==================================================================
    // EX stage: latch ALU results + branch info
    // ==================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            EX_MEM_ctrl     <= 0;
            EX_MEM_rd       <= 0;
            EX_MEM_ALU      <= 0;
            EX_MEM_B        <= 0;
            EX_MEM_memWrite <= 0;
            EX_MEM_regWrite <= 0;
            EX_MEM_addr     <= 0;
            EX_MEM_wrData   <= 0;
            EX_MEM_changePC <= 0;
            EX_MEM_target   <= 0;
        end else begin
            EX_MEM_ctrl     <= ID_EX_ctrl;
            EX_MEM_rd       <= ID_EX_rd;
            EX_MEM_ALU      <= aluResult;
            EX_MEM_B        <= ID_EX_B;
            EX_MEM_memWrite <= aluMemWrite;
            EX_MEM_regWrite <= aluRegWrite;
            EX_MEM_addr     <= aluAddr;
            EX_MEM_wrData   <= aluWrData;
            EX_MEM_changePC <= aluChangePC;
            EX_MEM_target   <= aluUpdatedNext;
        end
    end

    // ==================================================================
    // MEM stage: pass data to WB
    // ==================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            MEM_WB_ctrl     <= 0;
            MEM_WB_rd       <= 0;
            MEM_WB_ALU      <= 0;
            MEM_WB_memData  <= 0;
            MEM_WB_regWrite <= 0;
            MEM_WB_memToReg <= 0;
        end else begin
            MEM_WB_ctrl     <= EX_MEM_ctrl;
            MEM_WB_rd       <= EX_MEM_rd;
            MEM_WB_ALU      <= EX_MEM_ALU;
            MEM_WB_memData  <= mem_rdata;
            MEM_WB_regWrite <= EX_MEM_regWrite;
            MEM_WB_memToReg <= (EX_MEM_ctrl == 5'b10000);
        end
    end

    // ==================================================================
    // WB stage HALT detection
    // ==================================================================
    assign hlt = (MEM_WB_ctrl == 5'h0f);
endmodule
