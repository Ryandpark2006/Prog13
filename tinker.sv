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

module tinker_core(
    input  wire       clk,
    input  wire       reset,
    output wire       hlt
);
    // Program Counter
    reg [63:0] pc;

    // IF/ID pipeline registers
    reg [63:0] IF_ID_PC;
    reg [31:0] IF_ID_IR;

    // ID/EX pipeline registers
    reg [63:0] ID_EX_PC;
    reg [63:0] ID_EX_A;
    reg [63:0] ID_EX_B;
    reg [63:0] ID_EX_rdVal;
    reg [4:0]  ID_EX_ctrl;
    reg [4:0]  ID_EX_rd;
    reg        ID_EX_rtPassed;

    // EX/MEM pipeline registers
    reg [63:0] EX_MEM_ALUResult;
    reg [63:0] EX_MEM_nextPC;
    reg [4:0]  EX_MEM_ctrl;
    reg [4:0]  EX_MEM_rd;
    reg        EX_MEM_aluWE;
    reg        EX_MEM_memWE;
    reg        EX_MEM_chgPC;
    reg [31:0] EX_MEM_rwAddr;
    reg [63:0] EX_MEM_rwVal;

    // MEM/WB pipeline registers
    reg [63:0] MEM_WB_memData;
    reg [63:0] MEM_WB_ALUResult;
    reg [4:0]  MEM_WB_ctrl;
    reg [4:0]  MEM_WB_rd;
    reg        MEM_WB_aluWE;
    reg        MEM_WB_memToReg;

    // Wires for stage interconnects
    wire [31:0] instr;
    wire [63:0] mem_rdata;
    wire        alu_we, alu_mem_we, alu_chg_pc;
    wire [63:0] alu_result, fpu_result, updated_next_pc;
    wire [31:0] alu_rwAddr;
    wire [63:0] alu_rwVal;
    wire [4:0]  id_ctrl, id_rd, id_rs, id_rt;
    wire [11:0] id_L;
    wire        id_rtPassed;
    wire [63:0] reg_val1, reg_val2, rdVal, r31_val;

    // Fetch stage: instruction memory
    memory memory(
        .pc(pc),
        .clk(clk),
        .reset(reset),
        .mem_write_enable(EX_MEM_memWE),
        .rw_val(EX_MEM_rwVal),
        .rw_addr(EX_MEM_rwAddr),
        .instruction(instr),
        .r_out(mem_rdata)
    );

    // Decode stage
    instruction_decoder dec(
        .instruction(IF_ID_IR),
        .controlSignal(id_ctrl),
        .rd(id_rd),
        .rs(id_rs),
        .rt(id_rt),
        .L(id_L),
        .rtPassed(id_rtPassed)
    );
    register_file reg_file(
        .clk(clk),
        .reset(reset),
        .write_enable(MEM_WB_aluWE),
        .dataInput(MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALUResult),
        .readAddress1(id_rs),
        .readAddress2(id_rt),
        .writeAddress(MEM_WB_rd),
        .lPassed(~id_rtPassed),
        .L(id_L),
        .value1(reg_val1),
        .value2(reg_val2),
        .rdVal(rdVal),
        .r31_val(r31_val)
    );

    // Execute stage
    ALU alu(
        .pc(ID_EX_PC),
        .rdVal(ID_EX_rdVal),
        .operand1(ID_EX_A),
        .operand2(ID_EX_B),
        .opcode(ID_EX_ctrl),
        .r_out(mem_rdata),
        .r31_val(r31_val),
        .result(alu_result),
        .writeEnable(alu_we),
        .mem_write_enable(alu_mem_we),
        .rw_addr(alu_rwAddr),
        .rw_val(alu_rwVal),
        .updated_next(updated_next_pc),
        .changing_pc(alu_chg_pc)
    );
    FPU fpu(
        .operand1(ID_EX_A),
        .operand2(ID_EX_B),
        .opcode(ID_EX_ctrl),
        .result(fpu_result),
        .writeEnable()  // unused
    );

    // Detect halt at decode stage
    wire [63:0] signExt = {{52{id_L[11]}}, id_L};
    assign hlt = (id_ctrl==5'h0f) && (signExt[3:0]==4'h0);

    // Pipeline register updates
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize PC and flush pipelines
            pc            <= 64'h2000;
            IF_ID_IR      <= 32'd0;
            ID_EX_ctrl    <= 5'd0;
            EX_MEM_ctrl   <= 5'd0;
            MEM_WB_ctrl   <= 5'd0;
        end else begin
            // Writeback → RegFile occurs automatically via regfile.write_enable

            // MEM → WB
            MEM_WB_memData   <= mem_rdata;
            MEM_WB_ALUResult <= EX_MEM_ALUResult;
            MEM_WB_ctrl      <= EX_MEM_ctrl;
            MEM_WB_rd        <= EX_MEM_rd;
            MEM_WB_aluWE     <= EX_MEM_aluWE;
            MEM_WB_memToReg  <= (EX_MEM_ctrl == 5'b10000);

            // EX → MEM
            EX_MEM_ALUResult <= (ID_EX_ctrl[4:2] == 3'b101) ? fpu_result : alu_result;
            EX_MEM_nextPC    <= updated_next_pc;
            EX_MEM_ctrl      <= ID_EX_ctrl;
            EX_MEM_rd        <= ID_EX_rd;
            EX_MEM_aluWE     <= alu_we;
            EX_MEM_memWE     <= alu_mem_we;
            EX_MEM_chgPC     <= alu_chg_pc;
            EX_MEM_rwAddr    <= alu_rwAddr;
            EX_MEM_rwVal     <= alu_rwVal;

            // ID → EX
            ID_EX_PC     <= IF_ID_PC;
            ID_EX_A      <= reg_val1;
            ID_EX_B      <= reg_val2;
            ID_EX_rdVal  <= rdVal;
            ID_EX_ctrl   <= id_ctrl;
            ID_EX_rd     <= id_rd;
            ID_EX_rtPassed <= id_rtPassed;

            // IF → ID (flush on branch)
            if (EX_MEM_chgPC) begin
                IF_ID_IR  <= 32'd0;
                IF_ID_PC  <= 64'd0;
            end else begin
                IF_ID_IR  <= instr;
                IF_ID_PC  <= pc;
            end

            // PC update: branch resolution in EX/MEM
            if (EX_MEM_chgPC)
                pc <= EX_MEM_nextPC;
            else
                pc <= pc + 64'd4;
        end
    end
endmodule

