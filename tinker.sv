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
            // 5'b10010: result = operand2;             // mov rd, L (literal)
            5'b10010: begin                           // mov rd, L: set bits 52-63
                result = (rdVal & 64'h000FFFFFFFFFFFFF) |
                        (operand2[11:0] << 52);
            end
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
/*
  Vanilla 5-stage pipeline implementation of the Tinker processor.
  Stages: IF -> ID -> EX -> MEM -> WB
  No hazard/forwarding logic (assumes no data/control hazards).
*/


/*
  Vanilla 5-stage pipeline implementation of the Tinker processor.
  Stages: IF -> ID -> EX -> MEM -> WB
  No hazard/forwarding logic (assumes no data/control hazards).
*/

/*
  Vanilla 5-stage pipeline implementation of the Tinker processor.
  Stages: IF -> ID -> EX -> MEM -> WB
  No hazard/forwarding logic (assumes no data/control hazards).
*/

module tinker_core(
    input  wire        clk,
    input  wire        reset,
    output wire        hlt
);

    // Program Counter
    reg [63:0] PC;

    // -------------------
    // IF/ID Pipeline Register
    // -------------------
    reg [31:0] IF_ID_IR;
    reg [63:0] IF_ID_PC;

    // -------------------
    // ID/EX Pipeline Register
    // -------------------
    reg [4:0]  ID_EX_control, ID_EX_rs, ID_EX_rt, ID_EX_rd;
    reg [11:0] ID_EX_L;
    reg        ID_EX_rtPassed;
    reg [63:0] ID_EX_reg_val1, ID_EX_reg_val2;
    reg [63:0] ID_EX_PC;
    reg        ID_EX_memToReg;    // new: select mem data vs ALU for WB

    // -------------------
    // EX/MEM Pipeline Register
    // -------------------
    reg [4:0]  EX_MEM_rd;
    reg [63:0] EX_MEM_alu_result, EX_MEM_reg_val2;
    reg        EX_MEM_mem_write_enable, EX_MEM_writeEnable, EX_MEM_changing_pc;
    reg [31:0] EX_MEM_rw_addr;
    reg [63:0] EX_MEM_rw_val, EX_MEM_updated_next;
    reg        EX_MEM_memToReg;   // new

    // -------------------
    // MEM/WB Pipeline Register
    // -------------------
    reg [4:0]  MEM_WB_rd;
    reg [63:0] MEM_WB_alu_result, MEM_WB_mem_data;
    reg        MEM_WB_writeEnable;
    reg        MEM_WB_memToReg;   // new

    // ------------
    // Instruction Memory / Data Memory
    // ------------
    wire [31:0] instruction_from_mem;
    wire [63:0] mem_rdata;
    memory memory(
        .pc(PC),
        .clk(clk),
        .reset(reset),
        .mem_write_enable(EX_MEM_mem_write_enable),
        .rw_val(EX_MEM_rw_val),
        .rw_addr(EX_MEM_rw_addr),
        .instruction(instruction_from_mem),
        .r_out(mem_rdata)
    );

    // ------------
    // Instruction Decode
    // ------------
    wire [4:0]  controlSignal;
    wire [4:0]  rd, rs, rt;
    wire [11:0] L;
    wire        rtPassed;
    instruction_decoder decoder_inst(
        .instruction(IF_ID_IR),
        .controlSignal(controlSignal),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .L(L),
        .rtPassed(rtPassed)
    );

    // ------------
    // Register File (write back through MEM/WB)
    // ------------
    wire [63:0] wb_data = MEM_WB_memToReg ? MEM_WB_mem_data : MEM_WB_alu_result;
    wire [63:0] reg_val1, reg_val2, rdVal, r31_val;
    register_file reg_file(
        .clk(clk),
        .reset(reset),
        .write_enable(MEM_WB_writeEnable),
        .dataInput(wb_data),
        .readAddress1(rs),
        .readAddress2(rt),
        .writeAddress(MEM_WB_rd),
        .lPassed(ID_EX_rtPassed),
        .L(ID_EX_L),
        .value1(reg_val1),
        .value2(reg_val2),
        .rdVal(rdVal),
        .r31_val(r31_val)
    );

    // Halt detection
    assign hlt = (controlSignal == 5'h0f) && (L == 12'h0);

    // ------------------------------------------------------------------------------------------------
    //  IF Stage: Fetch instruction, update PC
    // ------------------------------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC        <= 64'h2000;
            IF_ID_IR  <= 32'b0;
            IF_ID_PC  <= 64'b0;
        end else begin
            PC        <= EX_MEM_changing_pc ? EX_MEM_updated_next : PC + 4;
            IF_ID_IR  <= instruction_from_mem;
            IF_ID_PC  <= PC;
        end
    end

    // ------------------------------------------------------------------------------------------------
    //  ID Stage: Read registers, sign-extend literal, generate memToReg
    // ------------------------------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ID_EX_control   <= 5'b0;
            ID_EX_rs        <= 5'b0;
            ID_EX_rt        <= 5'b0;
            ID_EX_rd        <= 5'b0;
            ID_EX_L         <= 12'b0;
            ID_EX_rtPassed  <= 1'b0;
            ID_EX_reg_val1  <= 64'b0;
            ID_EX_reg_val2  <= 64'b0;
            ID_EX_PC        <= 64'b0;
            ID_EX_memToReg  <= 1'b0;
        end else begin
            ID_EX_control   <= controlSignal;
            ID_EX_rs        <= rs;
            ID_EX_rt        <= rt;
            ID_EX_rd        <= rd;
            ID_EX_L         <= L;
            ID_EX_rtPassed  <= rtPassed;
            ID_EX_reg_val1  <= reg_val1;
            ID_EX_reg_val2  <= reg_val2;
            ID_EX_PC        <= IF_ID_PC;
            ID_EX_memToReg  <= (controlSignal == 5'b10000);
        end
    end

    // ------------------------------------------------------------------------------------------------
    //  EX Stage: ALU operations & branch decision
    // ------------------------------------------------------------------------------------------------
    wire [63:0] signExtL       = {{52{ID_EX_L[11]}}, ID_EX_L};
    // FIXED: select register value when rtPassed, else literal
    wire [63:0] alu_operand2   = ID_EX_rtPassed ? ID_EX_reg_val2 : signExtL;

    wire [63:0] alu_result;
    wire        alu_writeEnable;
    wire        mem_write_enable_alu;
    wire [31:0] alu_rw_addr;
    wire [63:0] alu_rw_val;
    wire [63:0] alu_updated_next;
    wire        alu_changing_pc;

    ALU alu_inst(
        .pc(ID_EX_PC),
        .rdVal(rdVal),
        .operand1(ID_EX_reg_val1),
        .operand2(alu_operand2),
        .opcode(ID_EX_control),
        .r_out(mem_rdata),
        .r31_val(r31_val),
        .result(alu_result),
        .writeEnable(alu_writeEnable),
        .mem_write_enable(mem_write_enable_alu),
        .rw_addr(alu_rw_addr),
        .rw_val(alu_rw_val),
        .updated_next(alu_updated_next),
        .changing_pc(alu_changing_pc)
    );

    // ------------------------------------------------------------------------------------------------
    // EX/MEM Pipeline Register
    // ------------------------------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            EX_MEM_rd               <= 5'b0;
            EX_MEM_alu_result       <= 64'b0;
            EX_MEM_reg_val2         <= 64'b0;
            EX_MEM_mem_write_enable <= 1'b0;
            EX_MEM_writeEnable      <= 1'b0;
            EX_MEM_changing_pc      <= 1'b0;
            EX_MEM_rw_addr          <= 32'b0;
            EX_MEM_rw_val           <= 64'b0;
            EX_MEM_updated_next     <= 64'b0;
            EX_MEM_memToReg         <= 1'b0;
        end else begin
            EX_MEM_rd               <= ID_EX_rd;
            EX_MEM_alu_result       <= alu_result;
            EX_MEM_reg_val2         <= ID_EX_reg_val2;
            EX_MEM_mem_write_enable <= mem_write_enable_alu;
            EX_MEM_writeEnable      <= alu_writeEnable;
            EX_MEM_changing_pc      <= alu_changing_pc;
            EX_MEM_rw_addr          <= alu_rw_addr;
            EX_MEM_rw_val           <= alu_rw_val;
            EX_MEM_updated_next     <= alu_updated_next;
            EX_MEM_memToReg         <= ID_EX_memToReg;
        end
    end

    // ------------------------------------------------------------------------------------------------
    //  MEM Stage: Data memory access
    // ------------------------------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            MEM_WB_rd         <= 5'b0;
            MEM_WB_alu_result <= 64'b0;
            MEM_WB_mem_data   <= 64'b0;
            MEM_WB_writeEnable<= 1'b0;
            MEM_WB_memToReg   <= 1'b0;
        end else begin
            MEM_WB_rd         <= EX_MEM_rd;
            MEM_WB_alu_result <= EX_MEM_alu_result;
            MEM_WB_mem_data   <= mem_rdata;
            MEM_WB_writeEnable<= EX_MEM_writeEnable;
            MEM_WB_memToReg   <= EX_MEM_memToReg;
        end
    end

endmodule
