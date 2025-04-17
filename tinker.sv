//------------------------------------------------------
// ALU Module
//------------------------------------------------------
module ALU(
    input  wire [63:0] pc,
    input  wire [63:0] rdVal,
    input  wire [63:0] operand1,
    input  wire [63:0] operand2,
    input  wire [4:0]  opcode,
    input  wire [63:0] r_out,
    input  wire [63:0] r31_val,
    output reg  [63:0] result,
    output reg         writeEnable,
    output reg         mem_write_enable,
    output reg  [31:0] rw_addr,
    output reg  [63:0] rw_val,
    output reg  [63:0] updated_next,
    output reg         changing_pc
);
    always @(*) begin
        // defaults
        writeEnable      = 1'b1;
        mem_write_enable = 1'b0;
        changing_pc      = 1'b0;
        updated_next     = pc + 64'd4;
        rw_addr          = 32'd0;
        rw_val           = 64'd0;

        case(opcode)
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
            5'b00100: result = operand1 >> operand2; // shr reg
            5'b00101: result = operand1 >> operand2; // shr imm
            5'b00110: result = operand1 << operand2; // shl reg
            5'b00111: result = operand1 << operand2; // shl imm

            // Data movement
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
            5'b10001: result = operand1;             // mov rd, rs

            // MOV_L_TO_REG (integer‐literal high half):
            //    movL rX, L  →  rX = (signext L) << 12
            5'b10010: begin
                result = (operand2[11:0] << 12);
            end

            // Control
            5'b01000: begin // jump abs
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
                    changing_pc  = 1'b1;
                    updated_next = rdVal;
                end
            end
            5'b01110: begin // brgt
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = ($signed(operand1) > $signed(operand2))
                               ? rdVal
                               : (pc + 64'd4);
            end

            // CALL
            5'b01100: begin
                writeEnable      = 1'b0;
                mem_write_enable = 1'b1;
                changing_pc      = 1'b1;
                rw_val           = pc + 64'd4;       // return address
                rw_addr          = r31_val - 32'd8;  // push onto stack
                updated_next     = rdVal;            // jump target
            end

            // RETURN
            5'b01101: begin
                writeEnable      = 1'b0;
                mem_write_enable = 1'b1;             // pop isn't a write?
                changing_pc      = 1'b1;
                rw_addr          = r31_val - 32'd8;  // read return addr
                updated_next     = r_out;
            end

            // Floating ALU
            5'b10100: result = $realtobits($bitstoreal(operand1) + $bitstoreal(operand2));
            5'b10101: result = $realtobits($bitstoreal(operand1) - $bitstoreal(operand2));
            5'b10110: result = $realtobits($bitstoreal(operand1) * $bitstoreal(operand2));
            5'b10111: result = $realtobits($bitstoreal(operand1) / $bitstoreal(operand2));

            default: begin
                writeEnable      = 1'b0;
                mem_write_enable = 1'b0;
                result           = 64'd0;
            end
        endcase
    end
endmodule

//------------------------------------------------------
// FPU Module (unchanged)
//------------------------------------------------------
module FPU(
    input  wire [63:0] operand1,
    input  wire [63:0] operand2,
    input  wire [4:0]  opcode,
    output reg  [63:0] result,
    output reg         writeEnable
);
    always @(*) begin
        writeEnable = 1'b1;
        case(opcode)
            5'b10100: result = $realtobits($bitstoreal(operand1) + $bitstoreal(operand2));
            5'b10101: result = $realtobits($bitstoreal(operand1) - $bitstoreal(operand2));
            5'b10110: result = $realtobits($bitstoreal(operand1) * $bitstoreal(operand2));
            5'b10111: result = $realtobits($bitstoreal(operand1) / $bitstoreal(operand2));
            default:  result = 64'b0;
        endcase
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
    // L‑type ops (no second reg) flatten rtPassed=0
    assign rtPassed = !( controlSignal == 5'b11001 ||  // addi
                         controlSignal == 5'b11011 ||  // subi
                         controlSignal == 5'b00101 ||  // shr imm
                         controlSignal == 5'b00111 ||  // shl imm
                         controlSignal == 5'b10010 ||  // movL
                         controlSignal == 5'b01010 ||  // jump rel2
                         controlSignal == 5'b10011 ||  // store
                         controlSignal == 5'b10000 );  // load
endmodule

//------------------------------------------------------
// Memory
//------------------------------------------------------
module memory(
    input  wire        clk,
    input  wire        reset,
    input  wire [63:0] pc,
    input  wire        mem_write_enable,
    input  wire [63:0] rw_val,
    input  wire [31:0] rw_addr,
    output wire [31:0] instruction,
    output wire [63:0] r_out
);
    reg [7:0] bytes [0:524287];
    // Little‑endian fetch
    assign instruction = { bytes[pc+3], bytes[pc+2],
                           bytes[pc+1], bytes[pc] };
    // 64‑bit read
    assign r_out = { bytes[rw_addr+7], bytes[rw_addr+6],
                     bytes[rw_addr+5], bytes[rw_addr+4],
                     bytes[rw_addr+3], bytes[rw_addr+2],
                     bytes[rw_addr+1], bytes[rw_addr  ] };

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 524288; i=i+1)
                bytes[i] <= 8'b0;
        end else if (mem_write_enable) begin
            // store 64 bits, little‑endian
            bytes[rw_addr  ] <= rw_val[7:0];
            bytes[rw_addr+1] <= rw_val[15:8];
            bytes[rw_addr+2] <= rw_val[23:16];
            bytes[rw_addr+3] <= rw_val[31:24];
            bytes[rw_addr+4] <= rw_val[39:32];
            bytes[rw_addr+5] <= rw_val[47:40];
            bytes[rw_addr+6] <= rw_val[55:48];
            bytes[rw_addr+7] <= rw_val[63:56];
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
    reg [63:0] registers [0:31];
    assign value1 = registers[readAddress1];
    assign value2 = lPassed
                    ? {{52{L[11]}}, L}  // sign‑ext immediate
                    : registers[readAddress2];
    assign rdVal  = registers[writeAddress];
    assign r31_val= registers[31];

    integer j;
    always @(posedge clk) begin
        if (reset) begin
            for (j = 0; j < 31; j=j+1)
                registers[j] <= 64'd0;
            registers[31] <= 64'd524288;  // initial stack base
        end else if (write_enable) begin
            registers[writeAddress] <= dataInput;
        end
    end
endmodule

//------------------------------------------------------
// Top‐Level: 5‑Stage FSM Tinker Core
//------------------------------------------------------
module tinker_core(
    input  wire clk,
    input  wire reset,
    output wire hlt
);
    // FSM states
    localparam FETCH     = 3'd0,
               DECODE    = 3'd1,
               EXECUTE   = 3'd2,
               MEM       = 3'd3,
               WRITEBACK = 3'd4;

    reg [2:0] state, next_state;
    reg [63:0] programCounter;
    reg        halted;

    // internal latches
    reg  [31:0] IR;
    reg  [63:0] A, B, result_reg;
    reg         mem_write;
    reg  [31:0] mem_rw_addr;
    reg  [63:0] mem_rw_val;

    // decode outputs
    wire [31:0] instruction_from_mem;
    wire [4:0]  controlSignal, rd, rs, rt;
    wire [11:0] L;
    wire        rtPassed;

    // register file read
    wire [63:0] reg_val1, reg_val2, rdVal, r31_val;
    wire        rf_we;
    wire [63:0] rf_wdata;

    // ALU/FPU
    wire [63:0] alu_result, fpu_result;
    wire        alu_we, alu_mem_we, alu_chg_pc;
    wire [31:0] alu_rw_addr;
    wire [63:0] alu_rw_val;
    wire [63:0] updated_next_pc;

    // memory read data
    wire [63:0] mem_rdata;

    // Halt detection
    wire [63:0] signExtL = {{52{L[11]}}, L};
    wire        local_hlt = (controlSignal == 5'h0f) && (signExtL[3:0] == 4'h0);

    //-----------------------------------------------------------------------------
    // combinational
    //-----------------------------------------------------------------------------
    // next‐state logic
    always @(*) begin
        case(state)
            FETCH:     next_state = DECODE;
            DECODE:    next_state = EXECUTE;
            EXECUTE:   next_state = MEM;
            MEM:       next_state = WRITEBACK;
            WRITEBACK: next_state = FETCH;
            default:   next_state = FETCH;
        endcase
    end

    // memory fetch
    memory mem0 (
        .clk(clk),
        .reset(reset),
        .pc(programCounter),
        .mem_write_enable(1'b0),
        .rw_val(64'd0),
        .rw_addr(32'd0),
        .instruction(instruction_from_mem),
        .r_out(mem_rdata)
    );

    // decode
    instruction_decoder dec0 (
        .instruction(instruction_from_mem),
        .controlSignal(controlSignal),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .L(L),
        .rtPassed(rtPassed)
    );

    // regfile (writes in WB only)
    assign rf_we    = (state == WRITEBACK) && writeEnable_final;
    assign rf_wdata = result_reg;
    register_file rf0 (
        .clk(clk),
        .reset(reset),
        .write_enable(rf_we),
        .dataInput(rf_wdata),
        .readAddress1(rs),
        .readAddress2(rt),
        .writeAddress(rd),
        .lPassed(~rtPassed),
        .L(L),
        .value1(reg_val1),
        .value2(reg_val2),
        .rdVal(rdVal),
        .r31_val(r31_val)
    );

    // ALU / FPU
    ALU alu0 (
        .pc(programCounter),
        .rdVal(rdVal),
        .operand1(A),
        .operand2(B),
        .opcode(controlSignal),
        .r_out(mem_rdata),
        .r31_val(r31_val),
        .result(alu_result),
        .writeEnable(alu_we),
        .mem_write_enable(alu_mem_we),
        .rw_addr(alu_rw_addr),
        .rw_val(alu_rw_val),
        .updated_next(updated_next_pc),
        .changing_pc(alu_chg_pc)
    );
    FPU fpu0 (
        .operand1(A),
        .operand2(B),
        .opcode(controlSignal),
        .result(fpu_result),
        .writeEnable()  // ignored
    );

    //--------------------------------------------------------------------
    // sequential: 5‑stage pipeline via FSM
    //--------------------------------------------------------------------
    reg writeEnable_final;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state          <= FETCH;
            programCounter <= 64'h2000;  // testbench code starts at 0x2000
            halted         <= 1'b0;
            IR             <= 32'd0;
            A <= 0; B <= 0;
            result_reg <= 0;
            mem_write       <= 1'b0;
            mem_rw_addr     <= 32'd0;
            mem_rw_val      <= 64'd0;
            writeEnable_final <= 1'b0;
        end else begin
            // advance state
            state <= next_state;

            // pipeline latches
            case(state)
                FETCH: begin
                    IR <= instruction_from_mem;
                end
                DECODE: begin
                    A <= reg_val1;
                    B <= reg_val2;
                end
                EXECUTE: begin
                    // honor store / call / return
                    mem_write   <= alu_mem_we;
                    mem_rw_addr <= alu_rw_addr;
                    mem_rw_val  <= alu_rw_val;
                    // choose ALU vs FPU
                    result_reg  <= ((controlSignal >= 5'b10100 && controlSignal <= 5'b10111)
                                    ? fpu_result
                                    : alu_result);
                end
                MEM: begin
                    // on a load, the data is sign‑extended from 32 to 64:
                    if (controlSignal == 5'b10000)
                        result_reg <= {{32{mem_rdata[31]}}, mem_rdata[31:0]};
                    else
                        result_reg <= mem_rdata;
                    mem_write   <= 1'b0;
                end
                WRITEBACK: begin
                    mem_write       <= 1'b0;
                    writeEnable_final <= alu_we;
                end
            endcase

            // commit PC (or halt) in WRITEBACK
            if (!halted && state == WRITEBACK) begin
                if (local_hlt)
                    halted <= 1'b1;
                else if (controlSignal == 5'b01101)       // return
                    programCounter <= mem_rdata;
                else if (controlSignal == 5'b01100)       // call
                    programCounter <= updated_next_pc;
                else if (alu_chg_pc)                      // jump/br
                    programCounter <= updated_next_pc;
                else
                    programCounter <= programCounter + 64'd4;
            end
        end
    end

    assign hlt = halted;
endmodule
