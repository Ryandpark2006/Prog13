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

//------------------------------------------------------
// FSM-Style Tinker Core
//------------------------------------------------------
module tinker_core(
    input clk,
    input reset,
    output hlt
);
    reg [63:0] programCounter;
    reg halted;
    reg [31:0] IR;
    reg [63:0] A, B;
    reg [63:0] result_reg;
    localparam [2:0] FETCH=3'd0, DECODE=3'd1, EXECUTE=3'd2, MEM=3'd3, WRITEBACK=3'd4;
    reg [2:0] state, next_state;

    wire [31:0] instruction_from_mem;
    wire [63:0] mem_rdata;
    reg         mem_write;
    reg  [31:0] mem_rw_addr;
    reg  [63:0] mem_rw_val;
    wire final_write_enable;

    memory memory(
        .pc(programCounter), .clk(clk), .reset(reset),
        .mem_write_enable(mem_write), .rw_val(mem_rw_val), .rw_addr(mem_rw_addr),
        .instruction(instruction_from_mem), .r_out(mem_rdata)
    );

    wire [4:0] controlSignal, rd, rs, rt;
    wire [11:0] L;
    wire        rtPassed;
    instruction_decoder decoder_inst(
        .instruction(IR), .controlSignal(controlSignal), .rd(rd), .rs(rs), .rt(rt), .L(L), .rtPassed(rtPassed)
    );

    wire [63:0] signExtendedLiteral = {{52{L[11]}}, L};
    wire        local_hlt = (controlSignal==5'h0f) && (signExtendedLiteral[3:0]==4'h0);
    assign hlt = halted;

    wire [63:0] reg_val1, reg_val2, rdVal, r31_val;
    wire [4:0] operator1 = (controlSignal==5'b10011)? rs : (rtPassed? rs: rd);
    wire [4:0] operator2 = rt;
    register_file register_file(
        .clk(clk), .reset(reset), .write_enable((state==WRITEBACK)? final_write_enable:1'b0),
        .dataInput(result_reg), .readAddress1(operator1), .readAddress2(operator2),
        .writeAddress(rd), .lPassed(~rtPassed), .L(L), .value1(reg_val1), .value2(reg_val2),
        .rdVal(rdVal), .r31_val(r31_val)
    );

    wire [63:0] alu_result;
    wire        alu_writeEnable;
    wire        mem_write_enable_alu;
    wire [31:0] alu_rw_addr;
    wire [63:0] alu_rw_val;
    wire [63:0] updated_next;
    wire        alu_changing_pc;
    ALU alu_inst(
        .pc(programCounter), .rdVal(rdVal), .operand1(A), .operand2(B),
        .opcode(controlSignal), .r_out(mem_rdata), .r31_val(r31_val),
        .result(alu_result), .writeEnable(alu_writeEnable),
        .mem_write_enable(mem_write_enable_alu),
        .rw_addr(alu_rw_addr), .rw_val(alu_rw_val),
        .updated_next(updated_next), .changing_pc(alu_changing_pc)
    );

    wire [63:0] fpu_result;
    wire        fpu_writeEnable;
    FPU fpu_inst(
        .operand1(A), .operand2(B), .opcode(controlSignal),
        .result(fpu_result), .writeEnable(fpu_writeEnable)
    );

    wire [63:0] final_result = ((controlSignal==5'b10100)||(controlSignal==5'b10101)||
                               (controlSignal==5'b10110)||(controlSignal==5'b10111))?
                               fpu_result: alu_result;
    assign final_write_enable = ((controlSignal==5'b10100)||(controlSignal==5'b10101)||
                                 (controlSignal==5'b10110)||(controlSignal==5'b10111))?
                                 fpu_writeEnable: alu_writeEnable;

    // Next-state
    always @(*) begin
        case(state)
          FETCH:     next_state=DECODE;
          DECODE:    next_state=EXECUTE;
          EXECUTE:   next_state = ((controlSignal==5'b10000)||(controlSignal==5'b10011))?MEM:WRITEBACK;
          MEM:       next_state=WRITEBACK;
          WRITEBACK: next_state=FETCH;
          default:   next_state=FETCH;
        endcase
    end

    // FSM main
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IR             <=32'b0;
            A              <=64'b0;
            B              <=64'b0;
            result_reg     <=64'b0;
            mem_write      <=1'b0;
            mem_rw_addr    <=32'b0;
            mem_rw_val     <=64'b0;
            state          <=FETCH;
            halted         <=1'b0;
            programCounter<=64'h2000;
        end else begin
            state<=next_state;
            case(state)
               FETCH: IR<=instruction_from_mem;
               DECODE: begin A<=reg_val1; B<=reg_val2; end
               EXECUTE: begin
                   if (controlSignal==5'b10010) begin
                       result_reg<=final_result; mem_rw_addr<=32'b0;
                       mem_rw_val<=64'b0; mem_write<=1'b0;
                   end else if (controlSignal==5'b10000||controlSignal==5'b10011||controlSignal==5'b01100) begin
                       result_reg<=final_result; mem_rw_addr<=alu_rw_addr;
                       mem_rw_val<=alu_rw_val; mem_write<=mem_write_enable_alu;
                   end else begin
                       result_reg<=final_result; mem_rw_addr<=alu_rw_addr;
                       mem_rw_val<=alu_rw_val; mem_write<=1'b0;
                   end
               end
               MEM: begin
                   if (controlSignal==5'b10000)
                       result_reg<={{32{mem_rdata[31]}},mem_rdata};
                   else
                       result_reg<=mem_rdata;
                   mem_write<=1'b0;
               end
               WRITEBACK: mem_write<=1'b0;
            endcase
            if (local_hlt) halted<=1'b1;
            if (!halted && state==WRITEBACK) begin
                case(controlSignal)
                    5'b01101: programCounter<=mem_rdata;
                    5'b01100: programCounter<=updated_next;
                    5'b10010: programCounter<=programCounter+4;
                    default: if (alu_changing_pc) programCounter<=updated_next;
                             else programCounter<=programCounter+4;
                endcase
            end
        end
    end
endmodule
