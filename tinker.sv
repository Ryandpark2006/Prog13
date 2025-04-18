module ALU(
    input wire [4:0] opcode,
    input wire [63:0] operand1,
    input wire [63:0] operand2,
    input wire [63:0] operand3,
    input wire [63:0] signExtendedLiteral,
    input wire [63:0] pc,
    input wire [63:0] stackPointer,
    input wire [63:0] memoryReadVal,
    output reg [63:0] result,
    output reg [63:0] readWriteAddress,
    output reg [63:0] updated_next,
    output reg changing_pc,
    output reg writeEnable,
    output reg mem_write_enable,
    output reg halt
);
    always @(*) begin
        // Default values (same as reference)
        result            = 64'd0;
        readWriteAddress  = 64'd0;
        updated_next      = pc + 64'd4;
        writeEnable       = 1'b1;
        mem_write_enable  = 1'b0;
        changing_pc       = 1'b0;
        halt              = 1'b0;

        case (opcode)
            5'h00: result = operand1 & operand2;                         // and
            5'h01: result = operand1 | operand2;                         // or
            5'h02: result = operand1 ^ operand2;                         // xor
            5'h03: result = ~operand1;                                   // not
            5'h04: result = operand1 >> operand2[5:0];                   // shftr
            5'h05: result = operand3 >> signExtendedLiteral;            // shftri
            5'h06: result = operand1 << operand2[5:0];                   // shftl
            5'h07: result = operand3 << signExtendedLiteral;            // shftli

            5'h08: begin                                                 // br rd
                changing_pc  = 1;
                writeEnable  = 0;
                updated_next = operand3;
            end
            5'h09: begin                                                 // brr rd
                changing_pc  = 1;
                writeEnable  = 0;
                updated_next = pc + operand3;
            end
            5'h0a: begin                                                 // brr L
                changing_pc  = 1;
                writeEnable  = 0;
                updated_next = pc + signExtendedLiteral;
            end
            5'h0b: begin                                                 // brnz rd, rs
                writeEnable = 0;
                if (operand1 != 0) begin
                    updated_next = operand3;
                    changing_pc = 1;
                end
            end
            5'h0c: begin                                                 // call rd, rs, rt
                writeEnable       = 0;
                mem_write_enable  = 1;
                changing_pc       = 1;
                updated_next      = operand3;
                readWriteAddress  = stackPointer - 64'd8;
                result            = pc + 64'd4;
            end
            5'h0d: begin                                                 // return
                writeEnable       = 0;
                changing_pc       = 1;
                readWriteAddress  = stackPointer - 64'd8;
                updated_next      = memoryReadVal;
            end
            5'h0e: begin                                                 // brgt rd, rs, rt
                writeEnable = 0;
                if ($signed(operand1) > $signed(operand2)) begin
                    updated_next = operand3;
                    changing_pc = 1;
                end
            end
            5'h0f: begin                                                 // priv
                writeEnable = 0;
                if (signExtendedLiteral[3:0] == 4'h0) begin
                    halt = 1;
                end
            end
            5'h10: begin                                                 // mov rd, (rs)(L)
                readWriteAddress = operand1 + signExtendedLiteral;
                result = memoryReadVal;
            end
            5'h11: result = operand1;                                    // mov rd, rs
            5'h12: result = {operand3[63:12], signExtendedLiteral[11:0]};// mov rd, L
            5'h13: begin                                                 // mov (rd)(L), rs
                writeEnable = 0;
                mem_write_enable = 1;
                readWriteAddress = operand3 + signExtendedLiteral;
                result = operand1;
            end
            5'h14: result = $realtobits($bitstoreal(operand1) + $bitstoreal(operand2)); // addf
            5'h15: result = $realtobits($bitstoreal(operand1) - $bitstoreal(operand2)); // subf
            5'h16: result = $realtobits($bitstoreal(operand1) * $bitstoreal(operand2)); // mulf
            5'h17: result = $realtobits($bitstoreal(operand1) / $bitstoreal(operand2)); // divf

            5'h18: result = operand1 + operand2;                         // add
            5'h19: result = operand3 + signExtendedLiteral;             // addi
            5'h1a: result = operand1 - operand2;                         // sub
            5'h1b: result = operand3 - signExtendedLiteral;             // subi
            5'h1c: result = operand1 * operand2;                         // mul
            5'h1d: result = (operand2 == 0) ? 64'd0 : operand1 / operand2; // div

            default: begin
                writeEnable = 0;
                mem_write_enable = 0;
                changing_pc = 0;
            end
        endcase
    end
endmodule




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
            default:  result = 64'b0;
        endcase
        writeEnable = 1;
    end
endmodule

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
    assign rtPassed = !(controlSignal == 5'b11001 ||  // addi
                    controlSignal == 5'b11011 ||  // subi
                    controlSignal == 5'b00101 ||  // shftri
                    controlSignal == 5'b00111 ||  // shftli
                    controlSignal == 5'b10010 ||  // mov_L_to_reg
                    controlSignal == 5'b01010 ||  // brr
                    controlSignal == 5'b10011 ||  // store
                    controlSignal == 5'b10000);   // load

    // assign rtPassed      = (controlSignal == 5'b11001 || controlSignal == 5'b11011 ||
    //                         controlSignal == 5'b00101 || controlSignal == 5'b00111 ||
    //                         controlSignal == 5'b10010 || controlSignal == 5'b01010 ||
    //                         controlSignal == 5'b10011 || controlSignal == 5'b10000)
    //                        ? 0 : 1;
endmodule

module memory(
    input  wire [63:0] pc,
    input  wire        clk,
    input  wire        reset,
    input  wire        mem_write_enable,
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
    reg [63:0] registers [31:0];

    assign value1 = registers[readAddress1];
    assign value2 = lPassed ? {{52{L[11]}},L} : registers[readAddress2];
    assign rdVal  = registers[readAddress3];
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


module tinker_core(
    input  wire        clk,
    input  wire        reset,
    output wire        hlt
);
    reg [63:0] PC;
    reg [1:0]  stall_cnt;

    reg [63:0] IF_ID_PC;
    reg [31:0] IF_ID_IR;

    wire [4:0]  IF_ctrl, IF_rd, IF_rs, IF_rt;
    wire [11:0] IF_L;
    wire        IF_rtPassed;

    instruction_decoder dec(
        .instruction  (IF_ID_IR),
        .controlSignal(IF_ctrl),
        .rd           (IF_rd),
        .rs           (IF_rs),
        .rt           (IF_rt),
        .L            (IF_L),
        .rtPassed     (IF_rtPassed)
    );

    wire load_use_hazard = (EX_MEM_ctrl == 5'b10000) && (
        (IF_ctrl != 5'b00000 && EX_MEM_rd == IF_rs) ||
        (IF_ctrl != 5'b00000 && EX_MEM_rd == IF_rt && IF_rtPassed)
    );

    wire flushFetch = aluChangePC;
    wire fetchStall = load_use_hazard;

    wire [31:0] inst;
    wire [63:0] mem_rdata;
    memory memory (
        .pc               (PC),
        .clk              (clk),
        .reset            (reset),
        .mem_write_enable (EX_MEM_memWrite),
        .rw_val           (EX_MEM_wrData),
        .rw_addr          (EX_MEM_addr[31:0]),
        .instruction      (inst),
        .r_out            (mem_rdata)
    );

    wire [31:0] next_IF_ID_IR = flushFetch ? 32'd0 :
                                fetchStall ? IF_ID_IR :
                                             inst;
    wire [63:0] next_IF_ID_PC = flushFetch ? 64'd0 :
                                fetchStall ? IF_ID_PC :
                                             PC;

    reg [63:0] ID_EX_PC;
    reg [4:0]  ID_EX_ctrl, ID_EX_rd, ID_EX_rs, ID_EX_rt;
    reg [11:0] ID_EX_L;
    reg        ID_EX_rtPassed;
    reg [63:0] ID_EX_A, ID_EX_B;
    reg [63:0] ID_EX_r31, ID_EX_rdVal;

    reg [4:0]  EX_MEM_ctrl, EX_MEM_rd;
    reg [63:0] EX_MEM_ALU, EX_MEM_B;
    reg        EX_MEM_memWrite, EX_MEM_regWrite, EX_MEM_changePC;
    reg [63:0] EX_MEM_addr, EX_MEM_wrData, EX_MEM_target;

    reg [4:0]  MEM_WB_ctrl, MEM_WB_rd;
    reg [63:0] MEM_WB_ALU, MEM_WB_memData;
    reg        MEM_WB_regWrite, MEM_WB_memToReg;

    wire [63:0] regOut1, regOut2, rdValSignal, r31Val;
    register_file reg_file(
        .clk         (clk),
        .reset       (reset),
        .write_enable(MEM_WB_regWrite),
        .dataInput   (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU),
        .readAddress1(IF_rs),
        .readAddress2(IF_rt),
        .readAddress3(IF_rd),
        .writeAddress(MEM_WB_rd),
        .lPassed     (~IF_rtPassed),
        .L           (IF_L),
        .value1      (regOut1),
        .value2      (regOut2),
        .rdVal       (rdValSignal),
        .r31_val     (r31Val)
    );

    wire forwardA_EX = EX_MEM_regWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs);
    wire forwardA_MEM = MEM_WB_regWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs);
    wire forwardB_EX = EX_MEM_regWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rt) && ID_EX_rtPassed;
    wire forwardB_MEM = MEM_WB_regWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rt) && ID_EX_rtPassed;
    wire forwardRD_EX = EX_MEM_regWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rd);
    wire forwardRD_MEM = MEM_WB_regWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rd);

    wire [63:0] forwarded_A = forwardA_EX ? EX_MEM_ALU :
                             (forwardA_MEM ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU) : ID_EX_A);

    wire [63:0] forwarded_B = forwardB_EX ? EX_MEM_ALU :
                             (forwardB_MEM ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU) : ID_EX_B);

    wire [63:0] forwarded_rdVal = forwardRD_EX ? EX_MEM_ALU :
                                 (forwardRD_MEM ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU) : ID_EX_rdVal);

    wire [63:0] aluOp1 = (ID_EX_ctrl == 5'b11001 || ID_EX_ctrl == 5'b11011)
                        ? forwarded_rdVal : forwarded_A;
    wire [63:0] aluOp2 = ID_EX_rtPassed
                        ? forwarded_B
                        : {{52{ID_EX_L[11]}}, ID_EX_L};

    wire [63:0] aluResult, aluAddr, aluUpdatedNext;
    wire        aluRegWrite, aluMemWrite, aluChangePC, aluHalt;
    ALU ALU_INST(
        .opcode           (ID_EX_ctrl),
        .operand1         (aluOp1),
        .operand2         (aluOp2),
        .operand3         (forwarded_rdVal),
        .signExtendedLiteral({{52{ID_EX_L[11]}}, ID_EX_L}),
        .pc               (ID_EX_PC),
        .stackPointer     (ID_EX_r31),
        .memoryReadVal    (mem_rdata),
        .result           (aluResult),
        .readWriteAddress (aluAddr),
        .updated_next     (aluUpdatedNext),
        .changing_pc      (aluChangePC),
        .writeEnable      (aluRegWrite),
        .mem_write_enable (aluMemWrite),
        .halt             (aluHalt)
    );

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 64'h2000;
            IF_ID_PC <= 0;
            IF_ID_IR <= 0;
        end else begin
            if (flushFetch)
                PC <= aluUpdatedNext;
            else if (!fetchStall)
                PC <= PC + 4;

            IF_ID_PC <= next_IF_ID_PC;
            IF_ID_IR <= next_IF_ID_IR;
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset || EX_MEM_changePC || fetchStall) begin
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
        end else begin
            ID_EX_ctrl     <= IF_ctrl;
            ID_EX_rd       <= IF_rd;
            ID_EX_rs       <= IF_rs;
            ID_EX_rt       <= IF_rt;
            ID_EX_L        <= IF_L;
            ID_EX_rtPassed <= IF_rtPassed;
            ID_EX_A        <= regOut1;
            ID_EX_B        <= regOut2;
            ID_EX_PC       <= IF_ID_PC;
            ID_EX_r31      <= r31Val;
            ID_EX_rdVal    <= rdValSignal;
        end
    end

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
            EX_MEM_wrData   <= aluResult;
            EX_MEM_changePC <= aluChangePC;
            EX_MEM_target   <= aluUpdatedNext;
        end
    end

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

    reg halt_flag;
    always @(posedge clk or posedge reset) begin
        if (reset)
            halt_flag <= 0;
        else if (aluHalt)
            halt_flag <= 1;
    end
    assign hlt = halt_flag;

endmodule
