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
            5'b00100: result = operand1 >> operand2; // shftr
            5'b00101: result = operand1 >> operand2; // shftri
            5'b00110: result = operand1 << operand2; // shftl
            5'b00111: result = operand1 << operand2; // shftli

            // Data movement
            5'b10001: result = operand1;             // mov rd, rs
            5'b10010: result = {rdVal[63:12], operand2[11:0]}; // mov_L_to_reg
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
            5'b01110: begin // brgt
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = ($signed(operand1) > $signed(operand2)) ? rdVal : pc + 4;
            end

            // **Return**: skip exactly one instruction
            5'b01101: begin  // return
                writeEnable      = 1'b0;
                mem_write_enable = 1'b0;
                changing_pc      = 1'b1;
                rw_addr          = r31_val - 8; // pop return address slot
                updated_next     = r_out;       // jump there
            end

            // **Call** remains unchanged
            5'b01100: begin // call
                writeEnable      = 1'b0;
                changing_pc      = 1'b1;
                mem_write_enable = 1'b1;
                rw_val           = pc + 4;
                rw_addr          = r31_val - 8;
                updated_next     = rdVal;
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
    assign rtPassed      = (controlSignal == 5'b11001 || controlSignal == 5'b11011 ||
                            controlSignal == 5'b00101 || controlSignal == 5'b00111 ||
                            controlSignal == 5'b10010 || controlSignal == 5'b01010 ||
                            controlSignal == 5'b10011 || controlSignal == 5'b10000)
                           ? 0 : 1;
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

// module tinker_core(
//     input  wire        clk,
//     input  wire        reset,
//     output wire        hlt
// );
//     reg [63:0] PC;
//     reg [1:0]  stall_cnt;
//     reg [63:0] IF_ID_PC;
//     reg [31:0] IF_ID_IR;

//     wire [4:0]  IF_ctrl, IF_rd, IF_rs, IF_rt;
//     wire [11:0] IF_L;
//     wire        IF_rtPassed;
//     instruction_decoder dec(
//         .instruction (IF_ID_IR),
//         .controlSignal(IF_ctrl),
//         .rd           (IF_rd),
//         .rs           (IF_rs),
//         .rt           (IF_rt),
//         .L            (IF_L),
//         .rtPassed     (IF_rtPassed)
//     );

//     wire load_use_hazard = (EX_MEM_ctrl == 5'b10000) && (
//         (IF_ctrl != 5'b0 && EX_MEM_rd == IF_rs) ||
//         (IF_ctrl != 5'b0 && EX_MEM_rd == IF_rt && IF_rtPassed)
//     );

//     reg [63:0] ID_EX_PC;
//     reg [4:0]  ID_EX_ctrl, ID_EX_rd, ID_EX_rs, ID_EX_rt;
//     reg [11:0] ID_EX_L;
//     reg        ID_EX_rtPassed;
//     reg [63:0] ID_EX_A, ID_EX_B;
//     reg [63:0] ID_EX_r31, ID_EX_rdVal;

//     reg [4:0]  EX_MEM_ctrl, EX_MEM_rd;
//     reg [63:0] EX_MEM_ALU, EX_MEM_B;
//     reg        EX_MEM_memWrite, EX_MEM_regWrite, EX_MEM_changePC;
//     reg [31:0] EX_MEM_addr;
//     reg [63:0] EX_MEM_wrData, EX_MEM_target;  // **EX_MEM_target now driven by ALU updated_next only**

//     reg [4:0]  MEM_WB_ctrl, MEM_WB_rd;
//     reg [63:0] MEM_WB_ALU, MEM_WB_memData;
//     reg        MEM_WB_regWrite, MEM_WB_memToReg;

//     wire [31:0] inst;
//     wire [63:0] mem_rdata;
//     memory memory(
//         .pc              (PC),
//         .clk             (clk),
//         .reset           (reset),
//         .mem_write_enable(EX_MEM_memWrite),
//         .rw_val          (EX_MEM_wrData),
//         .rw_addr         (EX_MEM_addr),
//         .instruction     (inst),
//         .r_out           (mem_rdata)
//     );

//     wire [63:0] regOut1, regOut2, rdVal, r31Val;
//     register_file reg_file(
//         .clk         (clk),
//         .reset       (reset),
//         .write_enable(MEM_WB_regWrite),
//         .dataInput   (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU),
//         .readAddress1(IF_rs),
//         .readAddress2(IF_rt),
//         .readAddress3(IF_rd),
//         .writeAddress(MEM_WB_rd),
//         .lPassed     (~IF_rtPassed),
//         .L           (IF_L),
//         .value1      (regOut1),
//         .value2      (regOut2),
//         .rdVal       (rdVal),
//         .r31_val     (r31Val)
//     );

//     wire [63:0] aluOp1 = (ID_EX_ctrl == 5'b11001 || ID_EX_ctrl == 5'b11011)
//         ? ID_EX_rdVal
//         : (EX_MEM_regWrite && EX_MEM_rd != 0 && EX_MEM_rd == ID_EX_rs)
//             ? EX_MEM_ALU
//             : (MEM_WB_regWrite && MEM_WB_rd != 0 && MEM_WB_rd == ID_EX_rs)
//                 ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU)
//                 : ID_EX_A;

//     wire [63:0] literal = {{52{ID_EX_L[11]}}, ID_EX_L};
//     wire [63:0] aluOp2 = ID_EX_rtPassed
//       ? ( (EX_MEM_regWrite && EX_MEM_rd != 0 && EX_MEM_rd == ID_EX_rt)
//             ? EX_MEM_ALU
//             : (MEM_WB_regWrite && MEM_WB_rd != 0 && MEM_WB_rd == ID_EX_rt)
//                 ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU)
//                 : ID_EX_B )
//       : literal;



//     wire [63:0] aluResult, aluUpdatedNext;
//     wire        aluRegWrite, aluMemWrite, aluChangePC;
//     wire [31:0] aluAddr;
//     wire [63:0] aluWrData;
//     ALU ALU_INST(
//         .pc               (ID_EX_PC),
//         .rdVal            (ID_EX_rdVal),
//         .operand1         (aluOp1),
//         .operand2         (aluOp2),
//         .opcode           (ID_EX_ctrl),
//         .r_out            (mem_rdata),
//         .r31_val          (ID_EX_r31),
//         .result           (aluResult),
//         .writeEnable      (aluRegWrite),
//         .mem_write_enable (aluMemWrite),
//         .rw_addr          (aluAddr),
//         .rw_val           (aluWrData),
//         .updated_next     (aluUpdatedNext),
//         .changing_pc      (aluChangePC)
//     );

//     // IF stage
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             PC        <= 64'h2000;
//             stall_cnt <= 5;
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 0;
//         end else if (stall_cnt != 0) begin
//             stall_cnt <= stall_cnt - 1;
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 0;
//         end else if (EX_MEM_changePC) begin
//             PC        <= EX_MEM_target;
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 0;
//         end else if (load_use_hazard) begin
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 32'h00000000;  // NOP
//         end else begin
//             PC        <= PC + 4;
//             IF_ID_PC  <= PC;
//             IF_ID_IR  <= inst;
//         end
//     end

//     // ID stage
//     always @(posedge clk or posedge reset) begin
//         if (reset || EX_MEM_changePC || stall_cnt != 0) begin
//             ID_EX_ctrl     <= 0;
//             ID_EX_rd       <= 0;
//             ID_EX_rs       <= 0;
//             ID_EX_rt       <= 0;
//             ID_EX_L        <= 0;
//             ID_EX_rtPassed <= 0;
//             ID_EX_A        <= 0;
//             ID_EX_B        <= 0;
//             ID_EX_PC       <= 0;
//             ID_EX_r31      <= 0;
//             ID_EX_rdVal    <= 0;
//         end else begin
//             ID_EX_ctrl     <= IF_ctrl;
//             ID_EX_rd       <= IF_rd;
//             ID_EX_rs       <= IF_rs;
//             ID_EX_rt       <= IF_rt;
//             ID_EX_L        <= IF_L;
//             ID_EX_rtPassed <= IF_rtPassed;
//             ID_EX_A        <= regOut1;
//             ID_EX_B        <= regOut2;
//             ID_EX_PC       <= IF_ID_PC;
//             ID_EX_r31      <= r31Val;
//             ID_EX_rdVal    <= rdVal;
//         end
//     end

//     // EX stage
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             EX_MEM_ctrl     <= 0;
//             EX_MEM_rd       <= 0;
//             EX_MEM_ALU      <= 0;
//             EX_MEM_B        <= 0;
//             EX_MEM_memWrite <= 0;
//             EX_MEM_regWrite <= 0;
//             EX_MEM_addr     <= 0;
//             EX_MEM_wrData   <= 0;
//             EX_MEM_changePC <= 0;
//             EX_MEM_target   <= 0;
//         end else begin
//             EX_MEM_ctrl     <= ID_EX_ctrl;
//             EX_MEM_rd       <= ID_EX_rd;
//             EX_MEM_ALU      <= aluResult;
//             EX_MEM_B        <= ID_EX_B;
//             EX_MEM_memWrite <= aluMemWrite;
//             EX_MEM_regWrite <= aluRegWrite;
//             EX_MEM_addr     <= aluAddr;
//             EX_MEM_wrData   <= aluWrData;
//             EX_MEM_changePC <= aluChangePC;
//             EX_MEM_target   <= aluUpdatedNext;    
//         end
//     end

//     // MEM stage
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             MEM_WB_ctrl     <= 0;
//             MEM_WB_rd       <= 0;
//             MEM_WB_ALU      <= 0;
//             MEM_WB_memData  <= 0;
//             MEM_WB_regWrite <= 0;
//             MEM_WB_memToReg <= 0;
//         end else begin
//             MEM_WB_ctrl     <= EX_MEM_ctrl;
//             MEM_WB_rd       <= EX_MEM_rd;
//             MEM_WB_ALU      <= EX_MEM_ALU;
//             MEM_WB_memData  <= mem_rdata;
//             MEM_WB_regWrite <= EX_MEM_regWrite;
//             MEM_WB_memToReg <= (EX_MEM_ctrl == 5'b10000);
//         end
//     end

//     // // HALT logic
//     // reg halt_flag;
//     // always @(posedge clk or posedge reset) begin
//     //     if (reset)
//     //         halt_flag <= 0;
//     //     else if (MEM_WB_ctrl == 5'h0f)
//     //         halt_flag <= 1;
//     // end
//     // assign hlt = halt_flag;
//     // Modified HALT logic with pipeline draining
//     reg halt_detected;
//     reg [2:0] drain_counter;
//     reg halt_flag;

//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             halt_detected <= 0;
//             drain_counter <= 0;
//             halt_flag <= 0;
//         end else if (ID_EX_ctrl == 5'h0f && !halt_detected) begin
//             // Detect halt instruction in ID/EX stage
//             halt_detected <= 1;
//             drain_counter <= 3'd4; // Need 4 cycles to drain the pipeline
//         end else if (halt_detected && drain_counter > 0) begin
//             // Count down until pipeline is drained
//             drain_counter <= drain_counter - 1;
//         end else if (halt_detected && drain_counter == 0) begin
//             // All instructions have completed, now set the halt flag
//             halt_flag <= 1;
//         end
//     end

//     // Modify the IF stage to stop fetching new instructions when halt is detected
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             PC        <= 64'h2000;
//             stall_cnt <= 5;
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 0;
//         end else if (stall_cnt != 0) begin
//             stall_cnt <= stall_cnt - 1;
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 0;
//         end else if (halt_detected && !halt_flag) begin
//             // When halt is detected but not yet flagged, stop fetching new instructions
//             // but allow pipeline to continue processing existing instructions
//             IF_ID_PC  <= IF_ID_PC;
//             IF_ID_IR  <= IF_ID_IR;
//         end else if (EX_MEM_changePC) begin
//             PC        <= EX_MEM_target;
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 0;
//         end else if (load_use_hazard) begin
//             IF_ID_PC  <= 0;
//             IF_ID_IR  <= 32'h00000000;  // NOP
//         end else begin
//             PC        <= PC + 4;
//             IF_ID_PC  <= PC;
//             IF_ID_IR  <= inst;
//         end
//     end

//     assign hlt = halt_flag;

// endmodule


module tinker_core(
    input  wire        clk,
    input  wire        reset,
    output wire        hlt
);
    //------------------------------------------------------------------------------
    // Program Counter and initial stall to fill pipeline
    //------------------------------------------------------------------------------
    reg [63:0] PC;
    reg [1:0]  stall_cnt;

    //------------------------------------------------------------------------------
    // IF/ID Pipeline Registers
    //------------------------------------------------------------------------------
    reg [63:0] IF_ID_PC;
    reg [31:0] IF_ID_IR;

    //------------------------------------------------------------------------------
    // Instruction Decode (IF stage outputs)
    //------------------------------------------------------------------------------
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

    //------------------------------------------------------------------------------
    // Load-Use Hazard Detection
    //------------------------------------------------------------------------------
    wire load_use_hazard = (EX_MEM_ctrl == 5'b10000) && (
        (IF_ctrl != 5'b00000 && EX_MEM_rd == IF_rs) ||
        (IF_ctrl != 5'b00000 && EX_MEM_rd == IF_rt && IF_rtPassed)
    );

    //------------------------------------------------------------------------------
    // ID/EX Pipeline Registers
    //------------------------------------------------------------------------------
    reg [63:0] ID_EX_PC;
    reg [4:0]  ID_EX_ctrl, ID_EX_rd, ID_EX_rs, ID_EX_rt;
    reg [11:0] ID_EX_L;
    reg        ID_EX_rtPassed;
    reg [63:0] ID_EX_A, ID_EX_B;
    reg [63:0] ID_EX_r31, ID_EX_rdVal;

    //------------------------------------------------------------------------------
    // EX/MEM Pipeline Registers
    //------------------------------------------------------------------------------
    reg [4:0]  EX_MEM_ctrl, EX_MEM_rd;
    reg [63:0] EX_MEM_ALU, EX_MEM_B;
    reg        EX_MEM_memWrite, EX_MEM_regWrite, EX_MEM_changePC;
    reg [31:0] EX_MEM_addr;
    reg [63:0] EX_MEM_wrData, EX_MEM_target;

    //------------------------------------------------------------------------------
    // MEM/WB Pipeline Registers
    //------------------------------------------------------------------------------
    reg [4:0]  MEM_WB_ctrl, MEM_WB_rd;
    reg [63:0] MEM_WB_ALU, MEM_WB_memData;
    reg        MEM_WB_regWrite, MEM_WB_memToReg;

    //------------------------------------------------------------------------------
    // Memory and Register File Instances
    //------------------------------------------------------------------------------
    wire [31:0] inst;
    wire [63:0] mem_rdata;
    memory memory(
        .pc               (PC),
        .clk              (clk),
        .reset            (reset),
        .mem_write_enable (EX_MEM_memWrite),
        .rw_val           (EX_MEM_wrData),
        .rw_addr          (EX_MEM_addr),
        .instruction      (inst),
        .r_out            (mem_rdata)
    );

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

    //------------------------------------------------------------------------------
    // Forwarding Logic (EX->EX then MEM->EX priorities)
    //------------------------------------------------------------------------------
    // RS forwarding
    wire [63:0] ex_forward_A =
        (EX_MEM_regWrite && EX_MEM_rd != 0 && EX_MEM_rd == ID_EX_rs)
        ? EX_MEM_ALU : ID_EX_A;
    wire [63:0] mem_forward_A =
        (MEM_WB_regWrite && MEM_WB_rd != 0 && MEM_WB_rd == ID_EX_rs)
        ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU)
        : ex_forward_A;
    wire [63:0] aluOp1 =
        (ID_EX_ctrl == 5'b11001 || ID_EX_ctrl == 5'b11011)
        ? ID_EX_rdVal : mem_forward_A;

    // RT forwarding
    wire [63:0] ex_forward_B =
        (EX_MEM_regWrite && EX_MEM_rd != 0 && EX_MEM_rd == ID_EX_rt && ID_EX_rtPassed)
        ? EX_MEM_ALU : ID_EX_B;
    wire [63:0] mem_forward_B =
        (MEM_WB_regWrite && MEM_WB_rd != 0 && MEM_WB_rd == ID_EX_rt && ID_EX_rtPassed)
        ? (MEM_WB_memToReg ? MEM_WB_memData : MEM_WB_ALU)
        : ex_forward_B;
    wire [63:0] aluOp2 = ID_EX_rtPassed
        ? mem_forward_B
        : {{52{ID_EX_L[11]}}, ID_EX_L};

    //------------------------------------------------------------------------------
    // ALU Instance (combinational execute)
    //------------------------------------------------------------------------------
    wire [63:0] aluResult, aluUpdatedNext;
    wire        aluRegWrite, aluMemWrite, aluChangePC;
    wire [31:0] aluAddr;
    wire [63:0] aluWrData;
    ALU ALU_INST(
        .pc               (ID_EX_PC),
        .rdVal            (ID_EX_rdVal),
        .operand1         (aluOp1),
        .operand2         (aluOp2),
        .opcode           (ID_EX_ctrl),
        .r_out            (mem_rdata),
        .r31_val          (ID_EX_r31),
        .result           (aluResult),
        .writeEnable      (aluRegWrite),
        .mem_write_enable (aluMemWrite),
        .rw_addr          (aluAddr),
        .rw_val           (aluWrData),
        .updated_next     (aluUpdatedNext),
        .changing_pc      (aluChangePC)
    );

    //------------------------------------------------------------------------------
    // IF Stage: Fetch and simple flush/bubble logic
    //------------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC        <= 64'h2000;
            stall_cnt <= 5;
            IF_ID_PC  <= 0;
            IF_ID_IR  <= 0;
        end else if (stall_cnt != 0) begin
            stall_cnt <= stall_cnt - 1;
            IF_ID_PC  <= 0;
            IF_ID_IR  <= 0;
        end else if (EX_MEM_changePC) begin
            PC        <= EX_MEM_target;
            IF_ID_PC  <= 0;
            IF_ID_IR  <= 0;
        end else if (load_use_hazard) begin
            IF_ID_PC  <= 0;
            IF_ID_IR  <= 32'h00000000;  // NOP
        end else begin
            PC        <= PC + 4;
            IF_ID_PC  <= PC;
            IF_ID_IR  <= inst;
        end
    end

    //------------------------------------------------------------------------------
    // ID Stage: Latch decode into ID/EX regs
    //------------------------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset || EX_MEM_changePC || stall_cnt != 0) begin
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

    //------------------------------------------------------------------------------
    // EX Stage: Latch ALU results and control signals
    //------------------------------------------------------------------------------
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

    //------------------------------------------------------------------------------
    // MEM Stage: Latch memory outputs into MEM/WB regs
    //------------------------------------------------------------------------------
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

    //------------------------------------------------------------------------------
    // HALT Logic: Assert hlt when HALT opcode reaches WB
    //------------------------------------------------------------------------------
    reg halt_flag;
    always @(posedge clk or posedge reset) begin
        if (reset)
            halt_flag <= 0;
        else if (MEM_WB_ctrl == 5'b01111)  // HALT opcode = 0x0F
            halt_flag <= 1;
    end
    assign hlt = halt_flag;
endmodule
