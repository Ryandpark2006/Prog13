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

/* tinker‑core — refactored with pc / if_id / id_ex / ex_mem style */
module tinker_core (
    input  wire clk,
    input  wire reset,
    output wire hlt
);

/* 0. fetch */
reg  [63:0] pc;
wire [63:0] pc_plus4 = pc + 64'd4;

/* hazard flags */
wire stall_if;
wire flush_if;

/* 1. if→id */
reg [31:0] if_id_ir;
reg [63:0] if_id_pc4;

/* 2. id→ex */
reg [4:0]  id_ex_op , id_ex_rd , id_ex_rs , id_ex_rt;
reg [63:0] id_ex_a  , id_ex_b  , id_ex_c;
reg [63:0] id_ex_sext, id_ex_pc4, id_ex_sp;

/* 3. ex→mem */
reg [4:0]  ex_mem_rd;
reg [63:0] ex_mem_alu , ex_mem_addr , ex_mem_pctarget;
reg        ex_mem_br  , ex_mem_regw , ex_mem_memw , ex_mem_hlt;

/* 4. mem→wb */
reg [4:0]  mem_wb_rd;
reg [63:0] mem_wb_val;
reg        mem_wb_regw , mem_wb_hlt;

/* memory */
wire [31:0] imem_out;
wire [63:0] dmem_out;

memory mem (
    .clk(clk),
    .programCounter(pc),
    .dataAddress(ex_mem_addr),
    .writeEnable(ex_mem_memw),
    .writeData(ex_mem_alu),
    .readInstruction(imem_out),
    .readData(dmem_out)
);

/* decode */
wire [4:0] op, rd, rs, rt;
wire [11:0] lit;
decoder dec (
    .instruction(if_id_ir),
    .opcode(op), .rd(rd), .rs(rs), .rt(rt), .l(lit)
);

/* register file */
wire [63:0] reg_a, reg_b, reg_c, reg_sp;
registers regs (
    .clk(clk), .reset(reset),
    .write(mem_wb_regw),
    .data_input(mem_wb_val),
    .registerOne(rs), .registerTwo(rt), .registerThree(rd),
    .writeAddress(mem_wb_rd),
    .data_outputOne(reg_a), .data_outputTwo(reg_b),
    .data_outputThree(reg_c), .stackPointer(reg_sp)
);

/* sign‑extend */
wire [63:0] sext = {{52{lit[11]}}, lit};

/* hazard detect */
wire idex_regw = (id_ex_op != 5'h1f) && (id_ex_op < 5'h08 || id_ex_op > 5'h0f) && (id_ex_op != 5'h13);
wire raw_idex = idex_regw   && ((id_ex_rd==rs)||(id_ex_rd==rt)||(id_ex_rd==rd));
wire raw_ex   = ex_mem_regw && ((ex_mem_rd==rs)||(ex_mem_rd==rt)||(ex_mem_rd==rd));
wire raw_mem  = mem_wb_regw && ((mem_wb_rd==rs)||(mem_wb_rd==rt)||(mem_wb_rd==rd));
assign stall_if = raw_idex | raw_ex | raw_mem;

/* bubble mux */
localparam [4:0] NOP = 5'h1f;
wire bubble = stall_if | flush_if;
wire [31:0] if_id_ir_n  = bubble ? 32'd0 : imem_out;
wire [63:0] if_id_pc4_n = bubble ? 64'd0 : pc_plus4;

/* alu */
wire [63:0] alu_y, alu_addr, alu_pc_out;
wire        alu_taken, alu_regw, alu_memw, alu_hlt;
alu alu_inst (
    .opcode(id_ex_op),
    .inputDataOne(id_ex_a), .inputDataTwo(id_ex_b), .inputDataThree(id_ex_c),
    .signExtendedLiteral(id_ex_sext),
    .programCounter(id_ex_pc4 - 64'd4),
    .stackPointer(id_ex_sp),
    .readMemory(dmem_out),
    .result(alu_y), .readWriteAddress(alu_addr),
    .newProgramCounter(alu_pc_out),
    .branchTaken(alu_taken),
    .writeToRegister(alu_regw),
    .writeToMemory(alu_memw),
    .hlt(alu_hlt)
);

assign flush_if = alu_taken;

/* sequential logic */
always @(posedge clk or posedge reset) begin
    if (reset) begin
        pc <= 64'h2000;
        if_id_ir <= 0; if_id_pc4 <= 0;
        id_ex_op <= NOP; id_ex_rd<=0; id_ex_rs<=0; id_ex_rt<=0;
        id_ex_a<=0; id_ex_b<=0; id_ex_c<=0; id_ex_sext<=0; id_ex_pc4<=0; id_ex_sp<=0;
        ex_mem_rd<=0; ex_mem_alu<=0; ex_mem_addr<=0; ex_mem_pctarget<=0;
        ex_mem_br<=0; ex_mem_regw<=0; ex_mem_memw<=0; ex_mem_hlt<=0;
        mem_wb_rd<=0; mem_wb_val<=0; mem_wb_regw<=0; mem_wb_hlt<=0;
    end else begin
        /* pc */
        if (alu_taken)      pc <= alu_pc_out;
        else if (!stall_if) pc <= pc_plus4;

        /* if→id */
        if_id_ir  <= if_id_ir_n;
        if_id_pc4 <= if_id_pc4_n;

        /* id→ex */
        id_ex_op   <= bubble ? NOP : op;
        id_ex_rd   <= bubble ? 5'd0 : rd;
        id_ex_rs   <= bubble ? 5'd0 : rs;
        id_ex_rt   <= bubble ? 5'd0 : rt;
        id_ex_a    <= bubble ? 0 : reg_a;
        id_ex_b    <= bubble ? 0 : reg_b;
        id_ex_c    <= bubble ? 0 : reg_c;
        id_ex_sext <= bubble ? 0 : sext;
        id_ex_pc4  <= bubble ? 0 : if_id_pc4;
        id_ex_sp   <= bubble ? 0 : reg_sp;

        /* ex→mem */
        ex_mem_rd       <= id_ex_rd;
        ex_mem_alu      <= alu_y;
        ex_mem_addr     <= alu_addr;
        ex_mem_pctarget <= alu_pc_out;
        ex_mem_br       <= alu_taken;
        ex_mem_regw     <= alu_regw;
        ex_mem_memw     <= alu_memw;
        ex_mem_hlt      <= alu_hlt;

        /* mem→wb */
        mem_wb_rd   <= ex_mem_rd;
        mem_wb_val  <= ex_mem_alu;
        mem_wb_regw <= ex_mem_regw;
        mem_wb_hlt  <= ex_mem_hlt;
    end
end

/* halt */
assign hlt = mem_wb_hlt;

endmodule

