module alu (
    input  [4:0]  op,
    input  [63:0] src_a,
    input  [63:0] src_b,
    input  [63:0] src_c,
    input  [63:0] imm_ext,
    input  [63:0] pc_in,
    input  [63:0] sp_in,
    input  [63:0] mem_in,
    output reg [63:0] alu_out,
    output reg [63:0] addr_out,
    output reg [63:0] pc_next,
    output reg       take_branch,
    output reg       reg_we,
    output reg       mem_we,
    output reg       halt_flag
);
    always @(*) begin
        alu_out      = 64'd0;
        addr_out     = 64'd0;
        pc_next      = pc_in + 64'd4;
        reg_we       = 1'b1;
        mem_we       = 1'b0;
        take_branch  = 1'b0;
        halt_flag    = 1'b0;

        case (op)
            5'h00: alu_out = src_a & src_b;                                 // and
            5'h01: alu_out = src_a | src_b;                                 // or
            5'h02: alu_out = src_a ^ src_b;                                 // xor
            5'h03: alu_out = ~src_a;                                        // not
            5'h04: alu_out = src_a >> src_b[5:0];                           // shftr
            5'h05: alu_out = src_c >> imm_ext;                              // shftri
            5'h06: alu_out = src_a << src_b[5:0];                           // shftl
            5'h07: alu_out = src_c << imm_ext;                              // shftli

            5'h08: begin                                                    // br
                reg_we      = 0;
                take_branch = 1;
                pc_next     = src_c;
            end
            5'h09: begin                                                    // brr rd
                reg_we      = 0;
                take_branch = 1;
                pc_next     = pc_in + src_c;
            end
            5'h0a: begin                                                    // brr L
                reg_we      = 0;
                take_branch = 1;
                pc_next     = pc_in + imm_ext;
            end
            5'h0b: begin                                                    // brnz
                reg_we = 0;
                if (src_a != 0) begin
                    take_branch = 1;
                    pc_next     = src_c;
                end
            end
            5'h0c: begin                                                    // call
                reg_we      = 0;
                mem_we      = 1;
                take_branch = 1;
                pc_next     = src_c;
                addr_out    = sp_in - 64'd8;
                alu_out     = pc_in + 64'd4;
            end
            5'h0d: begin                                                    // return
                reg_we      = 0;
                take_branch = 1;
                addr_out    = sp_in - 64'd8;
                pc_next     = mem_in;
            end
            5'h0e: begin                                                    // brgt
                reg_we = 0;
                if ($signed(src_a) > $signed(src_b)) begin
                    take_branch = 1;
                    pc_next     = src_c;
                end
            end
            5'h0f: begin                                                    // halt/priv
                reg_we = 0;
                if (imm_ext[3:0] == 4'h0) halt_flag = 1;
            end
            5'h10: begin                                                    // load
                addr_out = src_a + imm_ext;
                alu_out  = mem_in;
            end
            5'h11: alu_out = src_a;                                         // mov reg‑reg
            5'h12: alu_out = {src_c[63:12], imm_ext[11:0]};                 // mov_L_to_reg
            5'h13: begin                                                    // store
                reg_we    = 0;
                mem_we    = 1;
                addr_out  = src_c + imm_ext;
                alu_out   = src_a;
            end
            5'h14: alu_out = $realtobits($bitstoreal(src_a) + $bitstoreal(src_b)); // addf
            5'h15: alu_out = $realtobits($bitstoreal(src_a) - $bitstoreal(src_b)); // subf
            5'h16: alu_out = $realtobits($bitstoreal(src_a) * $bitstoreal(src_b)); // mulf
            5'h17: alu_out = $realtobits($bitstoreal(src_a) / $bitstoreal(src_b)); // divf
            5'h18: alu_out = src_a + src_b;                                 // add
            5'h19: alu_out = src_c + imm_ext;                               // addi
            5'h1a: alu_out = src_a - src_b;                                 // sub
            5'h1b: alu_out = src_c - imm_ext;                               // subi
            5'h1c: alu_out = src_a * src_b;                                 // mul
            5'h1d: alu_out = (src_b == 0) ? 64'd0 : (src_a / src_b);        // div
            default: begin
                reg_we      = 0;
                mem_we      = 0;
                take_branch = 0;
            end
        endcase
    end
endmodule

module register_file (
    input         clk,
    input         reset,
    input         we,
    input  [63:0] data_in,
    input  [4:0]  ra1,
    input  [4:0]  ra2,
    input  [4:0]  ra3,
    input  [4:0]  wa,
    output [63:0] rd1,
    output [63:0] rd2,
    output [63:0] rd3,
    output [63:0] sp_val
);
    reg [63:0] registers [0:31];

    assign rd1    = registers[ra1];
    assign rd2    = registers[ra2];
    assign rd3    = registers[ra3];
    assign sp_val = registers[31];

    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) registers[i] <= 64'd0;
            registers[31] <= 64'd524288;
        end else if (we) begin
            registers[wa] <= data_in;
        end
    end
endmodule

module memory (
    input         clk,
    input  [63:0] pc_in,
    input  [63:0] addr_in,
    input         we,
    input  [63:0] data_in,
    output [31:0] instr_out,
    output [63:0] data_out
);
    reg [7:0] bytes [0:524287];

    assign instr_out = {bytes[pc_in+3], bytes[pc_in+2],
                        bytes[pc_in+1], bytes[pc_in]};

    assign data_out  = {bytes[addr_in+7], bytes[addr_in+6],
                        bytes[addr_in+5], bytes[addr_in+4],
                        bytes[addr_in+3], bytes[addr_in+2],
                        bytes[addr_in+1], bytes[addr_in]};

    always @(posedge clk) begin
        if (we) begin
            {bytes[addr_in+7], bytes[addr_in+6], bytes[addr_in+5],
             bytes[addr_in+4], bytes[addr_in+3], bytes[addr_in+2],
             bytes[addr_in+1], bytes[addr_in]} <= data_in;
        end
    end
endmodule

module decoder (
    input  [31:0] instr,
    output [4:0]  op,
    output [4:0]  dest,
    output [4:0]  src1,
    output [4:0]  src2,
    output [11:0] lit
);
    assign op   = instr[31:27];
    assign dest = instr[26:22];
    assign src1 = instr[21:17];
    assign src2 = instr[16:12];
    assign lit  = instr[11:0];
endmodule


// ================================================================
// tinker_core — corrected forwarding & 1‑cycle load‑use stall
// ================================================================
module tinker_core (
    input  wire clk,
    input  wire reset,
    output wire hlt
);

/* 0. pc & fetch ------------------------------------------------ */
reg  [63:0] pc_reg;
wire [63:0] pc_plus4 = pc_reg + 64'd4;

/* pipe‑flush on taken branch */
wire flush_if;

/* 1. IF / ID ---------------------------------------------------*/
reg [31:0] if_id_ir;
reg [63:0] if_id_pc4;

/* 2. ID / EX ---------------------------------------------------*/
reg [4:0]  id_ex_op, id_ex_rd, id_ex_rs, id_ex_rt;
reg [63:0] id_ex_a,  id_ex_b,  id_ex_c;
reg [63:0] id_ex_imm, id_ex_pc4, id_ex_sp;

/* 3. EX / MEM --------------------------------------------------*/
reg [4:0]  ex_mem_op, ex_mem_rd;
reg [63:0] ex_mem_res, ex_mem_addr, ex_mem_npc;
reg        ex_mem_br, ex_mem_we_reg, ex_mem_we_mem, ex_mem_hlt;

/* 4. MEM / WB --------------------------------------------------*/
reg [4:0]  mem_wb_rd;
reg [63:0] mem_wb_res;
reg        mem_wb_we_reg, mem_wb_hlt;

assign hlt = mem_wb_hlt;

/* memory -------------------------------------------------------*/
wire [31:0] instr_fetch;
wire [63:0] mem_rdata;

memory memory (
    .clk     (clk),
    .pc_in   (pc_reg),
    .addr_in (alu_addr),
    .we      (alu_we_mem),
    .data_in (alu_out),
    .instr_out(instr_fetch),
    .data_out(mem_rdata)
);

/* decode -------------------------------------------------------*/
wire [4:0] dec_op, dec_rd, dec_rs, dec_rt;
wire [11:0] dec_lit;

decoder dec_unit (
    .instr(if_id_ir),
    .op(dec_op), .dest(dec_rd), .src1(dec_rs), .src2(dec_rt), .lit(dec_lit)
);

wire [63:0] imm_ext = {{52{dec_lit[11]}}, dec_lit};

/* register file -----------------------------------------------*/
wire [63:0] rf_a, rf_b, rf_c, sp_val;

register_file reg_file (
    .clk(clk), .reset(reset),
    .we     (mem_wb_we_reg),
    .data_in(mem_wb_res),
    .ra1(dec_rs), .ra2(dec_rt), .ra3(dec_rd),
    .wa(mem_wb_rd),
    .rd1(rf_a), .rd2(rf_b), .rd3(rf_c), .sp_val(sp_val)
);

/* ---------- forwarding (to EX stage) --------------------------*/
/* does the instruction currently in EX write a destination? */
wire id_ex_we_reg = (id_ex_op != 5'h1f) && (id_ex_op != 5'h08) &&
                    (id_ex_op != 5'h09) && (id_ex_op != 5'h0a) &&
                    (id_ex_op != 5'h0b) && (id_ex_op != 5'h0c) &&
                    (id_ex_op != 5'h0d) && (id_ex_op != 5'h0e) &&
                    (id_ex_op != 5'h0f) && (id_ex_op != 5'h13);

/* does the instruction in MEM load from memory? */
wire ex_mem_is_load = (ex_mem_op == 5'h10);

/* one‑cycle stall if load‑use hazard */
wire load_use_hazard = ex_mem_is_load &&
                      ((ex_mem_rd == dec_rs) ||
                       (ex_mem_rd == dec_rt) ||
                       (ex_mem_rd == dec_rd));

wire stall_if = load_use_hazard;

/* ----------- forwarding comparators (allow rd==0) -------------*/
wire fwd_ex_rs  = (id_ex_we_reg && (id_ex_rd == dec_rs));
wire fwd_mem_rs = (ex_mem_we_reg && (ex_mem_rd == dec_rs));
wire fwd_wb_rs  = (mem_wb_we_reg && (mem_wb_rd == dec_rs));

wire fwd_ex_rt  = (id_ex_we_reg && (id_ex_rd == dec_rt));
wire fwd_mem_rt = (ex_mem_we_reg && (ex_mem_rd == dec_rt));
wire fwd_wb_rt  = (mem_wb_we_reg && (mem_wb_rd == dec_rt));

wire fwd_ex_rd  = (id_ex_we_reg && (id_ex_rd == dec_rd));
wire fwd_mem_rd = (ex_mem_we_reg && (ex_mem_rd == dec_rd));
wire fwd_wb_rd  = (mem_wb_we_reg && (mem_wb_rd == dec_rd));

wire [63:0] dec_a = fwd_ex_rs ? alu_out  :
                    fwd_mem_rs ? ex_mem_res :
                    fwd_wb_rs  ? mem_wb_res : rf_a;

wire [63:0] dec_b = fwd_ex_rt ? alu_out  :
                    fwd_mem_rt ? ex_mem_res :
                    fwd_wb_rt  ? mem_wb_res : rf_b;

wire [63:0] dec_c = fwd_ex_rd ? alu_out  :
                    fwd_mem_rd ? ex_mem_res :
                    fwd_wb_rd  ? mem_wb_res : rf_c;

/* bubble only on flush ----------------------------------------*/
localparam [4:0] BUBBLE_OP = 5'h1f;
wire bubble = flush_if;

/* next‑state for ID/EX ----------------------------------------*/
wire [4:0]  nxt_op  = bubble ? BUBBLE_OP : dec_op;
wire [4:0]  nxt_rd  = bubble ? 5'd0      : dec_rd;
wire [4:0]  nxt_rs  = bubble ? 5'd0      : dec_rs;
wire [4:0]  nxt_rt  = bubble ? 5'd0      : dec_rt;

wire [63:0] nxt_a   = bubble ? 64'd0 : dec_a;
wire [63:0] nxt_b   = bubble ? 64'd0 : dec_b;
wire [63:0] nxt_c   = bubble ? 64'd0 : dec_c;
wire [63:0] nxt_imm = bubble ? 64'd0 : imm_ext;
wire [63:0] nxt_pc4 = bubble ? 64'd0 : if_id_pc4;
wire [63:0] nxt_sp  = bubble ? 64'd0 : sp_val;

/* execute ------------------------------------------------------*/
wire [63:0] alu_out, alu_addr, alu_npc;
wire        alu_br, alu_we_reg, alu_we_mem, alu_hlt;

alu alu_inst (
    .op(id_ex_op),
    .src_a(id_ex_a), .src_b(id_ex_b), .src_c(id_ex_c),
    .imm_ext(id_ex_imm),
    .pc_in(id_ex_pc4 - 64'd4),
    .sp_in(id_ex_sp),
    .mem_in(mem_rdata),
    .alu_out(alu_out), .addr_out(alu_addr),
    .pc_next(alu_npc),
    .take_branch(alu_br),
    .reg_we(alu_we_reg),
    .mem_we(alu_we_mem),
    .halt_flag(alu_hlt)
);

assign flush_if = alu_br;  // taken branch flushes next IF/ID

/* EX/MEM latches ----------------------------------------------*/
wire [4:0]  nxt_ex_mem_op   = id_ex_op;
wire [4:0]  nxt_ex_mem_rd   = id_ex_rd;
wire [63:0] nxt_ex_mem_res  = alu_out;
wire [63:0] nxt_ex_mem_addr = alu_addr;
wire [63:0] nxt_ex_mem_npc  = alu_npc;
wire        nxt_ex_mem_br   = alu_br;
wire        nxt_ex_mem_we_r = alu_we_reg;
wire        nxt_ex_mem_we_m = alu_we_mem;
wire        nxt_ex_mem_hlt  = alu_hlt;

/* MEM/WB latches ----------------------------------------------*/
wire [4:0]  nxt_mem_wb_rd   = ex_mem_rd;
wire [63:0] nxt_mem_wb_res  = ex_mem_res;
wire        nxt_mem_wb_we_r = ex_mem_we_reg;
wire        nxt_mem_wb_hlt  = ex_mem_hlt;

/* sequential ---------------------------------------------------*/
always @(posedge clk or posedge reset) begin
    if (reset) begin
        pc_reg <= 64'h2000;

        if_id_ir  <= 32'd0;
        if_id_pc4 <= 64'd0;

        id_ex_op  <= BUBBLE_OP; id_ex_rd <= 0; id_ex_rs <= 0; id_ex_rt <= 0;
        id_ex_a   <= 0; id_ex_b <= 0; id_ex_c <= 0;
        id_ex_imm <= 0; id_ex_pc4 <= 0; id_ex_sp <= 0;

        ex_mem_op <= 0; ex_mem_rd <= 0; ex_mem_res <= 0; ex_mem_addr <= 0;
        ex_mem_npc <= 0; ex_mem_br <= 0;
        ex_mem_we_reg <= 0; ex_mem_we_mem <= 0; ex_mem_hlt <= 0;

        mem_wb_rd <= 0; mem_wb_res <= 0; mem_wb_we_reg <= 0; mem_wb_hlt <= 0;

    end else begin
        /* pc */
        if (flush_if)        pc_reg <= alu_npc;
        else if (!stall_if)  pc_reg <= pc_plus4;

        /* IF/ID */
        if (!stall_if) begin
            if_id_ir  <= flush_if ? 32'd0 : instr_fetch;
            if_id_pc4 <= flush_if ? 64'd0 : pc_plus4;
        end

        /* ID/EX */
        if (!stall_if) begin
            id_ex_op   <= nxt_op;
            id_ex_rd   <= nxt_rd;
            id_ex_rs   <= nxt_rs;
            id_ex_rt   <= nxt_rt;
            id_ex_a    <= nxt_a;
            id_ex_b    <= nxt_b;
            id_ex_c    <= nxt_c;
            id_ex_imm  <= nxt_imm;
            id_ex_pc4  <= nxt_pc4;
            id_ex_sp   <= nxt_sp;
        end

        /* EX/MEM */
        ex_mem_op       <= nxt_ex_mem_op;
        ex_mem_rd       <= nxt_ex_mem_rd;
        ex_mem_res      <= nxt_ex_mem_res;
        ex_mem_addr     <= nxt_ex_mem_addr;
        ex_mem_npc      <= nxt_ex_mem_npc;
        ex_mem_br       <= nxt_ex_mem_br;
        ex_mem_we_reg   <= nxt_ex_mem_we_r;
        ex_mem_we_mem   <= nxt_ex_mem_we_m;
        ex_mem_hlt      <= nxt_ex_mem_hlt;

        /* MEM/WB */
        mem_wb_rd      <= nxt_mem_wb_rd;
        mem_wb_res     <= nxt_mem_wb_res;
        mem_wb_we_reg  <= nxt_mem_wb_we_r;
        mem_wb_hlt     <= nxt_mem_wb_hlt;
    end
end

endmodule
