`timescale 1ns/1ps
module tinker_tb;
    reg         clk;
    reg         reset;
    integer     i;

    wire        hlt;

    // Instantiate the processor core.
    tinker_core cpu (
        .clk(clk),
        .reset(reset),
        .hlt(hlt)
    );

    // Clock generation: period = 2 time units.
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end

    //----------------------------------------------------------------
    // Task: zero all regs 0–15, then preset regs 16/17 for R‑type
    // and 16 for I‑type tests.
    //----------------------------------------------------------------
    task init_regs_r; input [63:0] vrs, vrt; begin
        // clear 0–15
        for (i = 0; i < 16; i = i + 1)
            cpu.reg_file.registers[i] = 64'd0;
        // preset two sources
        cpu.reg_file.registers[16] = vrs;
        cpu.reg_file.registers[17] = vrt;
        // the rest can stay at zero / default
        cpu.reg_file.registers[31] = 64'h80000;
    end endtask

    task init_regs_i; input [63:0] vrs; begin
        // clear 0–15
        for (i = 0; i < 16; i = i + 1)
            cpu.reg_file.registers[i] = 64'd0;
        // preset one source
        cpu.reg_file.registers[16] = vrs;
        cpu.reg_file.registers[31] = 64'h80000;
    end endtask

    //----------------------------------------------------------------
    // Task: poke a 32‑bit word into little‑endian memory at 8192
    //----------------------------------------------------------------
    task write_inst(input [31:0] inst); begin
        // main instruction
        { cpu.memory.bytes[8195],
          cpu.memory.bytes[8194],
          cpu.memory.bytes[8193],
          cpu.memory.bytes[8192] } = inst;
        // NOP right after (just to flush)
        { cpu.memory.bytes[8199],
          cpu.memory.bytes[8198],
          cpu.memory.bytes[8197],
          cpu.memory.bytes[8196] } = 32'h11000000;
    end endtask

    //----------------------------------------------------------------
    // Task: run one test
    //----------------------------------------------------------------
    task run_r_test(
        input [8*16:1] name,
        input [4:0]    opcode,
        input [63:0]   vs,    // reg[16]
        input [63:0]   vt,    // reg[17]
        input [63:0]   expect // expected in reg[0]
    );
        reg [31:0] inst;
        begin
            // assert reset
            reset = 1;
            #10;
            // init regs
            init_regs_r(vs, vt);
            // build R‑type: {op, rd=0, rs=16, rt=17, L=0}
            inst = { opcode, 5'd0, 5'd16, 5'd17, 12'd0 };
            write_inst(inst);
            // deassert reset → start
            reset = 0;
            // wait 5 stages + some slop
            #50;
            // check result
            $display("%0s → got %0d, expect %0d",
                     name,
                     cpu.reg_file.registers[0],
                     expect);
            if (cpu.reg_file.registers[0] !== expect)
                $display("  ** FAIL ** %0s", name);
            else
                $display("  PASS");
            $display("");
        end
    endtask

    task run_i_test(
        input [8*16:1] name,
        input [4:0]    opcode,
        input [63:0]   vs,      // reg[16]
        input [11:0]   imm,     // literal
        input [63:0]   expect   // expected in reg[0]
    );
        reg [31:0] inst;
        reg [63:0] sxt;
        begin
            reset = 1;
            #10;
            init_regs_i(vs);
            // sign‑extend literal
            sxt = {{52{imm[11]}}, imm};
            // build I‑type: {op, rd=0, rs=16, rt=imm‑bits, L=imm}
            // here we treat rt field as L for addi/subi etc.
            inst = { opcode, 5'd0, 5'd16, 5'd0, imm };
            write_inst(inst);
            reset = 0;
            #50;
            $display("%0s → got %0d, expect %0d",
                     name,
                     cpu.reg_file.registers[0],
                     expect);
            if (cpu.reg_file.registers[0] !== expect)
                $display("  ** FAIL ** %0s", name);
            else
                $display("  PASS");
            $display("");
        end
    endtask

    //----------------------------------------------------------------
    // Main sequence
    //----------------------------------------------------------------
    initial begin
        $dumpfile("tinker_tb.vcd");
        $dumpvars(0, tinker_tb);

        // R‑type arithmetic & logic
        run_r_test("ADD   (16+17)", 5'b11000, 247, 162, 247+162);
        run_r_test("SUB   (16-17)", 5'b11010, 100,  40, 100-40);
        run_r_test("AND   (16&17)", 5'b00000, 15,   9,  15&9);
        run_r_test("OR    (16|17)", 5'b00001, 15,   9,  15|9);
        run_r_test("XOR   (16^17)", 5'b00010, 15,   9,  15^9);
        run_r_test("NOT   (~16)",   5'b00011, 7,    0, ~7);

        // R‑type multiply/divide
        run_r_test("MUL   (2*3)",   5'b11100, 2,    3,   2*3);
        run_r_test("DIV   (14/5)",  5'b11101, 14,   5,   14/5);

        // I‑type addi/subi
        run_i_test("ADDI  (16+5)",  5'b11001, 20,    5,   20+5);
        run_i_test("SUBI  (16-3)",  5'b11011, 20,    3,   20-3);

        $finish;
    end

endmodule
