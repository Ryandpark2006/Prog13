module tinker_tb;
    reg clk;
    reg reset;
    reg [31:0] instruction;
    integer i;
    
    wire hlt;
    
    // Instantiate the processor core.
    tinker_core cpu (
        .clk(clk),
        .reset(reset),
        .hlt(hlt)
    );

    // Clock generation: 2 time units period.
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end

    // Task to initialize registers inside the register file.
    task initializeRegisters;
        integer j;
        begin
            // Zero-out registers 0-15.
            for (j = 0; j < 16; j = j + 1) begin
                cpu.reg_file.registers[j] = 64'd0;
            end
            // Initialize registers with specific test values.
            cpu.reg_file.registers[16] = 64'd247;
            cpu.reg_file.registers[17] = 64'd162;
            cpu.reg_file.registers[18] = 64'd61;
            cpu.reg_file.registers[19] = 64'd0;
            cpu.reg_file.registers[20] = 64'd30;
            cpu.reg_file.registers[21] = 64'd2;
            cpu.reg_file.registers[22] = 64'd17;
            cpu.reg_file.registers[23] = 64'd62;
            cpu.reg_file.registers[24] = $realtobits(64'd10);
            cpu.reg_file.registers[25] = $realtobits(64'd70);
            cpu.reg_file.registers[26] = $realtobits(64'd604);
            cpu.reg_file.registers[27] = $realtobits(64'd53);
            cpu.reg_file.registers[28] = $realtobits(64'd900);
            cpu.reg_file.registers[29] = 64'd36;
            cpu.reg_file.registers[30] = 64'd12;
        end
    endtask

    // Main test bench initial block.
    initial begin
        $dumpfile("tinker_tb.vcd");
        $dumpvars(0, tinker_tb);

        // -------------------------------------
        // Test Case 01 - AND Instruction
        // (Opcode 5'h00: rd = rs & rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        // Form instruction: {opcode, rd, rs, rt, literal}
        instruction = {5'h00, 5'd0, 5'd16, 5'd17, 12'd0};
        {cpu.memory.bytes[8195], 
         cpu.memory.bytes[8194], 
         cpu.memory.bytes[8193], 
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[0] == (64'd247 & 64'd162))
            $display("Test Case 01 - AND Instruction: Pass");
        else
            $display("Test Case 01 - AND Instruction: Fail");

        // -------------------------------------
        // Test Case 02 - OR Instruction
        // (Opcode 5'h01: rd = rs | rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h01, 5'd1, 5'd17, 5'd18, 12'd0};
        {cpu.memory.bytes[8195], 
         cpu.memory.bytes[8194], 
         cpu.memory.bytes[8193], 
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[1] == (64'd162 | 64'd61))
            $display("Test Case 02 - OR Instruction: Pass");
        else
            $display("Test Case 02 - OR Instruction: Fail");

        // -------------------------------------
        // Test Case 03 - XOR Instruction
        // (Opcode 5'h02: rd = rs ^ rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h02, 5'd2, 5'd18, 5'd19, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[2] == (64'd61 ^ 64'd0))
            $display("Test Case 03 - XOR Instruction: Pass");
        else
            $display("Test Case 03 - XOR Instruction: Fail");

        // -------------------------------------
        // Test Case 04 - NOT Instruction
        // (Opcode 5'h03: rd = ~rs)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h03, 5'd3, 5'd19, 5'd20, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[3] == ~64'd0)
            $display("Test Case 04 - NOT Instruction: Pass");
        else
            $display("Test Case 04 - NOT Instruction: Fail");

        // -------------------------------------
        // Test Case 05 - Shift Right (Shftr) Instruction
        // (Opcode 5'h04: rd = rs >> rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h04, 5'd4, 5'd20, 5'd21, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[4] == (64'd30 >> 2))
            $display("Test Case 05 - Shftr Instruction: Pass");
        else
            $display("Test Case 05 - Shftr Instruction: Fail");

        // -------------------------------------
        // Test Case 06 - Shift Right Immediate (Shftri) Instruction
        // (Opcode 5'h05: rd = rs >> literal)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h05, 5'd20, 5'd20, 5'd0, 12'd2};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[20] == (64'd30 >> 2))
            $display("Test Case 06 - Shftri Instruction: Pass");
        else
            $display("Test Case 06 - Shftri Instruction: Fail");

        // -------------------------------------
        // Test Case 07 - Shift Left (Shftl) Instruction
        // (Opcode 5'h06: rd = rs << rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h06, 5'd5, 5'd21, 5'd22, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[5] == (64'd2 << 17))
            $display("Test Case 07 - Shftl Instruction: Pass");
        else
            $display("Test Case 07 - Shftl Instruction: Fail");

        // -------------------------------------
        // Test Case 08 - Shift Left Immediate (Shftli) Instruction
        // (Opcode 5'h07: rd = literal << immediate)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h07, 5'd21, 5'd0, 5'd0, 12'd17};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[21] == (64'd2 << 17))
            $display("Test Case 08 - Shftli Instruction: Pass");
        else
            $display("Test Case 08 - Shftli Instruction: Fail");

        // -------------------------------------
        // Test Case 09 - Unconditional Branch (Br) Instruction
        // (Opcode 5'h08: PC = rdVal; here rd is register 30 which is 12)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h08, 5'd30, 5'd0, 5'd0, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.programCounter == 12)
            $display("Test Case 09 - Br Instruction: Pass");
        else
            $display("Test Case 09 - Br Instruction: Fail");

        // -------------------------------------
        // Test Case 10 - Relative Branch with rd (Brr rd) Instruction
        // (Opcode 5'h09: PC = PC + rdVal; if PC was 8192 and rdVal from reg[30] is 12, then PC should be 8204)
        reset = 1;
        #10;
        reset = 0;
        initializeRegisters();
        instruction = {5'h09, 5'd30, 5'd0, 5'd0, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        $display("PC before Brr rd: %d", cpu.programCounter);
        #10;
        $display("PC after Brr rd: %d", cpu.programCounter);
        if (cpu.programCounter == 8204)
            $display("Test Case 10 - Brr rd Instruction: Pass");
        else
            $display("Test Case 10 - Brr rd Instruction: Fail");

        // -------------------------------------
        // Test Case 11 - Relative Branch with Literal (Brr L) Instruction
        // (Opcode 5'h0a: PC = PC + literal; expecting PC = 8192 + 3 = 8195)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h0a, 5'd0, 5'd0, 5'd0, 12'd3};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.programCounter == 8195)
            $display("Test Case 11 - Brr L Instruction: Pass");
        else
            $display("Test Case 11 - Brr L Instruction: Fail");

        // -------------------------------------
        // Test Case 12 - Conditional Branch if Nonzero (Brnz) Instruction
        // (Opcode 5'h0b: if (rs != 0) then PC = rdVal; here rs = reg[21] (2 ≠ 0) and rd (reg[30]) is 12)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h0b, 5'd30, 5'd21, 5'd0, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.programCounter == 12)
            $display("Test Case 12 - Brnz Instruction: Pass");
        else
            $display("Test Case 12 - Brnz Instruction: Fail");

        // -------------------------------------
        // Test Case 13 - Call Instruction
        // (Opcode 5'h0c: store return address and branch; here PC should jump to rdVal, which is 12)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h0c, 5'd30, 5'd0, 5'd0, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.programCounter == 12)
            $display("Test Case 13 - Call Instruction: Pass");
        else
            $display("Test Case 13 - Call Instruction: Fail");

        // -------------------------------------
        // Test Case 14 - Branch if Greater Than (Brgt) Instruction
        // (Opcode 5'h0e: if ($signed(rs) > $signed(rt)) then PC = rdVal;
        //  Here reg[29] (36) > reg[30] (12), so PC should be reg[rd] where rd = 18 and reg[18]=61)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h0e, 5'd18, 5'd29, 5'd30, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.programCounter == 61)
            $display("Test Case 14 - Brgt Instruction: Pass");
        else
            $display("Test Case 14 - Brgt Instruction: Fail");

        // -------------------------------------
        // Test Case 15 - MOV Register Instruction
        // (Opcode 5'h11: mov rd, rs)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h11, 5'd7, 5'd19, 5'd0, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[7] == cpu.reg_file.registers[19])
            $display("Test Case 15 - Mov Reg Instruction: Pass");
        else
            $display("Test Case 15 - Mov Reg Instruction: Fail");

        // -------------------------------------
        // Test Case 16 - MOV Literal Instruction
        // (Opcode 5'h12: mov rd, literal)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h12, 5'd8, 5'd0, 5'd0, 12'd35};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[8] == 35)
            $display("Test Case 16 - Mov Literal Instruction: Pass");
        else
            $display("Test Case 16 - Mov Literal Instruction: Fail. Actual value: %d", cpu.reg_file.registers[8]);

        // -------------------------------------
        // Test Case 17 - Floating Point Add (Addf) Instruction
        // (Opcode 5'h14: perform floating add using the FPU)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h14, 5'd9, 5'd24, 5'd25, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if ($bitstoreal(cpu.reg_file.registers[9]) == 80.0)
            $display("Test Case 17 - Addf Instruction: Pass");
        else
            $display("Test Case 17 - Addf Instruction: Fail");

        // -------------------------------------
        // Test Case 18 - Floating Point Subtract (Subf) Instruction
        // (Opcode 5'h15: perform floating subtract using the FPU)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h15, 5'd10, 5'd25, 5'd26, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if ($bitstoreal(cpu.reg_file.registers[10]) == -534.0)
            $display("Test Case 18 - Subf Instruction: Pass");
        else
            $display("Test Case 18 - Subf Instruction: Fail");

        // -------------------------------------
        // Test Case 19 - Floating Point Multiply (Mulf) Instruction
        // (Opcode 5'h16: perform floating multiply using the FPU)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h16, 5'd11, 5'd26, 5'd27, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if ($bitstoreal(cpu.reg_file.registers[11]) == 32012.0)
            $display("Test Case 19 - Mulf Instruction: Pass");
        else
            $display("Test Case 19 - Mulf Instruction: Fail");

        // -------------------------------------
        // Test Case 20 - Floating Point Divide (Divf) Instruction
        // (Opcode 5'h17: perform floating divide using the FPU)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h17, 5'd12, 5'd27, 5'd28, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (($bitstoreal(cpu.reg_file.registers[12]) - 0.058889) < 0.00001)
            $display("Test Case 20 - Divf Instruction: Pass");
        else
            $display("Test Case 20 - Divf Instruction: Fail");

        // -------------------------------------
        // Test Case 21 - ADD Instruction
        // (Opcode 5'h18: integer addition: rd = rs + rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h18, 5'd13, 5'd16, 5'd29, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[13] == 283)
            $display("Test Case 21 - Add Instruction: Pass");
        else
            $display("Test Case 21 - Add Instruction: Fail");

        // -------------------------------------
        // Test Case 22 - ADD Immediate (Addi) Instruction
        // (Opcode 5'h19: rd = rs + literal)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h19, 5'd20, 5'd0, 5'd0, 12'd6};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[20] == 36)
            $display("Test Case 22 - Addi Instruction: Pass");
        else
            $display("Test Case 22 - Addi Instruction: Fail");

        // -------------------------------------
        // Test Case 23 - SUB Instruction
        // (Opcode 5'h1a: integer subtraction: rd = rs - rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h1a, 5'd15, 5'd29, 5'd30, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[15] == 24)
            $display("Test Case 23 - Sub Instruction: Pass");
        else
            $display("Test Case 23 - Sub Instruction: Fail");

        // -------------------------------------
        // Test Case 24 - SUB Immediate (Subi) Instruction
        // (Opcode 5'h1b: rd = rs - literal)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h1b, 5'd22, 5'd0, 5'd0, 12'd7};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[22] == 10)
            $display("Test Case 24 - Subi Instruction: Pass");
        else
            $display("Test Case 24 - Subi Instruction: Fail");

        // -------------------------------------
        // Test Case 25 - MUL Instruction
        // (Opcode 5'h1c: integer multiply: rd = rs * rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h1c, 5'd0, 5'd20, 5'd19, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[0] == 0)
            $display("Test Case 25 - Mul Instruction: Pass");
        else
            $display("Test Case 25 - Mul Instruction: Fail");

        // -------------------------------------
        // Test Case 26 - DIV Instruction
        // (Opcode 5'h1d: integer division: rd = rs / rt)
        reset = 1;
        #5;
        reset = 0;
        initializeRegisters();
        instruction = {5'h1d, 5'd1, 5'd19, 5'd20, 12'd0};
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        #10;
        if (cpu.reg_file.registers[1] == 0)
            $display("Test Case 26 - Div Instruction: Pass");
        else
            $display("Test Case 26 - Div Instruction: Fail");

        $finish;
    end
endmodule
