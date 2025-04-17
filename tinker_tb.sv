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

    // Clock generation: period = 2 time units.
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end

    // Task to initialize registers inside the register file.
    task initializeRegisters;
        integer j;
        begin
            // Zero out registers 0-15.
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

    // Main test bench block.
    // The revised ordering is:
    // 1. Assert reset and wait sufficiently for memory to initialize.
    // 2. While reset is still asserted, write the test instruction into memory.
    // 3. Deassert reset so the processor starts fetching.
    // 4. Wait enough cycles for the instruction to complete.
    initial begin
        $dumpfile("tinker_tb.vcd");
        $dumpvars(0, tinker_tb);
        
        // Step 1: Assert reset and wait.
        reset = 1;
        #20;  // Give enough time for memory reset to complete.
        initializeRegisters();
        
        // Step 2: While reset is still high, store the test instruction into memory.
        // Test Case 01: AND Instruction
        // Format: {opcode, rd, rs, rt, literal}
        // We want: opcode 5'h00, rd = 0, rs = 16, rt = 17, literal = 0.
        instruction = {5'h00, 5'd0, 5'd16, 5'd17, 12'd0};
        // Write the 32-bit instruction into memory at address 8192.
        {cpu.memory.bytes[8195],
         cpu.memory.bytes[8194],
         cpu.memory.bytes[8193],
         cpu.memory.bytes[8192]} = instruction;
        // Also, write a NOP (defined as 32'h11000000) immediately after.
        {cpu.memory.bytes[8199],
         cpu.memory.bytes[8198],
         cpu.memory.bytes[8197],
         cpu.memory.bytes[8196]} = 32'h11000000;
        
        // Optionally, display the memory contents at 8192 for debugging:
        $display("Memory[8192]=%h %h %h %h",
                 cpu.memory.bytes[8195],
                 cpu.memory.bytes[8194],
                 cpu.memory.bytes[8193],
                 cpu.memory.bytes[8192]);
        
        // Step 3: Now deassert reset.
        reset = 0;
        
        // Step 4: Wait long enough for the instruction to flow through the 5-stage pipeline.
        #50;
        
        // Check: Register 0 should hold 247 & 162 = 162.
        $display("Register 0 = %d", cpu.reg_file.registers[0]);
        if (cpu.reg_file.registers[0] == (64'd247 & 64'd162))
            $display("Test Case 01 - AND Instruction: Pass");
        else
            $display("Test Case 01 - AND Instruction: Fail. Expected: %d, Got: %d",
                     (64'd247 & 64'd162), cpu.reg_file.registers[0]);
                     
        $finish;
    end
endmodule
