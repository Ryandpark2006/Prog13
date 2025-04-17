module ALU(
    
);
    always @(*) begin
        writeEnable = 1'b1;
        changing_pc = 1'b0;
        mem_write_enable = 1'b0;
        case (opcode)
            // integer arithmetic instructions
            5'b11000: begin
                result = operand1 + operand2;    // add
            end
            5'b11001: result = operand1 + operand2;    // addi
            5'b11010: result = operand1 - operand2;    // sub
            5'b11011: result = operand1 - operand2;    // subi
            5'b11100: result = operand1 * operand2;    // mul
            5'b11101: result = operand1 / operand2;    // div
            
            // logic instructions
            5'b00000: result = operand1 & operand2;    // and
            5'b00001: result = operand1 | operand2;    // or
            5'b00010: result = operand1 ^ operand2;    // xor
            5'b00011: result = ~ operand1;    // not 
            5'b00100: result = operand1 >> operand2;    // shftr
            5'b00101: result = operand1 >> operand2;    // shftri     
            5'b00110: result = operand1 << operand2;    // shftl
            5'b00111: result = operand1 << operand2;    // shftli

            // data movement instructions
            5'b10001: result = operand1;    // mov rd, rs
            5'b10010: result = operand2;    // mov rd, L
            // added memory
            5'b10000: begin            // mov rd, (rs)(L)
                writeEnable = 1'b1;
                rw_addr = operand1 + operand2; 
                mem_write_enable = 1'b0;
                result = r_out; 
            end
            5'b10011: begin
                writeEnable = 1'b0;
                mem_write_enable = 1'b1;
                rw_addr = rdVal + operand2;
                // $display("rw_addr = %64b", rw_addr);   
                // $display("operand 1: %64b", operand1);   
                rw_val = operand1;               // mov (rd)(L), rs
                // $display("rw_val: %64b", rw_val);   
            end

            // control instructions (regards memory)
            5'b01000: begin 
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = rdVal; 
            end 
            5'b01001: begin 
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = pc + rdVal; 
            end 
            5'b01010: begin  
                writeEnable = 1'b0;
                changing_pc = 1'b1;
                updated_next = pc + operand2; 
            end 
            5'b01011: begin 
                // if rs != 0 pc = rd
                writeEnable = 1'b0;
                if (operand1) begin
                    changing_pc = 1'b1;
                    updated_next = rdVal; 
                end 
                else begin 
                    changing_pc = 1'b0;
                end
            end 
            5'b01101: begin    // return
                writeEnable = 1'b0;
                changing_pc = 1'b1;   // do i need to do some logic here?
                mem_write_enable = 1'b1;
                rw_addr = r31_val - 8;
                updated_next = r_out; 
            end 
            5'b01100: begin   // call 
                // maybe make special command here
                writeEnable = 1'b0;
                changing_pc = 1'b1;  
                mem_write_enable = 1'b0;
                rw_val = pc + 4;
                rw_addr = r31_val - 8;
                updated_next = rdVal; 
            end 
            5'b01110: begin   // brgt
                writeEnable = 1'b0;
                changing_pc = 1'b1;   
                updated_next = operand1 <= operand2 ? pc + 4 : rdVal;
            end 

            default: begin 
                result = 64'b0;    // Default case (NOP)
                updated_next = 1'b0; 
                writeEnable = 1'b0;
                mem_write_enable = 1'b0;
                rw_addr = 32'b0;
                rw_val = 64'b0;
                updated_next = 64'b0;
                changing_pc = 1'b0;                
            end
        endcase
    end

endmodule

module FPU(
    input wire [63:0] operand1,  // rs
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
            5'b10111: result =  $realtobits($bitstoreal(operand1) / $bitstoreal(operand2)); 
            default: result = 64'b0; // Default case (NOP)
        endcase

        writeEnable = 1;
    end

    // assign writeEnable = 1; 
endmodule