module register_file(
    input wire clk, reset, write_enable,      
    input wire [63:0] dataInput,
    input wire [4:0] readAddress1, 
    input wire [4:0] readAddress2, 
    input wire [4:0] writeAddress,
    input wire lPassed,
    input wire [11:0]L,
    output wire [63:0] value1,
    output wire [63:0] value2, 
    output wire [63:0] rdVal,
    output wire [63:0] r31_val
);
    reg [63:0] registers [31:0];

    // read operation
    assign value1 = registers[readAddress1];
    // assign value2 = registers[readAddress2];
    assign value2 = lPassed ? {52'b0, L} : registers[readAddress2];

    // this is for the mov instructions
    assign rdVal = registers[writeAddress];
    assign r31_val = registers[31];

    // this is fine (nned to update write_enable though later on after its calculated)
    // always @(*) begin
    //     if (write_enable) begin
    //         registers[writeAddress] = dataInput;
    //     end
    // end
    integer i;
    always @(posedge clk) begin
        if  (reset) begin 
            for(i = 0; i < 31; i++) begin 
                registers[i] = 0;
            end
            registers[31] <= 64'h80000;
        end
        if (write_enable) begin
            registers[writeAddress] <= dataInput;
        end
    end
endmodule