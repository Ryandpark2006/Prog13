module fetch_control(
    input wire clk, reset, enable,
    input wire hlt,
    input wire changing_pc,     
    input wire [63:0] updated_next,        
    output reg [63:0] currPC
);    
    reg [63:0] pc;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= {18'b0, 14'b10000000000000}; 
        end
        else if (enable) begin 
            // if(~halt) begin
                if(changing_pc) begin 
                    pc <= updated_next; 
                end 
                else begin 
                    if (hlt) begin end
                    else 
                        pc <= pc + 4;
                end 
            // end
        end
        // when enable is not on, nothing happens 
        // currPC = pc;
    end

    always @(*) begin 
        currPC = pc;
    end

endmodule