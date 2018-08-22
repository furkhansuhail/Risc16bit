`timescale 1ns / 1ps
// Verilog code for RISC Processor 

`include "AluControl.v"
`include "alu.v"
`include "data.v"
`include "register.v"
`include "prog.v"
`include "Parameter.v"
`include "Datapath_Unit.v"
`include "Control_Unit.v"

module Risc_16_bit(
 input clk
);
 wire jump,bne,beq,mem_read,mem_write,alu_src,reg_dst,mem_to_reg,reg_write;
 wire[1:0] alu_op;
 wire [3:0] opcode;
 // Datapath
 Datapath_Unit DU
 (
  .clk(clk),
  .jump(jump),
  .beq(beq),
  .mem_read(mem_read),
  .mem_write(mem_write),
  .alu_src(alu_src),
  .reg_dst(reg_dst),
  .mem_to_reg(mem_to_reg),
  .reg_write(reg_write),
  .bne(bne),
  .alu_op(alu_op),
  .opcode(opcode)
 );
 // control unit
 Control_Unit control
 (
  .opcode(opcode),
  .reg_dst(reg_dst),
  .mem_to_reg(mem_to_reg),
  .alu_op(alu_op),
  .jump(jump),
  .bne(bne),
  .beq(beq),
  .mem_read(mem_read),
  .mem_write(mem_write),
  .alu_src(alu_src),
  .reg_write(reg_write)
 );
endmodule

`timescale 1ns / 1ps
// Verilog code for Data Path of the processor
module Datapath_Unit(
 input clk,
 input jump,beq,mem_read,mem_write,alu_src,reg_dst,mem_to_reg,reg_write,bne,
 input[1:0] alu_op,
 output[3:0] opcode
);
 reg  [15:0] pc_current;
 wire [15:0] pc_next,pc2;
 wire [15:0] instr;
 wire [2:0] reg_write_dest;
 wire [15:0] reg_write_data;
 wire [2:0] reg_read_addr_1;
 wire [15:0] reg_read_data_1;
 wire [2:0] reg_read_addr_2;
 wire [15:0] reg_read_data_2;
 wire [15:0] ext_im,read_data2;
 wire [2:0] ALU_Control;
 wire [15:0] ALU_out;
 wire zero_flag;
 wire [15:0] PC_j, PC_beq, PC_2beq,PC_2bne,PC_bne;
 wire beq_control;
 wire [12:0] jump_shift;
 wire [15:0] mem_read_data;
 // PC 
 initial begin
  pc_current <= 16'd0;
 end
 always @(posedge clk)
 begin 
   pc_current <= pc_next;
 end
 assign pc2 = pc_current + 16'd2;
 // instruction memory
 Instruction_Memory im(.pc(pc_current),.instruction(instr));
 // jump shift left 2
 assign jump_shift = {instr[11:0],1'b0};
 // multiplexer regdest
 assign reg_write_dest = (reg_dst==1'b1) ? instr[5:3] :instr[8:6];
 // register file
 assign reg_read_addr_1 = instr[11:9];
 assign reg_read_addr_2 = instr[8:6];
 
 // GENERAL PURPOSE REGISTERs
 GPRs reg_file
 (
  .clk(clk),
  .reg_write_en(reg_write),
  .reg_write_dest(reg_write_dest),
  .reg_write_data(reg_write_data),
  .reg_read_addr_1(reg_read_addr_1),
  .reg_read_data_1(reg_read_data_1),
  .reg_read_addr_2(reg_read_addr_2),
  .reg_read_data_2(reg_read_data_2)
 );
 // immediate extend
 assign ext_im = {{10{instr[5]}},instr[5:0]};  
 // ALU control unit
  alu_control ALU_Control_unit(.ALUOp(alu_op),.Function(instr[15:12]),.ALU_Control(ALU_Control));
 // multiplexer alu_src
 assign read_data2 = (alu_src==1'b1) ? ext_im : reg_read_data_2;
 // ALU 
 ALU alu_unit(.a(reg_read_data_1),.b(read_data2),.alu_control(ALU_Control),.result(ALU_out),.zero(zero_flag));
 // PC beq add
 assign PC_beq = pc2 + {ext_im[14:0],1'b0};
 assign PC_bne = pc2 + {ext_im[14:0],1'b0};
 // beq control
 assign beq_control = beq & zero_flag;
 assign bne_control = bne & (~zero_flag);
 // PC_beq
 assign PC_2beq = (beq_control==1'b1) ? PC_beq : pc2;
 // PC_bne
 assign PC_2bne = (bne_control==1'b1) ? PC_bne : PC_2beq;
 // PC_j
 assign PC_j = {pc2[15:13],jump_shift};
 // PC_next
 assign pc_next = (jump == 1'b1) ? PC_j :  PC_2bne;

 /// Data memory
  Data_Memory dm
   (
    .clk(clk),
    .mem_access_addr(ALU_out),
    .mem_write_data(reg_read_data_2),
    .mem_write_en(mem_write),
    .mem_read(mem_read),
    .mem_read_data(mem_read_data)
   );
 
 // write back
 assign reg_write_data = (mem_to_reg == 1'b1)?  mem_read_data: ALU_out;
 // output to control unit
 assign opcode = instr[15:12];
endmodule

`timescale 1ns / 1ps
// Verilog code for Control Unit 
module Control_Unit(
      input[3:0] opcode,
      output reg[1:0] alu_op,
      output reg jump,beq,bne,mem_read,mem_write,alu_src,reg_dst,mem_to_reg,reg_write    
    );


always @(*)
begin
 case(opcode) 
 4'b0000:  // LW
   begin
    reg_dst = 1'b0;
    alu_src = 1'b1;
    mem_to_reg = 1'b1;
    reg_write = 1'b1;
    mem_read = 1'b1;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b10;
    jump = 1'b0;   
   end
 4'b0001:  // SW
   begin
    reg_dst = 1'b0;
    alu_src = 1'b1;
    mem_to_reg = 1'b0;
    reg_write = 1'b0;
    mem_read = 1'b0;
    mem_write = 1'b1;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b10;
    jump = 1'b0;   
   end
 4'b0010:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b0011:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b0100:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b0101:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b0110:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b0111:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b1000:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b1001:  // data_processing
   begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0;   
   end
 4'b1011:  // BEQ
   begin
    reg_dst = 1'b0;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b0;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b1;
    bne = 1'b0;
    alu_op = 2'b01;
    jump = 1'b0;   
   end
 4'b1100:  // BNE
   begin
    reg_dst = 1'b0;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b0;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b1;
    alu_op = 2'b01;
    jump = 1'b0;   
   end
 4'b1101:  // J
   begin
    reg_dst = 1'b0;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b0;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b1;   
   end   
 default: begin
    reg_dst = 1'b1;
    alu_src = 1'b0;
    mem_to_reg = 1'b0;
    reg_write = 1'b1;
    mem_read = 1'b0;
    mem_write = 1'b0;
    beq = 1'b0;
    bne = 1'b0;
    alu_op = 2'b00;
    jump = 1'b0; 
   end
 endcase
 end

endmodule



























`include "Parameter.v"
// Verilog code for Instruction Memory
module Instruction_Memory(
 input[15:0] pc,
 output[15:0] instruction
);

 reg [`col - 1:0] memory [`row_i - 1:0];
 wire [3 : 0] rom_addr = pc[4 : 1];
 initial
 begin
  $readmemb("prog.mem", memory,0,14);
 end
 assign instruction =  memory[rom_addr]; 

endmodule

`timescale 1ns / 1ps
// Verilog code for register file
module GPRs(
 input    clk,
 // write port
 input    reg_write_en,
 input  [2:0] reg_write_dest,
 input  [15:0] reg_write_data,
 //read port 1
 input  [2:0] reg_read_addr_1,
 output  [15:0] reg_read_data_1,
 //read port 2
 input  [2:0] reg_read_addr_2,
 output  [15:0] reg_read_data_2
);
 reg [15:0] reg_array [7:0];
 integer i;
 // write port
 //reg [2:0] i;
 initial begin
  for(i=0;i<8;i=i+1)
   reg_array[i] <= 16'd0;
 end
 always @ (posedge clk ) begin
   if(reg_write_en) begin
    reg_array[reg_write_dest] <= reg_write_data;
   end
 end
 

 assign reg_read_data_1 = reg_array[reg_read_addr_1];
 assign reg_read_data_2 = reg_array[reg_read_addr_2];


endmodule

`timescale 1ns / 1ps  
 //Verilog code for ALU Control
 module alu_control( ALU_Control, ALUOp, Function);  
 output reg[2:0] ALU_Control;  
 input [1:0] ALUOp;  
 input [3:0] Function;  
 wire [5:0] ALUControlIn;  
 assign ALUControlIn = {ALUOp,Function};  
 always @(ALUControlIn)  
 casex (ALUControlIn)  
  6'b10xxxx: ALU_Control=3'b000;  
  6'b01xxxx: ALU_Control=3'b001;  
  6'b000010: ALU_Control=3'b000;  
  6'b000011: ALU_Control=3'b001;  
  6'b000100: ALU_Control=3'b010;  
  6'b000101: ALU_Control=3'b011;  
  6'b000110: ALU_Control=3'b100;  
  6'b000101: ALU_Control=3'b101;  
  6'b001000: ALU_Control=3'b110;  
  6'b001001: ALU_Control=3'b111;  
  default: ALU_Control=3'b000;  
  endcase  
 endmodule  

// Verilog code for ALU
module ALU(
 input  [15:0] a,  //src1
 input  [15:0] b,  //src2
 input  [2:0] alu_control, //function sel
 
 output reg [15:0] result,  //result 
 output zero
    );

always @(*)
begin 
 case(alu_control)
 3'b000: result = a + b; // add
 3'b001: result = a - b; // sub
 3'b010: result = ~a;
 3'b011: result = a<<b;
 3'b100: result = a>>b;
 3'b101: result = a & b; // and
 3'b110: result = a | b; // or
 3'b111: begin if (a<b) result = 16'd1;
    else result = 16'd0;
    end
 default:result = a + b; // add
 endcase
end
assign zero = (result==16'd0) ? 1'b1: 1'b0;
 
endmodule

`include "Parameter.v"
// Verilog code for data Memory
module Data_Memory(
 input clk,
 // address input, shared by read and write port
 input [15:0]   mem_access_addr,
 
 // write port
 input [15:0]   mem_write_data,
 input     mem_write_en,
 input mem_read,
 // read port
 output [15:0]   mem_read_data
);

reg [`col - 1:0] memory [`row_d - 1:0];
integer f;
wire [2:0] ram_addr=mem_access_addr[2:0];
initial
 begin
   $readmemb("data.mem", memory);

  `simulation_time;

 end
 
 always @(posedge clk) begin
  if (mem_write_en)
   memory[ram_addr] <= mem_write_data;
 end
 assign mem_read_data = (mem_read==1'b1) ? memory[ram_addr]: 16'd0; 


endmodule



//Memory Program
0000_0100_0000_0000 
0000_0100_0100_0001 
0010_0000_0101_0000 
0001_0010_1000_0000 
0011_0000_0101_0000 
0100_0000_0101_0000  
0101_0000_0101_0000  
0110_0000_0101_0000 
0111_0000_0101_0000  
1000_0000_0101_0000 
1001_0000_0101_0000  
0010_0000_0000_0000 
1011_0000_0100_0001 
1100_0000_0100_0000 
1101_0000_0000_0000

//Memory Data
0000_0000_0000_0001
0000_0000_0000_0010
0000_0000_0000_0001
0000_0000_0000_0010
0000_0000_0000_0001
0000_0000_0000_0010
0000_0000_0000_0001
0000_0000_0000_0010

`timescale 1ns / 1ps
`include "Parameter.v"
// Verilog testbench code to test the processor
module test_Risc_16_bit;

 // Inputs
 reg clk;

 // Instantiate the Unit Under Test (UUT)
 Risc_16_bit uut (
  .clk(clk)
 );

 initial 
  begin
  
   clk <=0;
   `simulation_time;
   $finish;
  end

 always 
  begin
   #5 clk = ~clk;
  end

endmodule
