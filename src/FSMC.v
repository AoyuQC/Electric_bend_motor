/********************************************************************************
 FSMC(Flexible static memory controller) MODULE
 2013
 ZAY
*********************************************************************************/

module FSMC(
CLK,
FSMC_A,
FSMC_NOE,
FSMC_NWE,
FSMC_NE,
FSMC_D,
led2,led3);

input CLK;						//EX CLK 50Mhz
input [3:0] FSMC_A;				//A0-A3 from STM32 FSMC
input FSMC_NOE;					//output enable
input FSMC_NWE;					//write enable
input [1:0] FSMC_NE;			//chip enable NE1-2 from STM32 FSMC

output led2;
output led3;

inout [7:0] FSMC_D;			//D0-DD7 to STM32 FSMC

reg [7:0] data[15:0]; 			//array to store data
reg [7:0] port_reg;
reg led2;
reg led3;

wire start_mem;					//flag to indicate FSMC_NE
wire [3:0] addr;			    //to store A from STM32


//assign value to addr
assign addr = FSMC_NE[1] ? 4'bzzzz : FSMC_A[3:0];	

//stm32 & CPLD memory transacton
//stm32 read data from CPLD
always @(FSMC_NOE)	 
	begin
		if(FSMC_NOE == 0)
			port_reg = data[addr];
		else
			port_reg = 1'bz;
	end
			
//stm32 wite data to CPLD
always @(negedge FSMC_NWE)	 
	begin
		data[addr] = FSMC_D;
		if (data[4] == 8'b00010110)
			led2 = 1;
		else
			led2 = 0;
			
//		if (data[1] == 8'b00010011)
//			led3 = 1;
//		else
//			led3 = 0;
	end								

assign FSMC_D = port_reg;
	
endmodule
		
		
