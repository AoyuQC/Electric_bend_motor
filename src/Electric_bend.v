
// Top Level module
module Electric_bend (
CLK,
FSMC_A,FSMC_NOE,FSMC_NWE,FSMC_NE1,FSMC_D,
led1,led2,led3,
M1_PWR,M1_PWM,M1_FWDREV,M1_EN,M1_BR,M1_LI,M1_DIAG,M1_IS,M1_TACHO,M1_H1,M1_H2,M1_H3,
M2_PWR,M2_PWM,M2_FWDREV,M2_EN,M2_BR,M2_LI,M2_DIAG,M2_IS,M2_TACHO,M2_H1,M2_H2,M2_H3,
M3_PWR,M3_PWM,M3_FWDREV,M3_EN,M3_BR,M3_LI,M3_DIAG,M3_IS,M3_TACHO,M3_H1,M3_H2,M3_H3,
M1_SPEED,M2_SPEED,M3_SPEED);

input CLK;						//EX CLK 50Mhz
input [4:0] FSMC_A;			//A0-A4 from STM32 FSMC
input FSMC_NOE;				//output enable
input FSMC_NWE;				//write enable
input FSMC_NE1;			//chip enable NE1-2 from STM32 FSMC

inout [7:0] FSMC_D;			//D0-DD7 to STM32 FSMC

//control signals for motors
//location: in front of ELECTRIC_BEND Rev.A, near CPLD&STM32
//motor1: in the middile
//motor2: on the right
//motor3: on the left
//control signals for motor 1
input M1_LI;					//Light sensor signal of motor 1
input M1_DIAG;					//Diagnosis signal of motor 1
input M1_IS;					//Diagnosis signal of power supply(BTS5235) for motor 1
input M1_TACHO;    			//speed feedback from motor 1
input M1_H1,M1_H2,M1_H3;	//hall sensor signals from motor 1

output M1_PWR;					//Power supply control of BTS5235
output M1_PWM;					//PWM signals for speed control
output M1_FWDREV;				//FWDREV signals for direction control
output M1_EN;					//EN signals for enable L6235
output M1_BR;					//BRAKE signals

//control signals for motor 2
input M2_LI;					//Light sensor signal of motor 2
input M2_DIAG;					//Diagnosis signal of motor 2
input M2_IS;					//Diagnosis signal of power supply(BTS5235) for motor 2
input M2_TACHO;    			//speed feedback from motor 2
input M2_H1,M2_H2,M2_H3;	//hall sensor signals from motor 2

output M2_PWR;					//Power supply control of BTS5235
output M2_PWM;					//PWM signals for speed control
output M2_FWDREV;				//FWDREV signals for direction control
output M2_EN;					//EN signals for enable L6235
output M2_BR;					//BRAKE signals

//control signals for motor 3
input M3_LI;					//Light sensor signal of motor 3
input M3_DIAG;					//Diagnosis signal of motor 3
input M3_IS;					//Diagnosis signal of power supply(BTS5235) for motor 3
input M3_TACHO;    			//speed feedback from motor 3
input M3_H1,M3_H2,M3_H3;	//hall sensor signals from motor 3

output M3_PWR;					//Power supply control of BTS5235
output M3_PWM;					//PWM signals for speed control
output M3_FWDREV;				//FWDREV signals for direction control
output M3_EN;					//EN signals for enable L6235
output M3_BR;					//BRAKE signals

//speed signal of motor1-3
output M1_SPEED;
output M2_SPEED;
output M3_SPEED;

output led1,led2,led3;			//debug led1:D19 led2:D20 led3:D18

reg [7:0] data[18:0]; 		//array to store data
reg [7:0] port_reg;
reg [4:0] addr;			   //to store FSMC address from STM32
reg [7:0] M1_STAT_BACK;

wire [7:0] S_STAT,M1_STAT,M2_STAT,M3_STAT;					//status of solenoid valve & motor 1 2 3
wire [3:0] M1_C,M2_C,M3_C;	    				               //control signals of motor 1 2 3
wire [15:0] M1_C_V, M1_D_V,M2_C_V,M2_D_V,M3_C_V,M3_D_V;	//counting value and duty value of motor 1 2 3 pwm

////assign value to addr
//assign addr = FSMC_NE[1] ? 4'bzzzz : FSMC_A[3:0];	

//stm32 & CPLD memory transacton
//stm32 read data from CPLD
//always @(negedge FSMC_NOE or negedge FSMC_NWE or negedge FSMC_NE[1])	 
always @(negedge CLK)	 
	begin
		if(FSMC_NE1 == 0)
			addr = FSMC_A[4:0];
		else
			addr = 5'bzzzzz;
			
		if(FSMC_NWE == 0 && addr != 5'd15 && addr != 5'd16 && addr != 5'd17 && addr != 5'd18)
			data[addr] = FSMC_D;
		else if(addr == 5'd15)
			data[15] = S_STAT;
		else if(addr == 5'd16)
			begin
			data[16] = M1_STAT;
			//data[16] = 8'b10101010;
			//led3_r = ~led3_r;
			end		
		else if(addr == 5'd17)
			data[17] = M2_STAT;
		else if(addr == 5'd18)
			data[18] = M3_STAT;
			
		
		if(FSMC_NOE == 0)
			begin
			port_reg = data[addr];
			//led3_r = ~led3_r;
			end
		else
			port_reg = 8'bzzzzzzzz;
	end								


assign FSMC_D = port_reg;
//split data to control signals and pwm data
assign M1_C = data[0][3:0];
assign M1_C_V = {data[2], data[1]};
assign M1_D_V = {data[4], data[3]};

assign M2_C = data[0][7:4];
assign M2_C_V = {data[6], data[5]};
assign M2_D_V = {data[8], data[7]};

assign M3_C = data[9][3:0];
assign M3_C_V = {data[11], data[10]};
assign M3_D_V = {data[13], data[12]};

assign led1 = M2_C[0];
assign led3 = M2_STAT[7];

//control module(MOTOR CONTROL) for motor x(x = 1-3)
MOTOR_CONTROL M1C (.CLK(CLK), .M_C(M1_C),
                  .PWR(M1_PWR), .PWM(M1_PWM), .FWDREV(M1_FWDREV), .EN(M1_EN), .BR(M1_BR), .COUNT_VALUE(16'd25000), .DUTY_VALUE(M1_D_V));
                  //.cn0(led1));
						
MOTOR_CONTROL M2C (.CLK(CLK), .M_C(M2_C),
                  .PWR(M2_PWR), .PWM(M2_PWM), .FWDREV(M2_FWDREV), .EN(M2_EN), .BR(M2_BR), .COUNT_VALUE(16'd25000), .DUTY_VALUE(M2_D_V));

//MOTOR_CONTROL M3C (.CLK(CLK), .M_C(M3_C), 
//                  .PWR(M3_PWR), .PWM(M3_PWM), .FWDREV(M3_FWDREV), .EN(M3_EN), .BR(M3_BR), .COUNT_VALUE(32'd25000000), .DUTY_VALUE(M3_D_V));
                  
//feedback module for motor x (x = 1-3)
MOTOR_FEEDBACK M1F (.CLK(CLK), .M_STAT(M1_STAT), .Mx_H(M1_SPEED), .LI(M1_LI), .DIAG(M1_DIAG), .IS(M1_IS), .TACHO(M1_TACHO), .H1(M1_H1), .H2(M1_H2), .H3(M1_H3));

MOTOR_FEEDBACK M2F (.CLK(CLK), .M_STAT(M2_STAT), .Mx_H(M2_SPEED), .LI(M2_LI), .DIAG(M2_DIAG), .IS(M2_IS), .TACHO(M2_TACHO), .H1(M2_H1), .H2(M2_H2), .H3(M2_H3));						


endmodule
