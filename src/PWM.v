/********************************************************************************
 PWM MODULE
 2013
 ZAY
*********************************************************************************/

module PWM(
CLK,
COUNT_VALUE,
DUTY_VALUE,
PWM);
//cn0);

input 		CLK;					//EX CLK 50Mhz
input[15:0] COUNT_VALUE;			//for Frequency division(Khz)
input[15:0] DUTY_VALUE; 			//duty VALUE of PWM signal

output PWM; //PWM signal for voltage reference
//output cn0;
 
reg[31:0] cn;
reg PWM, pwm_temp;

//assign cn0 = cn[13];

initial 
	begin
		cn=15'd0;
		PWM=1'b0;
	end

always @ (posedge CLK)
	begin
				if (cn == COUNT_VALUE)	//clock division: pwm_clk = ex clk(50mhz)/(count_value+1)
					cn <= 0;
				else
					cn <= cn+1;
				
				if (cn == DUTY_VALUE) 	//pwm duty: 1.duty_VALUE:1; 2.count_value-duty_VALUE:0
					PWM <= pwm_temp;
				else 
					begin
						if (cn < DUTY_VALUE) 
							begin 
								PWM <= 1'b1;
								pwm_temp <= PWM;
							end
						else if (cn > DUTY_VALUE)
							begin
								PWM <= 1'b0;
								pwm_temp <= PWM;
							end
					end
	end
	

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

endmodule
