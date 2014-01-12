/***********************************************************************************************
 Pulse Width Modulation
 December 2006 (ATLERA)
*********************************************************************************/

module pwm_main (up, dn, pwm);

input up, dn;                        // PWM control buttons

output pwm;
//pwm_inv, pwm1, pwm2, pwm3, pwm4; // Five sets of PWM and one set of its compliment

wire [3:0] dc_w;
wire clk_w, oscena_w, dc_clk_w, pwm_clk_w, pwm1,pwm2,pwm3,pwm4,pwm_inv;

//assign pwm1 = pwm;
//assign pwm2 = pwm;
//assign pwm3 = pwm;
//assign pwm4 = pwm;
//assign pwm_inv = !pwm;

assign oscena_w = 1;                 //Enabling internal UFM oscillator


        altufm_osc0_altufm_osc_1p3 u1 (.osc(clk_w), .oscena(oscena_w));
		clk_gen u2 (.osc(clk_w), .duty_cycle_clk(dc_clk_w), .pwm_clk(pwm_clk_w));
        duty_cycle u3 (.up(up), .dn(dn), .duty_cycle(dc_w), .clk(dc_clk_w));
        pwm_gen u4 (.duty_cycle(dc_w), .clk(pwm_clk_w), .pwm(pwm));

endmodule

/****************************************************************************************************/
/*Instantiation of UFM oscillator for utilizing built in clock*/

`timescale 1 ps / 1 ps
//synopsys translate_on
module  altufm_osc0_altufm_osc_1p3
    (
    osc,
    oscena) /* synthesis synthesis_clearbox=1 */;
    output   osc;
    input   oscena;

    wire  wire_maxii_ufm_block1_osc;

    maxii_ufm   maxii_ufm_block1
    (
    .arclk(1'b0),
    .ardin(1'b0),
    .arshft(1'b0),
    .bgpbusy(),
    .busy(),
    .drclk(1'b0),
    .drdout(),
    .drshft(1'b0),
    .osc(wire_maxii_ufm_block1_osc),
    .oscena(oscena)
    `ifdef FORMAL_VERIFICATION
    `else
    // synopsys translate_off
    `endif
    ,
    .drdin(1'b0),
    .erase(1'b0),
    .program(1'b0)
    `ifdef FORMAL_VERIFICATION
    `else
    // synopsys translate_on
    `endif
    // synopsys translate_off
    ,
    .ctrl_bgpbusy(),
    .devclrn(),
    .devpor(),
    .sbdin(),
    .sbdout()
    // synopsys translate_on
    );
    defparam
        maxii_ufm_block1.address_width = 9,
        maxii_ufm_block1.osc_sim_setting = 300000,
        maxii_ufm_block1.lpm_type = "maxii_ufm";
    assign
        osc = wire_maxii_ufm_block1_osc;
endmodule //altufm_osc0_altufm_osc_1p3


/*********************************************************************************************
*  Clk_gen module for generating reduced frequencies from UFM Osc 
   and to make suitable for generating duty cycle data clock and 
   PWM generating clock
**********************************************************************************************/
module clk_gen(osc, pwm_clk, duty_cycle_clk);

input osc;

output pwm_clk, duty_cycle_clk;

reg [18:0] count;
initial count=19'b0000000000000000000;

reg duty_cycle_clk;
reg pwm_clk;

always @ (posedge osc)
    begin
        count <= count + 1;
        pwm_clk <= count[10];
        duty_cycle_clk <= count[18];   
    end
endmodule

/*********************************************************************************************
* Generates 4 bit duty cycle word, each count (6.25%) corresponding to 
  1 of 16 duty cycle percentages
*********************************************************************************************/
module duty_cycle (up, dn, clk, duty_cycle);

input up, dn, clk;

output [3:0] duty_cycle;
initial duty_cycle=4'b0000;

reg [3:0] duty_cycle;

always @(posedge clk)
    begin
        if (!up)begin
            if (duty_cycle!=4'b1111)    				// upward limit
                duty_cycle <= duty_cycle +1;
        end else begin								
            if (!dn)begin
                if (duty_cycle!=4'b0000)					// downward limit
                    duty_cycle <= duty_cycle -1;
            end
        end
    end       
   
endmodule

/*************************************************************************************************************
* PWM generating module, uses duty cycle value from duty_cycle module
*************************************************************************************************************/
module pwm_gen (clk, duty_cycle, pwm);

input clk;
input [3:0] duty_cycle;

output pwm;

reg [3:0] count;
reg pwm, pwm_temp;

initial 
begin
count=4'b0000;
pwm=1'b0;
end

always @ (posedge clk)
begin
	count = count+1;
		if (count == duty_cycle) pwm = pwm_temp;
		else 
			begin
			if (count < duty_cycle) 
				begin 
					pwm = 1'b1;
					pwm_temp = pwm;
				end
			else if (count > duty_cycle)
				begin
					pwm =1'b0;
					pwm_temp = pwm;
				end
			end
end

endmodule

/****************************** End **************************************************************************/