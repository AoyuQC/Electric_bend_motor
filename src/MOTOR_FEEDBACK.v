/********************************************************************************
 MOTOR STATUS FEEDBACK MODULE
 2013
 ZAY
*********************************************************************************/

module MOTOR_FEEDBACK(
CLK,M_STAT,Mx_H,
LI,DIAG,IS,
TACHO,
H1,H2,H3);

input CLK;			//EX CLK 50Mhz
input LI;			//Light sensor signal of motor
input DIAG;			//Diagnosis signal of motor
input IS;			//Diagnosis signal of power supply(BTS5235) for motor
input TACHO;    	//speed feedback from motor
input H1,H2,H3;		//hall sensor signals from motor 

output[7:0] M_STAT;	//status of motor
output Mx_H;

reg Mx_H1_SYNC1,Mx_H1_SYNC2,Mx_H1_SYNC3;
reg Mx_H2_SYNC1,Mx_H2_SYNC2,Mx_H2_SYNC3;
reg Mx_H3_SYNC1,Mx_H3_SYNC2,Mx_H3_SYNC3;
reg Mx_H_reg;
reg LI_SYNC1,LI_SYNC2;
reg dir_fwd_rev;

//status feedback
assign M_STAT[0] = H1;	//hall sensor
assign M_STAT[1] = H2;
assign M_STAT[2] = H3;
assign M_STAT[3] = TACHO;	//speed feedback
assign M_STAT[4] = IS;	//IS signal
assign M_STAT[5] = DIAG;  //DIAG signal
assign M_STAT[6] = LI_SYNC2;	//LI signal
assign M_STAT[7] = dir_fwd_rev;	//reserved

//hall sensor interrupt
always @(posedge CLK)
	begin
		//synchronizer for Mx HALL1(M1_STAT[0]) and back Mx HALL3(M1_STAT[2])
		Mx_H1_SYNC1 <= M_STAT[0];
		Mx_H1_SYNC2 <= Mx_H1_SYNC1;
		Mx_H1_SYNC3 <= Mx_H1_SYNC2;

		Mx_H2_SYNC1 <= M_STAT[1];
		Mx_H2_SYNC2 <= Mx_H2_SYNC1;
		Mx_H2_SYNC3 <= Mx_H2_SYNC2;
				
		Mx_H3_SYNC1 <= M_STAT[2];
		Mx_H3_SYNC2 <= Mx_H3_SYNC1;
		Mx_H3_SYNC3 <= Mx_H3_SYNC2;		
//		//synchronizer for M2 HALL1(M2_STAT[0])
//		M2_H1_SYNC1 <= M2_STAT[0];
//		M2_H1_SYNC2 <= M2_H1_SYNC1;
//		M2_H1_SYNC3 <= M2_H1_SYNC2;
//		if((M1_STAT_BACK[0] != M1_STAT[0]) || (M1_STAT_BACK[1] != M1_STAT[1]) || (M1_STAT_BACK[2] != M1_STAT[2]))
		//detect posedge of H1
		if(Mx_H1_SYNC3 == 1'b0 && Mx_H1_SYNC2 == 1'b1)	
			begin
				Mx_H_reg <= ~Mx_H_reg;
				//direction of motor 1:forwad 0:reverse
				if((Mx_H2_SYNC3 == 1'b0)&&(Mx_H3_SYNC3 == 1'b1))
					dir_fwd_rev <= 1'b1;
				
				if((Mx_H2_SYNC3 == 1'b1)&&(Mx_H3_SYNC3 == 1'b0))
					dir_fwd_rev <= 1'b0;
			end
		//detect negedge of H1
		if(Mx_H1_SYNC3 == 1'b1 && Mx_H1_SYNC2 == 1'b0)	
			begin
				Mx_H_reg <= ~Mx_H_reg;
				//direction of motor 1:forwad 0:reverse
				if((Mx_H2_SYNC3 == 1'b1)&&(Mx_H3_SYNC3 == 1'b0))
					dir_fwd_rev <= 1'b1;
				
				if((Mx_H2_SYNC3 == 1'b0)&&(Mx_H3_SYNC3 == 1'b1))
					dir_fwd_rev <= 1'b0;
			end
		//detect posedge of H2
		if(Mx_H2_SYNC3 == 1'b0 && Mx_H2_SYNC2 == 1'b1)	
			begin
				Mx_H_reg <= ~Mx_H_reg;
				//direction of motor 1:forwad 0:reverse
				if((Mx_H1_SYNC3 == 1'b1)&&(Mx_H3_SYNC3 == 1'b0))
					dir_fwd_rev <= 1'b1;
				
				if((Mx_H1_SYNC3 == 1'b0)&&(Mx_H3_SYNC3 == 1'b1))
					dir_fwd_rev <= 1'b0;
			end
		//detect negedge of H2
		if(Mx_H2_SYNC3 == 1'b1 && Mx_H2_SYNC2 == 1'b0)	
			begin
				Mx_H_reg <= ~Mx_H_reg;
				//direction of motor 1:forwad 0:reverse
				if((Mx_H1_SYNC3 == 1'b0)&&(Mx_H3_SYNC3 == 1'b1))
					dir_fwd_rev <= 1'b1;
				
				if((Mx_H1_SYNC3 == 1'b1)&&(Mx_H3_SYNC3 == 1'b0))
					dir_fwd_rev <= 1'b0;
			end
        //detect posedge of H3
		if(Mx_H3_SYNC3 == 1'b0 && Mx_H3_SYNC2 == 1'b1)	
			begin
				Mx_H_reg <= ~Mx_H_reg;
				//direction of motor 1:forwad 0:reverse
				if((Mx_H1_SYNC3 == 1'b0)&&(Mx_H2_SYNC3 == 1'b1))
					dir_fwd_rev <= 1'b1;
				
				if((Mx_H1_SYNC3 == 1'b1)&&(Mx_H2_SYNC3 == 1'b0))
					dir_fwd_rev <= 1'b0;
			end
        //detect negedge of H3
		if(Mx_H3_SYNC3 == 1'b1 && Mx_H3_SYNC2 == 1'b0)	
			begin
				Mx_H_reg <= ~Mx_H_reg;
				//direction of motor 1:forwad 0:reverse
				if((Mx_H1_SYNC3 == 1'b1)&&(Mx_H3_SYNC3 == 1'b0))
					dir_fwd_rev <= 1'b1;
				
				if((Mx_H1_SYNC3 == 1'b0)&&(Mx_H3_SYNC3 == 1'b1))
					dir_fwd_rev <= 1'b0;
			end							
//		if(M2_H1_SYNC3 == 0 && M2_H1_SYNC2 == 1)
//			begin
//				m2h1_reg <= ~m2h1_reg;
//			end	

		//DFF fot light sensor
		LI_SYNC1 <= LI;
		LI_SYNC2 <= LI_SYNC1;
		
	end

//interrupt for hall 1
assign Mx_H = Mx_H_reg;

endmodule




