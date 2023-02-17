`ifdef RTL
	`include "NN.v"  
	`define CYCLE_TIME 50.0
`endif
`ifdef GATE
	`include "NN_SYN.v"
	`define CYCLE_TIME 50.0
`endif

module PATTERN(
	// Output signals
	clk,
	rst_n,
	in_valid_u,
	in_valid_w,
	in_valid_v,
	in_valid_x,
	weight_u,
	weight_w,
	weight_v,
	data_x,
	// Input signals
	out_valid,
	out
);
//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------
	parameter inst_sig_width = 23;
	parameter inst_exp_width = 8;
	parameter inst_ieee_compliance = 0;
    parameter fp = 31;

//================================================================
//   INPUT AND OUTPUT DECLARATION                         
//================================================================
	output reg clk,rst_n,in_valid_u,in_valid_w,in_valid_v,in_valid_x;
	output reg [inst_sig_width + inst_exp_width: 0] weight_u,weight_w,weight_v,data_x;
	input	out_valid;
	input	[inst_sig_width + inst_exp_width: 0] out;
	
//================================================================
// parameters & integer
//================================================================
integer PATNUM, patcount;
integer total_cycles;
integer i;
real E;
real diff;
reg [fp:0] weight_u_reg [8:0], weight_w_reg [8:0], weight_v_reg [8:0], data_x_reg [8:0], y_reg[8:0];

//================================================================
// clock
//================================================================
always	#(`CYCLE_TIME/2.0) clk = ~clk;
initial	clk = 0;

//================================================================
// initial
//================================================================
initial begin
    PATNUM = 300;
    E = 2.71828182846;

    in_valid_x = 0; in_valid_u = 0; in_valid_w = 0; in_valid_v = 0;
    data_x = 32'dx; weight_u = 32'dx; weight_w = 32'dx; weight_v = 32'dx;
	rst_n = 1;

	force clk = 0;
	reset_task;
	total_cycles = 0;

    @(negedge clk);

	for (patcount=0;patcount<PATNUM;patcount=patcount+1)begin
        gen_input_task;
        gen_answer_task;
        
        wait_outvalid;
		check_ans;
		delay_task;

        $display(" PASS PATTERN NO.%4d, accumulated cycle %d", patcount, total_cycles);
	end
	#(1000);
	YOU_PASS_task;
	$finish;

end


//================================================================
// env task
//================================================================
task reset_task ; begin
	#(0.5); rst_n = 0;

	#(10.0);
    // SPEC3. All output signals should be reset after the reset signal is asserted. 
	if ((out_valid!==0)||(out!==0)) YOU_FAIL(3);
	
	#(1.0); rst_n = 1 ;
	#(3.0); release clk;
end endtask

task YOU_FAIL;
input [4:0] fail_specnum;
begin
	if(fail_specnum==3)       $display ("SPEC 3 IS FAIL!\n");
    else if(fail_specnum==4)  $display ("SPEC 4 IS FAIL!\n");
    else if(fail_specnum==5)  $display ("SPEC 5 IS FAIL!\n");
    else if(fail_specnum==6)  $display ("SPEC 6 IS FAIL!\n");
    else if(fail_specnum==7)  $display ("SPEC 7 IS FAIL!\n");
    else if(fail_specnum==8)  $display ("wrong answer!\n");
	$finish;
end endtask

task gen_input_task;
integer sign, time_seed;
real    real_tmp;
begin
    in_valid_x = 0; in_valid_u = 0; in_valid_w = 0; in_valid_v = 0;
    data_x = 32'dx; weight_u = 32'dx; weight_w = 32'dx; weight_v = 32'dx;

    time_seed = $time;
    in_valid_x = 1; in_valid_u = 1; in_valid_w = 1; in_valid_v = 1;
    for(i=0;i<9;i=i+1)begin
        real_tmp = $urandom(time_seed + i) % 100;
        real_tmp = real_tmp / 100;
        sign = $urandom_range(0, 3);
        real_tmp = sign != 0 ? real_tmp : real_tmp * (-1);
        data_x_reg[i] = $shortrealtobits(real_tmp);
        data_x = data_x_reg[i];
        
        real_tmp = $urandom(time_seed + i*7) % 100;
        real_tmp = real_tmp / 100;
        sign = $urandom_range(0, 3);
        real_tmp = sign != 0 ? real_tmp : real_tmp * (-1);
        weight_u_reg[i] = $shortrealtobits(real_tmp);
        weight_u = weight_u_reg[i];
        
        real_tmp = $urandom(time_seed + i*13) % 100;
        real_tmp = real_tmp / 100;
        sign = $urandom_range(0, 3);
        real_tmp = sign != 0 ? real_tmp : real_tmp * (-1);
        weight_w_reg[i] = $shortrealtobits(real_tmp);
        weight_w = weight_w_reg[i];
        
        real_tmp = $urandom(time_seed + i*19) % 100;
        real_tmp = real_tmp / 100;
        sign = $urandom_range(0, 3);
        real_tmp = sign != 0 ? real_tmp : real_tmp * (-1);
        weight_v_reg[i] = $shortrealtobits(real_tmp);
        weight_v = weight_v_reg[i];

        @(negedge clk);
    end
    
    in_valid_x = 0; in_valid_u = 0; in_valid_w = 0; in_valid_v = 0;
    data_x = 32'dx; weight_u = 32'dx; weight_w = 32'dx; weight_v = 32'dx;

end endtask

task gen_answer_task;
real h1, h2, h3;
real ux_real_tmp1, ux_real_tmp2, ux_real_tmp3;
real wh_real_tmp1, wh_real_tmp2, wh_real_tmp3;
real f_wh_ux_real_tmp1, f_wh_ux_real_tmp2, f_wh_ux_real_tmp3;
real vh_real_tmp1, vh_real_tmp2, vh_real_tmp3;
begin
    /* 1st round */
    ux_real_tmp1 = $bitstoshortreal(weight_u_reg[0])*$bitstoshortreal(data_x_reg[0]) +
                   $bitstoshortreal(weight_u_reg[1])*$bitstoshortreal(data_x_reg[1]) +
                   $bitstoshortreal(weight_u_reg[2])*$bitstoshortreal(data_x_reg[2]);
    ux_real_tmp2 = $bitstoshortreal(weight_u_reg[3])*$bitstoshortreal(data_x_reg[0]) +
                   $bitstoshortreal(weight_u_reg[4])*$bitstoshortreal(data_x_reg[1]) +
                   $bitstoshortreal(weight_u_reg[5])*$bitstoshortreal(data_x_reg[2]);
    ux_real_tmp3 = $bitstoshortreal(weight_u_reg[6])*$bitstoshortreal(data_x_reg[0]) +
                   $bitstoshortreal(weight_u_reg[7])*$bitstoshortreal(data_x_reg[1]) +
                   $bitstoshortreal(weight_u_reg[8])*$bitstoshortreal(data_x_reg[2]);
    f_wh_ux_real_tmp1 = 1/(1+E**(ux_real_tmp1*(-1)));
    f_wh_ux_real_tmp2 = 1/(1+E**(ux_real_tmp2*(-1)));
    f_wh_ux_real_tmp3 = 1/(1+E**(ux_real_tmp3*(-1)));
    h1 = f_wh_ux_real_tmp1; h2 = f_wh_ux_real_tmp2; h3 = f_wh_ux_real_tmp3;

    vh_real_tmp1 = $bitstoshortreal(weight_v_reg[0])*(h1) +
                   $bitstoshortreal(weight_v_reg[1])*(h2) +
                   $bitstoshortreal(weight_v_reg[2])*(h3);
    vh_real_tmp2 = $bitstoshortreal(weight_v_reg[3])*(h1) +
                   $bitstoshortreal(weight_v_reg[4])*(h2) +
                   $bitstoshortreal(weight_v_reg[5])*(h3);
    vh_real_tmp3 = $bitstoshortreal(weight_v_reg[6])*(h1) +
                   $bitstoshortreal(weight_v_reg[7])*(h2) +
                   $bitstoshortreal(weight_v_reg[8])*(h3);
    y_reg[0] = vh_real_tmp1>0 ? $shortrealtobits(vh_real_tmp1) : 0;
    y_reg[1] = vh_real_tmp2>0 ? $shortrealtobits(vh_real_tmp2) : 0;
    y_reg[2] = vh_real_tmp3>0 ? $shortrealtobits(vh_real_tmp3) : 0;
    
    /* 2nd round */
    wh_real_tmp1 = $bitstoshortreal(weight_w_reg[0])*h1 + $bitstoshortreal(weight_w_reg[1])*h2 + $bitstoshortreal(weight_w_reg[2])*h3;
    wh_real_tmp2 = $bitstoshortreal(weight_w_reg[3])*h1 + $bitstoshortreal(weight_w_reg[4])*h2 + $bitstoshortreal(weight_w_reg[5])*h3;
    wh_real_tmp3 = $bitstoshortreal(weight_w_reg[6])*h1 + $bitstoshortreal(weight_w_reg[7])*h2 + $bitstoshortreal(weight_w_reg[8])*h3;

    ux_real_tmp1 = $bitstoshortreal(weight_u_reg[0])*$bitstoshortreal(data_x_reg[3]) +
                   $bitstoshortreal(weight_u_reg[1])*$bitstoshortreal(data_x_reg[4]) +
                   $bitstoshortreal(weight_u_reg[2])*$bitstoshortreal(data_x_reg[5]);
    ux_real_tmp2 = $bitstoshortreal(weight_u_reg[3])*$bitstoshortreal(data_x_reg[3]) +
                   $bitstoshortreal(weight_u_reg[4])*$bitstoshortreal(data_x_reg[4]) +
                   $bitstoshortreal(weight_u_reg[5])*$bitstoshortreal(data_x_reg[5]);
    ux_real_tmp3 = $bitstoshortreal(weight_u_reg[6])*$bitstoshortreal(data_x_reg[3]) +
                   $bitstoshortreal(weight_u_reg[7])*$bitstoshortreal(data_x_reg[4]) +
                   $bitstoshortreal(weight_u_reg[8])*$bitstoshortreal(data_x_reg[5]);
    
    f_wh_ux_real_tmp1 = 1/(1+E**((wh_real_tmp1 + ux_real_tmp1)*(-1)));
    f_wh_ux_real_tmp2 = 1/(1+E**((wh_real_tmp2 + ux_real_tmp2)*(-1)));
    f_wh_ux_real_tmp3 = 1/(1+E**((wh_real_tmp3 + ux_real_tmp3)*(-1)));
    h1 = f_wh_ux_real_tmp1; h2 = f_wh_ux_real_tmp2; h3 = f_wh_ux_real_tmp3;

    vh_real_tmp1 = $bitstoshortreal(weight_v_reg[0])*(h1) +
                   $bitstoshortreal(weight_v_reg[1])*(h2) +
                   $bitstoshortreal(weight_v_reg[2])*(h3);
    vh_real_tmp2 = $bitstoshortreal(weight_v_reg[3])*(h1) +
                   $bitstoshortreal(weight_v_reg[4])*(h2) +
                   $bitstoshortreal(weight_v_reg[5])*(h3);
    vh_real_tmp3 = $bitstoshortreal(weight_v_reg[6])*(h1) +
                   $bitstoshortreal(weight_v_reg[7])*(h2) +
                   $bitstoshortreal(weight_v_reg[8])*(h3);
    y_reg[3] = vh_real_tmp1>0 ? $shortrealtobits(vh_real_tmp1) : 0;
    y_reg[4] = vh_real_tmp2>0 ? $shortrealtobits(vh_real_tmp2) : 0;
    y_reg[5] = vh_real_tmp3>0 ? $shortrealtobits(vh_real_tmp3) : 0;

    /* 3rd round */
    wh_real_tmp1 = $bitstoshortreal(weight_w_reg[0])*h1 + $bitstoshortreal(weight_w_reg[1])*h2 + $bitstoshortreal(weight_w_reg[2])*h3;
    wh_real_tmp2 = $bitstoshortreal(weight_w_reg[3])*h1 + $bitstoshortreal(weight_w_reg[4])*h2 + $bitstoshortreal(weight_w_reg[5])*h3;
    wh_real_tmp3 = $bitstoshortreal(weight_w_reg[6])*h1 + $bitstoshortreal(weight_w_reg[7])*h2 + $bitstoshortreal(weight_w_reg[8])*h3;

    ux_real_tmp1 = $bitstoshortreal(weight_u_reg[0])*$bitstoshortreal(data_x_reg[6]) +
                   $bitstoshortreal(weight_u_reg[1])*$bitstoshortreal(data_x_reg[7]) +
                   $bitstoshortreal(weight_u_reg[2])*$bitstoshortreal(data_x_reg[8]);
    ux_real_tmp2 = $bitstoshortreal(weight_u_reg[3])*$bitstoshortreal(data_x_reg[6]) +
                   $bitstoshortreal(weight_u_reg[4])*$bitstoshortreal(data_x_reg[7]) +
                   $bitstoshortreal(weight_u_reg[5])*$bitstoshortreal(data_x_reg[8]);
    ux_real_tmp3 = $bitstoshortreal(weight_u_reg[6])*$bitstoshortreal(data_x_reg[6]) +
                   $bitstoshortreal(weight_u_reg[7])*$bitstoshortreal(data_x_reg[7]) +
                   $bitstoshortreal(weight_u_reg[8])*$bitstoshortreal(data_x_reg[8]);
    
    f_wh_ux_real_tmp1 = 1/(1+E**((wh_real_tmp1 + ux_real_tmp1)*(-1)));
    f_wh_ux_real_tmp2 = 1/(1+E**((wh_real_tmp2 + ux_real_tmp2)*(-1)));
    f_wh_ux_real_tmp3 = 1/(1+E**((wh_real_tmp3 + ux_real_tmp3)*(-1)));
    h1 = f_wh_ux_real_tmp1; h2 = f_wh_ux_real_tmp2; h3 = f_wh_ux_real_tmp3;

    vh_real_tmp1 = $bitstoshortreal(weight_v_reg[0])*(h1) +
                   $bitstoshortreal(weight_v_reg[1])*(h2) +
                   $bitstoshortreal(weight_v_reg[2])*(h3);
    vh_real_tmp2 = $bitstoshortreal(weight_v_reg[3])*(h1) +
                   $bitstoshortreal(weight_v_reg[4])*(h2) +
                   $bitstoshortreal(weight_v_reg[5])*(h3);
    vh_real_tmp3 = $bitstoshortreal(weight_v_reg[6])*(h1) +
                   $bitstoshortreal(weight_v_reg[7])*(h2) +
                   $bitstoshortreal(weight_v_reg[8])*(h3);
    y_reg[6] = vh_real_tmp1>0 ? $shortrealtobits(vh_real_tmp1) : 0;
    y_reg[7] = vh_real_tmp2>0 ? $shortrealtobits(vh_real_tmp2) : 0;
    y_reg[8] = vh_real_tmp3>0 ? $shortrealtobits(vh_real_tmp3) : 0;

end endtask

task wait_outvalid;
integer cycles;
begin
	cycles = -1;
	while(out_valid === 0)begin
		cycles = cycles + 1;

		// SPEC4. The out should be reset whenever your out_valid isn’t high.  
		if(out_valid==0 & out>0) YOU_FAIL(4);
        // SPEC6. The execution latency is limited in 3000 cycles. 
        if(cycles == 3000) YOU_FAIL(6);
	    
        @(negedge clk);
	end
	total_cycles = total_cycles + (cycles+1); // +1, since the 1st out count for 1 cycle.
end endtask


task delay_task ;
integer delay_gap;
begin
	delay_gap = $urandom_range(3, 4);
	// SPEC4. The out should be reset whenever your out_valid isn’t high.  
	if(out_valid==0 & out>0) YOU_FAIL(4);
	
    repeat(delay_gap)@(negedge clk);
end endtask


// YOU_FAIL(8) : SPEC8-3. If the guy jumps to the same height, out must be 2’b00 for 1 cycle
task check_ans ;
integer cnt;
begin
    cnt = 0;
    while(out_valid == 1 & cnt < 9) begin
        diff = $bitstoshortreal(out) - $bitstoshortreal(y_reg[cnt]);
        if(diff > 0.0005 || diff < -0.0005)
            YOU_FAIL(8);
        else
            cnt = cnt + 1;
        $display("difference : %d", diff);
        @(negedge clk);
	end
    if(out_valid == 1) YOU_FAIL(7);
end
endtask

task YOU_PASS_task;begin
    $display ("----------------------------------------------------------------------------------------------------------------------");
    $display ("                                                  Congratulations!                						             ");
    $display ("                                           You have passed all patterns!          						             ");
    $display ("                                                                                 						             ");
    $display ("                                        Your execution cycles   = %5d cycles      						             ", total_cycles);
    $display ("----------------------------------------------------------------------------------------------------------------------");

    $finish;	
end endtask

endmodule
