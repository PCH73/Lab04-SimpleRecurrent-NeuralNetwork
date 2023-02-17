module NN(
	// Input signals
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
	// Output signals
	out_valid,
	out
);

//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------

// IEEE floating point paramenters
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch = 2;
parameter inst_arch_type = 0;
parameter inst_rnd = 3'b000;
parameter S_IDLE = 2'd0;
parameter S_IN   = 2'd1;
parameter S_OUT  = 2'd2;
wire [7:0] status_inst;

//---------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION
//---------------------------------------------------------------------
input  clk, rst_n, in_valid_u, in_valid_w, in_valid_v, in_valid_x;
input [inst_sig_width+inst_exp_width:0] weight_u, weight_w, weight_v;
input [inst_sig_width+inst_exp_width:0] data_x;
output reg	out_valid;
output reg [inst_sig_width+inst_exp_width:0] out;

//================================================================
//  integer / genvar / parameters
//================================================================
// 
integer i;
genvar gen_i;
//  
parameter FP_ZERO = 32'b0_0000_0000_00000000000000000000000 ;
parameter FP_ONE = 32'b0_0111_1111_00000000000000000000000 ;
reg [1:0] c_state,n_state;
reg [22:0] cycle;
reg [5:0]count;
reg [3:0]count1;

//---------------------------------------------------------------------
//   WIRE AND REG DECLARATION
//---------------------------------------------------------------------
//input
reg [inst_sig_width+inst_exp_width:0] u_r[0:8],w_r[0:8],v_r[0:8],x_r[0:8],y1_r[0:2],y2_r[0:2],y3_r[0:2];
reg [inst_sig_width+inst_exp_width:0] h_r[0:2];
reg  [inst_sig_width+inst_exp_width:0] dpa[0:8],dpb[0:2];
reg  [inst_sig_width+inst_exp_width:0] exp[0:2];
reg  [inst_sig_width+inst_exp_width:0] suma[0:2],sumb[0:2];
reg  [inst_sig_width+inst_exp_width:0] inv[0:2];
reg  [inst_sig_width+inst_exp_width:0] ReLU_r[0:2];
wire [inst_sig_width+inst_exp_width:0] dpz[0:2];
wire [inst_sig_width+inst_exp_width:0] sumz[0:2];
wire [inst_sig_width+inst_exp_width:0] expz[0:2];
wire [inst_sig_width+inst_exp_width:0] invz[0:2];
wire [inst_sig_width+inst_exp_width:0] ReLU_w[0:2];

DW_fp_dp3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type) DP1(.a(dpa[0]), .b(dpb[0]), .c(dpa[1]),.d(dpb[1]),.e(dpa[2]),.f(dpb[2]),.rnd(inst_rnd), .z(dpz[0]), .status(status_inst));
DW_fp_dp3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type) DP2(.a(dpa[3]), .b(dpb[0]), .c(dpa[4]),.d(dpb[1]),.e(dpa[5]),.f(dpb[2]),.rnd(inst_rnd), .z(dpz[1]), .status(status_inst));
DW_fp_dp3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type) DP3(.a(dpa[6]), .b(dpb[0]), .c(dpa[7]),.d(dpb[1]),.e(dpa[8]),.f(dpb[2]),.rnd(inst_rnd), .z(dpz[2]), .status(status_inst));
DW_fp_exp #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch) EXP1(.a(exp[0]), .z(expz[0]), .status(status_inst));
DW_fp_exp #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch) EXP2(.a(exp[1]), .z(expz[1]), .status(status_inst));
DW_fp_exp #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch) EXP3(.a(exp[2]), .z(expz[2]), .status(status_inst));
DW_fp_add #(inst_sig_width, inst_exp_width, inst_ieee_compliance) ADD1(.a(suma[0]), .b(sumb[0]), .rnd(inst_rnd), .z(sumz[0]), .status(status_inst));
DW_fp_add #(inst_sig_width, inst_exp_width, inst_ieee_compliance) ADD2(.a(suma[1]), .b(sumb[1]), .rnd(inst_rnd), .z(sumz[1]), .status(status_inst));
DW_fp_add #(inst_sig_width, inst_exp_width, inst_ieee_compliance) ADD3(.a(suma[2]), .b(sumb[2]), .rnd(inst_rnd), .z(sumz[2]), .status(status_inst));
DW_fp_recip #(inst_sig_width, inst_exp_width, inst_ieee_compliance,0) inv1(.a(inv[0]),.rnd(inst_rnd), .z(invz[0]), .status(status_inst));
DW_fp_recip #(inst_sig_width, inst_exp_width, inst_ieee_compliance,0) inv2(.a(inv[1]),.rnd(inst_rnd), .z(invz[1]), .status(status_inst));
DW_fp_recip #(inst_sig_width, inst_exp_width, inst_ieee_compliance,0) inv3(.a(inv[2]),.rnd(inst_rnd), .z(invz[2]), .status(status_inst));

always@(posedge clk or negedge rst_n) 
begin
	if(!rst_n) c_state<=S_IDLE;
	else c_state<=n_state;
end

always@(*) 
begin
	case(c_state)
	S_IDLE:
		begin
		if(in_valid_x) n_state=S_IN;
		else n_state = S_IDLE;
		end
	S_IN:
		begin
		if(cycle[22]) n_state=S_OUT;
		else n_state=S_IN;
		end
	S_OUT:
		begin
		if(count1>8) n_state=S_IDLE;
		else n_state=S_OUT;
		end
	default:n_state=S_IDLE;
	endcase
end

//==============================================//
//                  Count                       //
//==============================================//
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) count<=0;
	else if(n_state==S_IDLE) count<=0;
	else if(n_state==S_IN) count<=count+1;
end

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) 
	begin
		for(i=0;i<23;i=i+1)
		cycle[i]<=0;
	end
	else if(n_state == S_IDLE)
	begin
		for(i=0;i<23;i=i+1)
			cycle[i]<=0;
	end
	else if(n_state == S_IN)
	begin
		cycle[0]<=in_valid_u;
		if(!in_valid_u)
		begin
			for(i=1;i<23;i=i+1)
			cycle[i]<=cycle[i-1];
		end
	end
end

//==============================================//
//                  Input Block                 //
//==============================================//
always @(posedge clk or negedge rst_n) 
begin
	if(!rst_n) 
	begin
		for(i=0;i<9;i=i+1)
		w_r[i]<=0;
	end
	else if(n_state == S_IDLE)
	begin
		for(i=0;i<9;i=i+1)
		w_r[i]<=0;
	end
	else if(n_state==S_IN)
	begin
		if(count==0) w_r[0]<=weight_w;
		else if(count==1) w_r[1]<=weight_w;
		else if(count==2) w_r[2]<=weight_w;
		else if(count==3) w_r[3]<=weight_w;
		else if(count==4) w_r[4]<=weight_w;
		else if(count==5) w_r[5]<=weight_w;
		else if(count==6) w_r[6]<=weight_w;
		else if(count==7) w_r[7]<=weight_w;
		else if(count==8) w_r[8]<=weight_w;
	end
end

always @(posedge clk or negedge rst_n) 
begin
	if(!rst_n) 
	begin
		for(i=0;i<9;i=i+1)
		u_r[i]<=0;
	end
	else if(n_state == S_IDLE)
	begin
		for(i=0;i<9;i=i+1)
		u_r[i]<=0;
	end
	else if(n_state==S_IN)
	begin
		if(count==0) u_r[0]<=weight_u;
		else if(count==1) u_r[1]<=weight_u;
		else if(count==2) u_r[2]<=weight_u;
		else if(count==3) u_r[3]<=weight_u;
		else if(count==4) u_r[4]<=weight_u;
		else if(count==5) u_r[5]<=weight_u;
		else if(count==6) u_r[6]<=weight_u;
		else if(count==7) u_r[7]<=weight_u;
		else if(count==8) u_r[8]<=weight_u;
	end
end

always @(posedge clk or negedge rst_n) 
begin
	if(!rst_n) 
	begin
		for(i=0;i<9;i=i+1)
		v_r[i]<=0;
	end
	else if(n_state == S_IDLE)
	begin
		for(i=0;i<9;i=i+1)
		v_r[i]<=0;
	end
	else if(n_state==S_IN)
	begin
		if(count==0) v_r[0]<=weight_v;
		else if(count==1) v_r[1]<=weight_v;
		else if(count==2) v_r[2]<=weight_v;
		else if(count==3) v_r[3]<=weight_v;
		else if(count==4) v_r[4]<=weight_v;
		else if(count==5) v_r[5]<=weight_v;
		else if(count==6) v_r[6]<=weight_v;
		else if(count==7) v_r[7]<=weight_v;
		else if(count==8) v_r[8]<=weight_v;
	end
end

always @(posedge clk or negedge rst_n) 
begin
	if(!rst_n) 
	begin
		for(i=0;i<9;i=i+1)
		x_r[i]<=0;
	end
	else if(n_state == S_IDLE)
	begin
		for(i=0;i<9;i=i+1)
		x_r[i]<=0;
	end
	else if(n_state==S_IN)
	begin
		if(count==0) x_r[0]<=data_x;
		else if(count==1) x_r[1]<=data_x;
		else if(count==2) x_r[2]<=data_x;
		else if(count==3) x_r[3]<=data_x;
		else if(count==4) x_r[4]<=data_x;
		else if(count==5) x_r[5]<=data_x;
		else if(count==6) x_r[6]<=data_x;
		else if(count==7) x_r[7]<=data_x;
		else if(count==8) x_r[8]<=data_x;
	end
end

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) h_r[gen_i]<=0;
			else if(n_state==S_IDLE) h_r[gen_i]<=0;
			else if(cycle[5]) h_r[gen_i]<=invz[gen_i];
			else if(cycle[12]) h_r[gen_i]<=invz[gen_i];
		end
	end
endgenerate

generate
	for(gen_i=0;gen_i<9;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) dpa[gen_i]<=0;
			else if(n_state==S_IDLE) dpa[gen_i]<=0;
			else if(cycle[1])  dpa[gen_i]<=u_r[gen_i];
			else if(cycle[6])  dpa[gen_i]<=u_r[gen_i];
			else if(cycle[13]) dpa[gen_i]<=u_r[gen_i];
			else if(cycle[2])  dpa[gen_i]<=w_r[gen_i];
			else if(cycle[7])  dpa[gen_i]<=w_r[gen_i];
			else if(cycle[14]) dpa[gen_i]<=w_r[gen_i];
			else if(cycle[5])  dpa[gen_i]<=v_r[gen_i];
			else if(cycle[12]) dpa[gen_i]<=v_r[gen_i];
			else if(cycle[19]) dpa[gen_i]<=v_r[gen_i];
		end
	end
endgenerate

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) dpb[gen_i]<=0;
			else if(n_state==S_IDLE) dpb[gen_i]<=0;
			else if(cycle[1])   dpb[gen_i]<=x_r[gen_i];
			else if(cycle[6])   dpb[gen_i]<=x_r[gen_i+3];
			else if(cycle[13])  dpb[gen_i]<=x_r[gen_i+6];
			else if(cycle[5])   dpb[gen_i]<=invz[gen_i];
			else if(cycle[12])  dpb[gen_i]<=invz[gen_i];
			else if(cycle[19])  dpb[gen_i]<=invz[gen_i];
			else if(cycle[7])   dpb[gen_i]<=h_r[gen_i];
			else if(cycle[14])  dpb[gen_i]<=h_r[gen_i];
			
			
		end
	end
endgenerate

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) suma[gen_i]<=0;
			else if(n_state==S_IDLE) suma[gen_i]<=0;
			else if(cycle[3])  suma[gen_i]<=expz[gen_i];
			else if(cycle[10]) suma[gen_i]<=expz[gen_i];
			else if(cycle[17]) suma[gen_i]<=expz[gen_i];
			else if(cycle[7])  suma[gen_i]<=dpz[gen_i];
			else if(cycle[14]) suma[gen_i]<=dpz[gen_i];
		end
	end
endgenerate

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) sumb[gen_i]<=0;
			else if(n_state==S_IDLE) sumb[gen_i]<=0;
			else if(cycle[3])  sumb[gen_i]<=FP_ONE;
			else if(cycle[10]) sumb[gen_i]<=FP_ONE;			
			else if(cycle[17]) sumb[gen_i]<=FP_ONE;
			else if(cycle[8])  sumb[gen_i]<=dpz[gen_i];
			else if(cycle[15]) sumb[gen_i]<=dpz[gen_i];
		end
	end
endgenerate

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) exp[gen_i]<=0;
			else if(n_state==S_IDLE) exp[gen_i]<=0;
			else if(cycle[2])  exp[gen_i]<={{~{dpz[gen_i][31]}},{dpz[gen_i][30:0]}};
			else if(cycle[9])  exp[gen_i]<={{~{sumz[gen_i][31]}},{sumz[gen_i][30:0]}};
			else if(cycle[16]) exp[gen_i]<={{~{sumz[gen_i][31]}},{sumz[gen_i][30:0]}};
		end
	end
endgenerate

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) inv[gen_i]<=0;
			else if(n_state==S_IDLE) inv[gen_i]<=0;
			else if(cycle[4])  inv[gen_i]<=sumz[gen_i];
			else if(cycle[11]) inv[gen_i]<=sumz[gen_i];
			else if(cycle[18]) inv[gen_i]<=sumz[gen_i];
		end
	end
endgenerate

generate
for(gen_i=0;gen_i<3;gen_i=gen_i+1)
begin
	assign ReLU_w[gen_i] =(dpz[gen_i][31]==1) ? FP_ZERO : dpz[gen_i];
end
endgenerate

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) ReLU_r[gen_i]<=0;
			else if(n_state==S_IDLE) ReLU_r[gen_i]<=0;
			else if(cycle[6])  ReLU_r[gen_i]<=ReLU_w[gen_i];
			else if(cycle[13]) ReLU_r[gen_i]<=ReLU_w[gen_i];
			else if(cycle[20]) ReLU_r[gen_i]<=ReLU_w[gen_i];
		end
	end
endgenerate

generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) 
	begin
		always@(posedge clk or negedge rst_n) 
		begin
			if(!rst_n) 
			begin
				y1_r[gen_i]<=0;
				y2_r[gen_i]<=0;
				y3_r[gen_i]<=0;
			end
			else if(n_state==S_IDLE)
			begin
				y1_r[gen_i]<=0;
				y2_r[gen_i]<=0;
				y3_r[gen_i]<=0;
			end
			else if(cycle[7])  y1_r[gen_i]<=ReLU_r[gen_i];
			else if(cycle[15]) y2_r[gen_i]<=ReLU_r[gen_i];
			else if(cycle[21]) y3_r[gen_i]<=ReLU_r[gen_i];
		end
	end
endgenerate

//==============================================//
//                Output Block                  //
//==============================================//
always@(posedge clk or negedge rst_n) 
begin
	if(!rst_n) out_valid<=0;
	else if( n_state==S_IDLE) out_valid<=0;
	else if(n_state==S_OUT) out_valid<=1;
end

always@(posedge clk or negedge rst_n) 
begin
	if(!rst_n) count1<=0;
	else if(n_state==S_IDLE) count1<=0;
	else if(n_state==S_OUT) count1<=count1+1;
end

always@(posedge clk or negedge rst_n) 
begin
	if(!rst_n) out<=0;
	else if( n_state==S_IDLE) out<=0;
	else if(n_state==S_OUT)
	begin
		if(count1==0) out<=y1_r[0];
		else if(count1==1) out<=y1_r[1];
		else if(count1==2) out<=y1_r[2];
		else if(count1==3) out<=y2_r[0];
		else if(count1==4) out<=y2_r[1];
		else if(count1==5) out<=y2_r[2];
		else if(count1==6) out<=y3_r[0];
		else if(count1==7) out<=y3_r[1];
		else if(count1==8) out<=y3_r[2];
	end
end
endmodule