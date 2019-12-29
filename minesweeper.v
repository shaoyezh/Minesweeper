module final_project(
		CLOCK_50,		//On Board 50 MHz
		KEY,
		SW,
		VGA_CLK,		//VGA Clock
		VGA_HS,			//VGA H_SYNC
		VGA_VS,			//VGA V_SYNC
		VGA_BLANK_N,	//VGA BLANK
		VGA_SYNC_N,		//VGA SYNC
		VGA_R,			//VGA Red[9:0]
		VGA_G,			//VGA Green[9:0]
		VGA_B,			//VGA Blue[9:0]
		HEX1,
		HEX0,
		HEX5,
		HEX4,
		HEX3,
		HEX2
		);
	input CLOCK_50;
	input [3:0] KEY;
	input [9:0] SW;
	output [6:0] HEX0;
	output [6:0] HEX5;
	output [6:0] HEX4;
	output [6:0] HEX2;
	output [6:0] HEX3;
	output [6:0] HEX1;
	output			VGA_CLK;		//	VGA Clock
	output			VGA_HS;			//	VGA H_SYNC
	output			VGA_VS;			//	VGA V_SYNC
	output			VGA_BLANK_N;	//	VGA BLANK
	output			VGA_SYNC_N;		//	VGA SYNC
	output	[9:0]	VGA_R;			//	VGA Red[9:0]
	output	[9:0]	VGA_G;			//	VGA Green[9:0]
	output	[9:0]	VGA_B;			//	VGA Blue[9:0]
	wire [3:0] memory_out;
	wire [3:0] data;
	wire [5:0] address;
	wire writeRam, ld_c, select, flag, clock,choose_draw, writeEn;
	wire [2:0] colour;
	wire [6:0] draw_board_x;
	wire [6:0] draw_board_y;
	wire [6:0] draw_x;
	wire [6:0] draw_y;
	wire [6:0] x;
	wire [6:0] y;
	wire [5:0] ram_address, random_address;
	wire choose_address;
	wire [4:0] numMine;
	
	wire [63:0] mineMapIn;
	vga_adapter VGA(
			.resetn(KEY[0] || over),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
	wire enable;
	wire [5:0] index0, index1, index2, index3, index4, index5, index6, index7;
	reg [2:0] counter3;
	
	// PICKER FOR GENERATE_MINE
	always @ (posedge CLOCK_50) begin
		if(!KEY[0]) begin
			counter3 = 3'b000;
			end
		else begin
			counter3 = counter3 + 1'b1;
		end
	end
	
	// mux2to1 for choosing ram_address 
	/* generate_mine gm(.clk(CLOCK_50), .resetn(KEY[0] || over), .en(1), 
				.index0(index0),
				.index1(index1),
				.index2(index2),
				.index3(index3),
				.index4(index4),
				.index5(index5),
				.index6(index6),
				.index7(index7)); */
	/* mux8to1 m4(.index0(index0),
				.index1(index1),
				.index2(index2),
				.index3(index3),
				.index4(index4),
				.index5(index5),
				.index6(index6),
				.index7(index7),
				.picker(counter3),
				.index_out(random_address)); */
	/* mux2to1 m3(.x(random_address), .y(ram_address), .s(choose_address), .m(address)); */
	generate_mine gm(
      .clk(CLOCK_50),
      .resetn(resetn),
      .en(1'b1),
      .mineOut(mineMapIn)
      );
	RateDivider rd0(.clk(CLOCK_50), .resetn(KEY[0] || over), .enable(enable));
	board_drawer drawer(.clk(CLOCK_50), .resetn(KEY[0] || over), 
						.enable(enable), .x_out(draw_board_x), .y_out(draw_board_y));
	mux2to1 m0(.x(draw_board_x), .y(draw_x), .s(choose_draw), .m(x));
	mux2to1 m1(.x(draw_board_y), .y(draw_y), .s(choose_draw), .m(y));
	//choose_clk cc(.x(KEY[1]), .y(KEY[2]), .z(clock));
	ram64x4 ram(.address(ram_address), .clock(clock), .data(data), .wren(writeRam), .q(memory_out));
	control c0(.clk(CLOCK_50), 
			.resetn(KEY[0] || over), 
			.select_key(!KEY[1]), 
			.flag_key(!KEY[2]), 
			.load_c(ld_c), 
			.writeRam(writeRam), 
			.select(select),
			.flag(flag),
			.choose_draw(choose_draw),
			.writeEn(writeEn),
			.choose_address(choose_address),
			.clock(clock)
			);
			
	datapath d0(.clk(CLOCK_50),
		.MMin(mineMapIn),
		.x_in(SW[6:4]),
		.y_in(SW[2:0]),
		.choose_draw(choose_draw),
		.ram_address(ram_address),
		.resetn(KEY[0] || over),
		.ld_c(ld_c),
		.select(select),
		.flag(flag),
		.ram_input(memory_out),
		.data_out(data),
		.color_out(colour),
		.x_out(draw_x),
		.y_out(draw_y),
		.game_over(over),
		.numMine(numMine)
		);
	
	// num mine
	hex_decoder H5(.hex_digit(numMine),
		.segments(HEX1));
		
	hex_decoder H1(.hex_digit(SW[2:0]),
		.segments(HEX5));
	
	hex_decoder H2(.hex_digit(SW[6:4]),
		.segments(HEX4));
	
	hex_decoder H3(.hex_digit(x),
		.segments(HEX3));
		
	hex_decoder H4(.hex_digit(colour),
		.segments(HEX2));
	
	hex_decoder H0(
		.hex_digit(memory_out[3:0]),
		.segments(HEX0)
		);
	//mux2to1 m0(.x(selection), .y(CLOCK_50), .s(set_ram), .m(clock));
	//mux2to1 m1(.x(KEY[2]), .y(KEY[1]), .s(select), .m(clock));
endmodule


module choose_clk(
	input x,
	input y,
	output z);
	
	assign z = x && y ? 1'b1 : 1'b0;
endmodule
	
module control(
	input clk, resetn, select_key, flag_key,
	output reg load_c, writeRam, select, flag, writeEn, choose_address, clock,
	output choose_draw);
	reg [3:0] current_state, next_state;
	reg draw_board;

	localparam  START = 4'b0000,
				START_WAIT_FLAG = 4'b0001,
				START_WAIT_SELECT = 4'b0010,
				SELECT = 4'b0100,
				SELECT_WAIT = 4'b0101,
				FLAG = 4'b0110,
				FLAG_WAIT = 4'b0111,
				SET_BOARD = 4'b1000,
				SET_BOARD_WAIT = 4'b1001,
				PREPARE = 4'b1010,
				PREPARE_WAIT = 4'b1011;
	//reset
	always @(posedge clk) begin
		if (!resetn)
			current_state <= SET_BOARD;
		else
			current_state <= next_state;
	end
	//state table
	always @(*) 
	begin: state_table
		case (current_state)
			SET_BOARD :	next_state = select_key ? SET_BOARD_WAIT : SET_BOARD;
			SET_BOARD_WAIT : next_state = select_key ? SET_BOARD_WAIT : PREPARE;
			PREPARE : next_state = select_key ? PREPARE_WAIT : PREPARE;
			PREPARE_WAIT : next_state = select_key ? PREPARE_WAIT : START;
			START: begin
				if (select_key == 1'b1 && flag_key == 1'b0) begin
					next_state = START_WAIT_SELECT;
				end
				else if (flag_key == 1'b1 && select_key == 1'b0) begin
					next_state = START_WAIT_FLAG;
				end
				else begin
					next_state = START;
				end
			end
			START_WAIT_SELECT : next_state = select_key ? START_WAIT_SELECT: SELECT;
			SELECT: next_state = select_key ? SELECT_WAIT : SELECT;
			SELECT_WAIT: next_state = select_key ? SELECT_WAIT : PREPARE;
			START_WAIT_FLAG: next_state = flag_key ? START_WAIT_FLAG: FLAG;
			FLAG: next_state = flag_key ? FLAG_WAIT : FLAG;
			FLAG_WAIT : next_state = flag_key ? FLAG_WAIT : PREPARE;
			//default: next_state = SET_BOARD;
		endcase
	end
	
	always @(*)
	begin
		load_c = 0;
		writeEn = 0;
		writeRam = 0;
		select = 0;
		flag = 0;
		draw_board = 1;
		choose_address = 1;
		clock = 0;
		//reset_ram = 0;
		case (current_state)
			SET_BOARD: begin
				draw_board = 0;
				writeEn = 1;
				choose_address = 0;
				//writeRam = 1;
				//reset_ram = 1;
				end
			SET_BOARD_WAIT: begin
				draw_board = 0;
				writeEn = 1;
				choose_address = 0;
				clock = 1;
				//writeRam = 1;
				//reset_ram = 1
				end
			START : begin 
				load_c = 1;
				end
			START_WAIT_SELECT : begin
				load_c = 1;
				clock = 1;
				end
			START_WAIT_FLAG : begin
				load_c = 1;
				clock = 1;
				end
			SELECT: begin
				writeEn = 1;
				select = 1;
				writeRam = 1;
				end
			SELECT_WAIT: begin
				writeEn = 1;
				writeRam = 1;
				select = 1;
				clock = 1;
				end
			FLAG: begin
				writeEn = 1;
				flag = 1;
				writeRam = 1;
				end
			FLAG_WAIT: begin
				writeEn = 1;
				writeRam = 1;
				flag = 1;
				clock = 1;
				end
		endcase
	end
	assign choose_draw = (draw_board == 1) ? 1'b1 : 1'b0;
endmodule 

module datapath(
	input clk, resetn, ld_c, select, flag, choose_draw,
	input [63:0] MMin,
	input [3:0] ram_input,
	input [2:0] x_in,
	input [2:0] y_in, // x = SW[5:3], y = SW [2:0]
	output [5:0] ram_address, 
	output [3:0] data_out,
	output [6:0] x_out,
	output [6:0] y_out,
	output [2:0] color_out,
	// output reg choose_address, 
	output reg game_over,
	output reg [3:0] numMine
	);
	//reg [5:0] address;
	reg [3:0] data;
	reg [3:0] temp_data;
	reg [2:0] temp_x;
	reg [2:0] temp_y;
	reg [2:0] color;
	reg [5:0] counter;
	reg [5:0] curr_location;// address of current loaded square
	//ram_reset_counter ram_c(.clk(clk), reset_ram(resetn), .address_out(address_out));
	//ram_128x2 ram(.address(address), .clk(clk), .data(data), .wren(writeRam), .q(data_out));
	
	always @(posedge clk) begin
		if (!resetn) begin
			//address <= 6'b0;
			temp_x <= 6'b0;
			temp_y <= 6'b0;
			color <= 3'b111;
			temp_data <= 4'b0;
			game_over <= 1'b1;
			end
		else begin
			/* if (reset_ram == 1) begin
				choose_address = 1; */
			if (choose_draw == 0) begin
				//data <= 4'b0100;
				color <= 3'b111;
				end
			if (ld_c) begin
				temp_x <= x_in; //register
				temp_y <= y_in; //register
				temp_data <= ram_input;
				color <= 3'b100;
				curr_location <= temp_x + 8 * temp_y;
				if (MMin[curr_location] == 1)  // check mine 
					game_over <= 0;
				else begin 
					numMine <= MMin[curr_location - 1'd1] 
					+ MMin[curr_location + 1'd1] 
					+ MMin[curr_location - 4'd8] 
					+ MMin[curr_location + 4'd8] 
					+ MMin[curr_location - 3'd7] 
					+ MMin[curr_location - 4'd9] 
					+ MMin[curr_location + 3'd7] 
					+ MMin[curr_location + 4'd9] ;
					end
				end
			if (select) begin
				data <=  {temp_data[3:1], 1'b1};
				if(temp_data[1] == 1)
					color <= 3'b001;
				else  
				color <= 3'b010;
				end
			if (flag) begin
				if(temp_data[0] == 0)begin
					color <= 3'b001;
					data <=  {temp_data[3:2], 2'b11};
					end
				else begin
					data <= temp_data;
					if (temp_data[1] == 1)
						color <= 3'b001;
					else color <= 3'b010;
					end
				end
			end	
		end
	
	always @(posedge clk) begin
		if (!resetn)
			counter <= 6'b000000;
		else
			if (counter == 111111)
				counter <= 6'b000000;
			else
				counter <= counter + 1'b1;
	end
	
	/* always @ (posedge clk) begin
		curr_location <= temp_x + 8 * temp_y;
		if (MMin[curr_location] == 1)  // check mine 
			game_over <= 0;
		else begin 
			numMine <= MMin[curr_location - 1'd1] 
			+ MMin[curr_location + 1'd1] 
			+ MMin[curr_location - 4'd8] 
			+ MMin[curr_location + 4'd8] 
			+ MMin[curr_location - 3'd7] 
			+ MMin[curr_location - 4'd9] 
			+ MMin[curr_location + 3'd7] 
			+ MMin[curr_location + 4'd9] ;
			end
		end */
		
	assign ram_address = temp_x + 8 * temp_y; // x_in and y_in
	assign data_out = data;
	assign x_out = 9*temp_x + counter[2:0];
	assign y_out = 9*temp_y + counter[5:3];
	assign color_out = color;
endmodule

module ram64x4 (
	address,
	clock,
	data,
	wren,
	q);

	input	[5:0]  address;
	input	  clock;
	input	[3:0]  data;
	input	  wren;
	output	[3:0]  q;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
	tri1	  clock;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

	wire [3:0] sub_wire0;
	wire [3:0] q = sub_wire0[3:0];

	altsyncram	altsyncram_component (
				.address_a (address),
				.clock0 (clock),
				.data_a (data),
				.wren_a (wren),
				.q_a (sub_wire0),
				.aclr0 (1'b0),
				.aclr1 (1'b0),
				.address_b (1'b1),
				.addressstall_a (1'b0),
				.addressstall_b (1'b0),
				.byteena_a (1'b1),
				.byteena_b (1'b1),
				.clock1 (1'b1),
				.clocken0 (1'b1),
				.clocken1 (1'b1),
				.clocken2 (1'b1),
				.clocken3 (1'b1),
				.data_b (1'b1),
				.eccstatus (),
				.q_b (),
				.rden_a (1'b1),
				.rden_b (1'b1),
				.wren_b (1'b0));
	defparam
		altsyncram_component.clock_enable_input_a = "BYPASS",
		altsyncram_component.clock_enable_output_a = "BYPASS",
		altsyncram_component.intended_device_family = "Cyclone V",
		altsyncram_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
		altsyncram_component.lpm_type = "altsyncram",
		altsyncram_component.numwords_a = 64,
		altsyncram_component.operation_mode = "SINGLE_PORT",
		altsyncram_component.outdata_aclr_a = "NONE",
		altsyncram_component.outdata_reg_a = "UNREGISTERED",
		altsyncram_component.power_up_uninitialized = "FALSE",
		altsyncram_component.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
		altsyncram_component.widthad_a = 6,
		altsyncram_component.width_a = 4,
		altsyncram_component.width_byteena_a = 1;


endmodule

module generate_mine(
  input clk, resetn, en,
  output reg [63:0] mineOut
  );
  wire [5:0] index0, index1, index2, index3, index4, index5, index6, index7;
  lfsr r0(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000000),
    .out(index0)
    );
  lfsr r1(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000001),
    .out(index1)
    );
  lfsr r2(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000010),
    .out(index2)
    );
  lfsr r3(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000011),
    .out(index3)
    );
  lfsr r4(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000100),
    .out(index4)
    );
  lfsr r5(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000101),
    .out(index5)
    );
  lfsr r6(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000110),
    .out(index6)
    );
  lfsr r7(
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .init(6'b000111),
    .out(index7)
    );
  always @(posedge clk, negedge resetn)
  begin
    if (!resetn)
      mineOut <= 64'b0;
    else
      mineOut <= 64'b0;
      mineOut[index0] <= 1;
      mineOut[index1] <= 1;
      mineOut[index2] <= 1;
      mineOut[index3] <= 1;
      mineOut[index4] <= 1;
      mineOut[index5] <= 1;
      mineOut[index6] <= 1;
      mineOut[index7] <= 1;
  end
endmodule

module lfsr(
  input clk, resetn, en,
  input [5:0] init,
  output reg [5:0] out
  );
  wire feedback;
  assign feedback = ~(out[5] ^ out[4]);

  always @(posedge clk, negedge resetn)
  begin
    if (!resetn)
      out <= init;
    else if (en)
      out <= {out[4:0], feedback};
    else
      out <= out;
  end
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;

    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;
            default: segments = 7'h7f;
        endcase
endmodule

module board_drawer(
	input clk, resetn, enable,
	output [6:0] x_out,
	output [6:0] y_out
);
	reg [5:0] counter1;
	reg [5:0] counter2;
	//wire enable;
	
	//RateDivider d0(.clk(clk), .enable(enable));
	
	always @(posedge clk)
	begin
		if(!resetn) begin
			counter1 <= 6'b000000;
		end
		else if (enable) begin
			if (counter1 == 111111)
				counter1 <= 6'b000000;
			else
				counter1 <= counter1 + 1'b1;
		end
	end

	always @(posedge clk)
	begin
		if(!resetn) begin
			counter2 <= 6'b000000;
		end
		else begin
			if (counter2 == 111111)
				counter2 <= 6'b000000;
			else
				counter2 <= counter2 + 1'b1;
		end
	end
	
	assign x_out = 9 * counter1[2:0] + counter2[2:0];
	assign y_out = 9 * counter1[5:3] + counter2[5:3];
	//assign x_out = counter1[]counter2[1:0];
	//assign y_out = counter2[3:2];
	
endmodule

module RateDivider(
	input clk, resetn,
	output enable);
	reg [5:0] count;
	always @(posedge clk)
	begin 
		if (!resetn) 
			count[5:0] <= 6'b000000;
		else if(count[5:0] == 111111)
			count <= 6'b000000;
		else
			count[5:0] <= count[5:0] + 1'b1;
	end
	assign enable = (count[5:0] == 6'b000000) ? 1'b0: 1'b1;
endmodule

module mux2to1(x, y, s, m);
	input [6:0] x;
	input [6:0] y;
	input s;
	output [6:0] m;
	
	assign m = s ? y : x;
endmodule

module mux8to1(
	input [5:0] index0,
	input [5:0] index1,
	input [5:0] index2,
	input [5:0] index3,
	input [5:0] index4,
	input [5:0] index5,
	input [5:0] index6,
	input [5:0] index7,
	input [2:0] picker,
	output [5:0] index_out);
	reg [5:0] data_out;
	
	localparam INDEX0 = 3'b000,
			INDEX1 = 3'b001,
			INDEX2 = 3'b010,
			INDEX3 = 3'b011,
			INDEX4 = 3'b100,
			INDEX5 = 3'b101,
			INDEX6 = 3'b110,
			INDEX7 = 3'b111;
	always @(*) 
	begin: mux8to1
		case (picker)
		INDEX0: data_out = index0;
		INDEX1: data_out = index1;
		INDEX2: data_out = index2;
		INDEX3: data_out = index3;
		INDEX4: data_out = index4;
		INDEX5: data_out = index5;
		INDEX6: data_out = index6;
		INDEX7: data_out = index7;
		default: data_out = index0;
		endcase
	end
	assign index_out = data_out;
endmodule
			
	
// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ADDRESSSTALL_A NUMERIC "0"
// Retrieval info: PRIVATE: AclrAddr NUMERIC "0"
// Retrieval info: PRIVATE: AclrByte NUMERIC "0"
// Retrieval info: PRIVATE: AclrData NUMERIC "0"
// Retrieval info: PRIVATE: AclrOutput NUMERIC "0"
// Retrieval info: PRIVATE: BYTE_ENABLE NUMERIC "0"
// Retrieval info: PRIVATE: BYTE_SIZE NUMERIC "8"
// Retrieval info: PRIVATE: BlankMemory NUMERIC "1"
// Retrieval info: PRIVATE: CLOCK_ENABLE_INPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: CLOCK_ENABLE_OUTPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: Clken NUMERIC "0"
// Retrieval info: PRIVATE: DataBusSeparated NUMERIC "1"
// Retrieval info: PRIVATE: IMPLEMENT_IN_LES NUMERIC "0"
// Retrieval info: PRIVATE: INIT_FILE_LAYOUT STRING "PORT_A"
// Retrieval info: PRIVATE: INIT_TO_SIM_X NUMERIC "0"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone V"
// Retrieval info: PRIVATE: JTAG_ENABLED NUMERIC "0"
// Retrieval info: PRIVATE: JTAG_ID STRING "NONE"
// Retrieval info: PRIVATE: MAXIMUM_DEPTH NUMERIC "0"
// Retrieval info: PRIVATE: MIFfilename STRING ""
// Retrieval info: PRIVATE: NUMWORDS_A NUMERIC "64"
// Retrieval info: PRIVATE: RAM_BLOCK_TYPE NUMERIC "0"
// Retrieval info: PRIVATE: READ_DURING_WRITE_MODE_PORT_A NUMERIC "3"
// Retrieval info: PRIVATE: RegAddr NUMERIC "1"
// Retrieval info: PRIVATE: RegData NUMERIC "1"
// Retrieval info: PRIVATE: RegOutput NUMERIC "0"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: SingleClock NUMERIC "1"
// Retrieval info: PRIVATE: UseDQRAM NUMERIC "1"
// Retrieval info: PRIVATE: WRCONTROL_ACLR_A NUMERIC "0"
// Retrieval info: PRIVATE: WidthAddr NUMERIC "6"
// Retrieval info: PRIVATE: WidthData NUMERIC "4"
// Retrieval info: PRIVATE: rden NUMERIC "0"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: CLOCK_ENABLE_INPUT_A STRING "BYPASS"
// Retrieval info: CONSTANT: CLOCK_ENABLE_OUTPUT_A STRING "BYPASS"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone V"
// Retrieval info: CONSTANT: LPM_HINT STRING "ENABLE_RUNTIME_MOD=NO"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altsyncram"
// Retrieval info: CONSTANT: NUMWORDS_A NUMERIC "64"
// Retrieval info: CONSTANT: OPERATION_MODE STRING "SINGLE_PORT"
// Retrieval info: CONSTANT: OUTDATA_ACLR_A STRING "NONE"
// Retrieval info: CONSTANT: OUTDATA_REG_A STRING "UNREGISTERED"
// Retrieval info: CONSTANT: POWER_UP_UNINITIALIZED STRING "FALSE"
// Retrieval info: CONSTANT: READ_DURING_WRITE_MODE_PORT_A STRING "NEW_DATA_NO_NBE_READ"
// Retrieval info: CONSTANT: WIDTHAD_A NUMERIC "6"
// Retrieval info: CONSTANT: WIDTH_A NUMERIC "4"
// Retrieval info: CONSTANT: WIDTH_BYTEENA_A NUMERIC "1"
// Retrieval info: USED_PORT: address 0 0 6 0 INPUT NODEFVAL "address[5..0]"
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT VCC "clock"
// Retrieval info: USED_PORT: data 0 0 4 0 INPUT NODEFVAL "data[3..0]"
// Retrieval info: USED_PORT: q 0 0 4 0 OUTPUT NODEFVAL "q[3..0]"
// Retrieval info: USED_PORT: wren 0 0 0 0 INPUT NODEFVAL "wren"
// Retrieval info: CONNECT: @address_a 0 0 6 0 address 0 0 6 0
// Retrieval info: CONNECT: @clock0 0 0 0 0 clock 0 0 0 0
// Retrieval info: CONNECT: @data_a 0 0 4 0 data 0 0 4 0
// Retrieval info: CONNECT: @wren_a 0 0 0 0 wren 0 0 0 0
// Retrieval info: CONNECT: q 0 0 4 0 @q_a 0 0 4 0
// Retrieval info: GEN_FILE: TYPE_NORMAL ram64x4.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL ram64x4.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL ram64x4.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL ram64x4.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL ram64x4_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL ram64x4_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf
