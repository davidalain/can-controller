//`include "can_crc.v"

module framer_FSM(
	clock,
	reset,

	rx_bit,  		// Sinal com o bit a ser lido
	sample_point,  // Indica quando o bit deve ser lido (na transicao deste sinal de 0 para 1)

	error_in,	// Sinal de erro (O modulo pode ser avisado por outros que houve um erro)
	error_out	// Sinal de erro (O modulo pode avisar aos outros que houve um erro)
);


// == Input pins/ports == 
input wire	rx_bit;
input wire	sample_point;

input wire clock;
input wire reset;

input wire error_in;

// == Output pins/ports == 
output wire error_out;

// == Internal Constants == 
//parameter IDLE  			= 5'd1;
//parameter START_OF_FRAME  	= 5'd2;
//parameter id_a  			= 5'd3;
//parameter RTR_SRR  			= 5'd4;
//parameter IDE  				= 5'd5;
//parameter id_b  			= 5'd6;
//parameter RTR  				= 5'd7;
//parameter RESERVED_1  		= 5'd8;
//parameter RESERVED_0  		= 5'd9;
//parameter DLC  				= 5'd10;
//parameter DATA  			= 5'd11;
//parameter CRC  				= 5'd12;
//parameter CRC_DELIMITER 	= 5'd13;
//parameter ACK_SLOT  		= 5'd14;
//parameter ACK_DELIMITER 	= 5'd15;
//parameter END_OF_FRAME  	= 5'd16;
//parameter INTERFRAME  		= 5'd17;
//parameter ERROR  			= 5'd18;


parameter LEN_ID_A 			= 4'd11;
parameter LEN_ID_B 			= 5'd18;
parameter LEN_DLC 			= 3'd4;
parameter LEN_CRC			= 4'd15;
parameter LEN_INTERFRAME	= 2'd3;
parameter LEN_EOF			= 3'd7;

// == Internal Variables == 

reg[4:0] 	state;
reg[5:0]	last_rx_bits; //Usado para verificação do bit stuffing
reg			bit_de_stuffing;	//Flag que indica se o bit atual é um bit stuffing

reg[3:0] 	contador_id_a;
reg[4:0] 	contador_id_b;
reg[2:0] 	contador_DLC;
reg[5:0]	contador_DATA;
reg[3:0]	contador_CRC;
reg[2:0]	contador_EOF;
reg[1:0]	contador_INTERFRAME;

// = = CAN Frame fields ==
reg 		field_start_of_frame;
reg[10:0]	field_id_a;
reg			field_ide;
reg			field_rtr;
reg			field_reserved_1;
reg			field_reserved_0;
reg[17:0]	field_id_b;
reg[3:0]	field_dlc;
reg[63:0]	field_data;
reg[14:0]	field_crc;
reg			field_crc_delimiter;
reg			field_ack;

reg			rtr_srr_temp;
reg[14:0]	calculated_crc;
reg			crc_enable;

reg 	state_idle;
reg 	state_id_a;				
reg 	state_rtr_srr_temp;
reg 	state_ide;
reg 	state_id_b;
reg		state_rtr;
reg 	state_reserved1;
reg 	state_reserved0;
reg 	state_dlc;
reg 	state_data;
reg 	state_crc;
reg 	state_crc_delimiter;
reg 	state_ack_slot;
reg 	state_ack_delimiter;
reg 	state_eof;
reg 	state_inter_frame;

//can_crc i_can_crc
//(
//	.clk(clock),
//	.data(rx_bit),
//	.enable(crc_enable & sample_point & (~bit_de_stuff)),
//	.initialize(crc_initialize),
//	.crc(calculated_crc)
//);

// == Behaviour == 

initial
begin
	resetAll();
end

task resetAll;
	begin
	
//		rx_frame = 128'b0; 	
//		rx_index = 0;
		
		state = IDLE;
		
		contador_id_a				= 4'd0; 
		contador_id_b				= 5'd0;
		contador_DLC 				= 3'd0;
		contador_DATA				= 6'd0;
		contador_CRC				= 4'd0;
		contador_EOF				= 3'd0;
		contador_INTERFRAME			= 2'd0;
	
		field_start_of_frame 		= 1'b0;
		field_id_a					= 11'b0;
		field_ide					= 1'b0;
		field_rtr					= 1'b0;
		field_reserved_1			= 1'b0;
		field_reserved_0			= 1'b0;
		field_id_b 					= 18'b0;
		field_dlc					= 4'b0;
		field_data					= 64'b0;
		field_crc					= 15'h0;
		field_crc_delimiter			= 1'b0;
		field_ack					= 1'b0;

		rtr_srr_temp				= 1'b0;
		calculated_crc				= 15'h0;
		crc_enable					= 1'b0;
	
	end
endtask

assign	last_bit_interframe	= 	state_inter_frame	& (contador_INTERFRAME == LEN_INTERFRAME);

assign 	go_state_idle			= 	sample_point	& rx_bit 		& last_bit_interframe;
assign 	go_state_id_a			= 	sample_point	& ~rx_bit		& (state_idle 			| last_bit_interframe);
assign 	go_state_rtr_srr_temp	= 	sample_point					& state_id_a  			& contador_id_a == LEN_ID_A;
assign 	go_state_ide			=	sample_point					& state_rtr_srr_temp;
assign 	go_state_id_b			=	sample_point	& rx_bit		& state_ide;
assign 	go_state_rtr			=	sample_point	 				& state_id_b			& contador_id_b == LEN_ID_B;
assign 	go_state_reserved1		=	sample_point					& state_rtr;
assign 	go_state_reserved0		=	sample_point					& state_reserved1;
assign 	go_state_dlc			=	sample_point					& state_reserved0;
assign 	go_state_data			=	sample_point					& state_dlc				& contador_DLC == LEN_DLC;
assign 	go_state_crc			=	sample_point					& (state_dlc			&((contador_DLC == LEN_DLC & field_dlc == 0) | field_rtr)) | ((state_data			& (contador_DATA == (8 * field_dlc))));
assign 	go_state_crc_delimiter	=	sample_point					& state_crc				& contador_CRC == LEN_CRC;
assign 	go_state_ack_slot		=	sample_point	& rx_bit		& state_crc_delimiter;
assign 	go_state_ack_delimiter	=	sample_point	& ~rx_bit		& state_ack_slot;
assign 	go_state_eof			=	sample_point	& rx_bit		& state_ack_delimiter;
assign 	go_state_inter_frame	=	sample_point					& state_eof				& contador_EOF == LEN_EOF;

assign	bit_error_srr				=	sample_point	& rx_bit		& state_ide				& ~rtr_srr_temp;
assign	bit_error_crc_delimiter		=	sample_point	& ~rx_bit		& state_crc_delimiter;
assign	bit_error_ack_slot			=	sample_point	& rx_bit		& state_ack_slot;
assign	bit_error_ack_delimiter		=	sample_point	& ~rx_bit		& state_ack_delimiter;
assign	bit_error_eof				=	sample_point	& ~rx_bit		& state_eof;
assign	bit_error_inter_frame		=	sample_point	& ~rx_bit		& state_inter_frame;
assign	bit_error_bit_stuffing		= 	enable_bitstuffing	& (({last_rx_bits,rx_bit}==6'h00) | ({last_rx_bits,rx_bit}==6'h3F));

assign	go_state_error 		=	bit_error_crc_delimiter | bit_error_ack_slot | bit_error_ack_delimiter | bit_error_eof | bit_error_inter_frame | bit_error_bit_stuffing;

assign	enable_bitstuffing	=	(state_id_a | state_rtr_srr_temp | state_ide | state_id_b | state_rtr | state_reserved1 | state_reserved0 | state_dlc | state_data | state_crc );
assign	bit_de_stuffing		=	enable_bitstuffing & ((last_rx_bits == 5'h00) & rx_bit) | ((last_rx_bits == 5'h1F) & ~rx_bit);

assign	crc_initialize		=	go_state_idle | (state_inter_frame && contador_INTERFRAME == LEN_INTERFRAME);
assign	crc_enable			=	state_id_a | state_rtr_srr_temp | state_ide | state_id_b | state_rtr |  (state_inter_frame & last_bit_interframe)


// Estado idle (start of frame)
always @(posedge clock or posedge reset)
begin
if(reset)
	state_idle <= 1'b0;
else if(go_state_id_a | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_idle <= 1'b0;
else if(go_state_ide)
	state_idle <= 1'b1; //Vai para o estado!
end

// Estado id_a
always @(posedge clock or posedge reset)
begin
if(reset)
	state_id_a <= 1'b0;
else if(go_state_rtr_srr_temp | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_id_a <= 1'b0;
else if(go_state_id_a)
	state_id_a <= 1'b1; //Vai para o estado!
end

// Estado ide
always @(posedge clock or posedge reset)
begin
if(reset)
	state_ide <= 1'b0;
else if(go_state_id_b | go_state_reserved0 | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_ide <= 1'b0;
else if(go_state_ide)
	state_ide <= 1'b1; //Vai para o estado!
end

// Estado id_b
always @(posedge clock or posedge reset)
begin
if(reset)
	state_id_b <= 1'b0;
else if(go_state_rtr | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_id_b <= 1'b0;
else if(go_state_id_b)
	state_id_b <= 1'b1; //Vai para o estado!
end


// Estado rtr
always @(posedge clock or posedge reset)
begin
if(reset)
	state_rtr <= 1'b0;
else if(go_state_reserved1 | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_rtr <= 1'b0;
else if(go_state_rtr)
	state_rtr <= 1'b1; //Vai para o estado!
end

// Estado reserved1
always @(posedge clock or posedge reset)
begin
if(reset)
	state_reserved1 <= 1'b0;
else if(go_state_reserved0 | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_reserved1 <= 1'b0;
else if(go_state_reserved1)
	state_reserved1 <= 1'b1; //Vai para o estado!
end

// Estado reserved0
always @(posedge clock or posedge reset)
begin
if(reset)
	state_reserved0 <= 1'b0;
else if(go_state_dlc | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_reserved0 <= 1'b0;
else if(go_state_reserved0)
	state_reserved0 <= 1'b1; //Vai para o estado!
end


// Estado dlc
always @(posedge clock or posedge reset)
begin
if(reset)
	state_dlc <= 1'b0;
else if(go_state_data | go_state_crc | go_state_error) //Sai do estado se a flag do próximo estiver ativa!
	state_dlc <= 1'b0;
else if(go_state_dlc)
	state_dlc <= 1'b1; //Vai para o estado!
end



//always @(posedge sample_point) /*TODO: verificar o momento da passagem de estado de acordo com os bits recebidos e o clock*/
//begin
//	case(state)
//		IDLE: 
//			if (rx_bit == 1'b0)  //Recebeu dominante, muda de estado!
//			begin
//				//Entrou no estado START_OF_FRAME:
//			
//				/*TODO: sincronizar os clocks do módulo de CRC e sample_point, porque pode acontecer de o crc ser calculado mais de uma vez no mesmo bit recebido*/
//			
//				//Habilita o cálculo do CRC para ser feito pelo módulo do CRC
//				crc_enable <= 1'b1;
//			
//				//Armazena o bit recebido
//				field_start_of_frame <= rx_bit;
//				
//				state <= START_OF_FRAME;
//			end
//			
//		id_a:
//			begin
//				//Entrou no estado id_a:
//				
//				field_id_a <= {10'b0,rx_bit}; //Shift pra esquerda e rx_bit no lsb
//				contador_id_a <= 4'd1;
//			
//				state <= id_a;
//			end
//			
//		id_a:
//			if (contador_id_a < 4'd11)
//			begin
//			
//				field_id_a <= {field_id_a[10:0],rx_bit}; //Shift pra esquerda e rx_bit no LSB
//				contador_id_a <= contador_id_a + 4'd1;
//				
//			end
//			else
//			begin
//				//Entrou no estado RTR_SRR:
//			
//				contador_id_a <= 4'd0;
//				rtr_srr_temp <= rx_bit;
//				
//				state <= RTR_SRR;
//			end
//		
//		RTR_SRR:
//			//Entrou no estado IDE:
//			field_ide <= rx_bit;
//			state <= IDE;
//			
//		IDE:
//			if(field_ide == 1'b1)
//			begin
//				//Entrou do estado contador_id_b
//			
//				field_id_a <= {17'b0,rx_bit}; //Shift pra esquerda e rx_bit no lsb
//				
//				state <= id_b;
//				contador_id_b <= 1;
//			end
//			else
//			begin
//				//
//			
//				state <= RESERVED_0;
//			end
//			
//		id_b:
//			if(contador_id_b < 5'd18)
//			begin
//				contador_id_b = contador_id_b + 1;
//			end
//			else
//			begin
//				state <= RTR;
//			end
//						
//		RTR:
//			state <= RESERVED_1;
//			
//		RESERVED_1:
//			state <= RESERVED_0;
//			
//		RESERVED_0: 
//			begin
//				state <= DLC;
//				contador_DLC = 3'd1;
//			end
//			
//		DLC:
//			if(contador_DLC < 3'd4)
//			begin
//				contador_DLC = contador_DLC + 1;
//			end
//			else
//			begin
//				state <= DATA;
//				contador_DATA <= 1;
//			end
//			
//		DATA:  
//			if(contador_DATA < 8*field_dlc) // Talvez seja melhor usar deslocamento para esquerda 3x
//			begin
//				contador_DATA = contador_DATA + 1;
//			end
//			else
//			begin
//				crc_enable <= 1'b0;
//			
//				state <= CRC;
//				contador_CRC <= 1;
//			end	
//			
//		CRC: 
//			if(contador_CRC < 4'd15)
//			begin
//				contador_CRC <= contador_CRC + 1;
//			end
//			else
//			begin
//				if (field_crc != calculated_crc)
//				begin
//						state <= ERROR; //Erro de CRC
//				end
//			
//				state <= CRC_DELIMITER;
//			end
//			
//		CRC_DELIMITER:
//			state <= ACK_SLOT;
//		ACK_SLOT:  	
//			state <= ACK_DELIMITER;
//		ACK_DELIMITER:
//			begin
//				state <= END_OF_FRAME;
//				contador_EOF <= 1;
//			end
//		END_OF_FRAME: 
//			if(contador_EOF < 3'd7)
//			begin
//				contador_EOF = contador_EOF + 1;
//			end
//			else
//			begin
//				state <= INTERFRAME;
//				contador_INTERFRAME = 1;
//			end
//			
//		INTERFRAME:  
//			if(contador_INTERFRAME < 2'd3)
//			begin
//				contador_INTERFRAME = contador_INTERFRAME + 1;
//			end
//			else
//			begin	
//				state <= IDLE;
//			end
//			
//		ERROR: /*TODO*/ 			
//		;	
//		default:/*TODO*/
//			state <= IDLE;
//		
//	endcase
//end
//	
///*	
//always @(posedge clock)
//begin
//	case(state)
//		IDLE: 
//		
//		START_OF_FRAME:
//		
//		id_a:
//		
//		RTR_SRR:
//		
//		IDE:
//		
//		id_b:
//		
//		RTR:
//		
//		RESERVED_1:
//		
//		RESERVED_0: 
//		
//		DLC:
//		
//		DATA:  
//		
//		CRC: 
//		
//		CRC_DELIMITER:
//		
//		ACK_SLOT:  	
//		
//		ACK_DELIMITER:
//		
//		END_OF_FRAME: 
//		
//		INTERFRAME:  
//		
//		ERROR:  		
//			
//		default:
//		
//	endcase
//end	
//	*/
	
endmodule	