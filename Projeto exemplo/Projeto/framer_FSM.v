module framer_FSM(
clock,
reset,

rx_bit,  // Sinal com o bit a ser lido
rx_clk,  // Indica quando o bit deve ser lido (na transicao deste sinal de 0 para 1)

error_in,	// Sinal de erro (O modulo pode ser avisado por outros que houve um erro)
error_out// Sinal de erro (O modulo pode avisar aos outros que houve um erro)
);


// == Input pins/ports == 
input wire	rx_bit;
input wire	rx_clk;

input	wire	clock;
input	wire	reset;

input wire error_in;

// == Output pins/ports == 
output wire error_out;

// == Internal Constants == 
parameter IDLE  			= 5'd1;
parameter START_FRAME  	= 5'd2;
parameter IDENTIFIER_A  = 5'd3;
parameter RTR_SRR  		= 5'd4;
parameter IDE  			= 5'd5;
parameter IDENTIFIER_B  = 5'd6;
parameter RTR  			= 5'd7;
parameter RESERVED_1  	= 5'd8;
parameter RESERVED_0  	= 5'd9;
parameter DLC  			= 5'd10;
parameter DATA  			= 5'd11;
parameter CRC  			= 5'd12;
parameter CRC_DELIMITER = 5'd13;
parameter ACK_SLOT  		= 5'd14;
parameter ACK_DELIMITER = 5'd15;
parameter END_OF_FRAME  = 5'd16;
parameter INTERFRAME  	= 5'd17;
parameter ERROR  			= 5'd18;


parameter size_IDENTIFIER_A = 4'd11;


// == Internal Variables == 
reg[0:127] 	rx_frame;
integer 		rx_index;

reg[4:0] 	state; 
reg[4:0]		next_state;

reg[3:0] 	contador_IDENTIFIER_A;
reg[4:0] 	contador_IDENTIFIER_B;
reg[2:0] 	contador_DLC;
reg[5:0]		contador_DATA;
reg[3:0]		contador_CRC;
reg[2:0]		contador_EOF;
reg[1:0]		contador_INTERFRAME	;

reg 			IDE_value;
reg[3:0] 	DLC_value;


// == Behaviour == 

initial
begin
	rx_frame = 128'b0; 	
	rx_index = 0;
	
	state = IDLE;
	next_state = IDLE;
	
	contador_IDENTIFIER_A	= 4'd0; 
	contador_IDENTIFIER_B	= 5'd0;
	contador_DLC 				= 3'd0;
	contador_DATA				= 6'd0;
	contador_CRC				= 4'd0;
	contador_EOF				= 3'd0;
	contador_INTERFRAME		= 2'd0;
	
	IDE_value 					= 1'bx;
	DLC_value					= 4'bx;
end


always @(posedge rx_clk)
begin
	case(state)
		IDLE: 
			if (rx_bit == 1'b0) 
			begin
				next_state <= START_FRAME;
			end
			
		START_FRAME:
			begin
				next_state <= IDENTIFIER_A;
				contador_IDENTIFIER_A <= 4'd1;
			end	
			
		IDENTIFIER_A:
			if (contador_IDENTIFIER_A < 4'd11)
			begin
				contador_IDENTIFIER_A <= contador_IDENTIFIER_A + 4'd1;
			end
			else
			begin
				contador_IDENTIFIER_A <= 4'd0;
				next_state <= RTR_SRR;
			end
		
		RTR_SRR:
			next_state <= IDE;
			
		IDE:
			if(IDE_value == 1'b1)
			begin
				next_state <= IDENTIFIER_B;
				contador_IDENTIFIER_B <= 1;
			end
			else if(IDE_value == 1'b0)
			begin
				next_state <= RESERVED_0;
			end
			
		IDENTIFIER_B:
			if(contador_IDENTIFIER_B < 5'd18)
			begin
				contador_IDENTIFIER_B = contador_IDENTIFIER_B + 1;
			end
			else
			begin
				next_state <= RTR;
			end
						
		RTR:
			next_state <= RESERVED_1;
			
		RESERVED_1:
			next_state <= RESERVED_0;
			
		RESERVED_0: 
			begin
				next_state <= DLC;
				contador_DLC = 3'd1;
			end
			
		DLC:
			if(contador_DLC < 3'd4)
			begin
				contador_DLC = contador_DLC + 1;
			end
			else
			begin
				next_state <= DATA;
				contador_DATA <= 1;
			end
			
		DATA:  
			if(contador_DATA < 8*DLC_value) // Talvez seja melhor usar deslocamento para esquerda 3x
			begin
				contador_DATA = contador_DATA + 1;
			end
			else
			begin
				next_state <= CRC;
				contador_CRC <= 1;
			end	
			
		CRC: 
			if(contador_CRC < 4'd15)
			begin
				contador_CRC <= contador_CRC + 1;
			end
			else
			begin
				next_state <= CRC_DELIMITER;
			end
			
		CRC_DELIMITER:
			next_state <= ACK_SLOT;
		ACK_SLOT:  	
			next_state <= ACK_DELIMITER;
		ACK_DELIMITER:
			begin
				next_state <= END_OF_FRAME;
				contador_EOF <= 1;
			end
		END_OF_FRAME: 
			if(contador_EOF < 3'd7)
			begin
				contador_EOF = contador_EOF + 1;
			end
			else
			begin
				next_state <= INTERFRAME;
				contador_INTERFRAME = 1;
			end
			
		INTERFRAME:  
			if(contador_INTERFRAME < 2'd3)
			begin
				contador_INTERFRAME = contador_INTERFRAME + 1;
			end
			else
			begin	
				next_state <= IDLE;
			end
			
		ERROR: /*TODO*/ 			
		;	
		default:/*TODO*/
		;
		
	endcase
end
	
/*	
always @(posedge clock)
begin
	case(state)
		IDLE: 
		
		START_FRAME:
		
		IDENTIFIER_A:
		
		RTR_SRR:
		
		IDE:
		
		IDENTIFIER_B:
		
		RTR:
		
		RESERVED_1:
		
		RESERVED_0: 
		
		DLC:
		
		DATA:  
		
		CRC: 
		
		CRC_DELIMITER:
		
		ACK_SLOT:  	
		
		ACK_DELIMITER:
		
		END_OF_FRAME: 
		
		INTERFRAME:  
		
		ERROR:  		
			
		default:
		
	endcase
end	
	*/
	
endmodule	