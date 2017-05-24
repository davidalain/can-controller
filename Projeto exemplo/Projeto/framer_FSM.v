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

input	wire	clock;
input	wire	reset;

input wire error_in;

// == Output pins/ports == 
output wire error_out;

// == Internal Constants == 
parameter IDLE  				= 5'd1;
parameter START_OF_FRAME  	= 5'd2;
parameter IDENTIFIER_A  	= 5'd3;
parameter RTR_SRR  			= 5'd4;
parameter IDE  				= 5'd5;
parameter IDENTIFIER_B  	= 5'd6;
parameter RTR  				= 5'd7;
parameter RESERVED_1  		= 5'd8;
parameter RESERVED_0  		= 5'd9;
parameter DLC  				= 5'd10;
parameter DATA  				= 5'd11;
parameter CRC  				= 5'd12;
parameter CRC_DELIMITER 	= 5'd13;
parameter ACK_SLOT  			= 5'd14;
parameter ACK_DELIMITER 	= 5'd15;
parameter END_OF_FRAME  	= 5'd16;
parameter INTERFRAME  		= 5'd17;
parameter ERROR  				= 5'd18;


parameter size_IDENTIFIER_A = 4'd11;

// == Internal Variables == 

//reg[0:127] 	rx_frame;
//reg[7:0]		rx_index;

reg[4:0] 	state;
reg[5:0]		last_rx_bits; //Usado para verificação do bit stuffing
reg			bit_de_stuffing;	//Flag que indica se o bit atual é um bit stuffing

reg[3:0] 	contador_IDENTIFIER_A;
reg[4:0] 	contador_IDENTIFIER_B;
reg[2:0] 	contador_DLC;
reg[5:0]		contador_DATA;
reg[3:0]		contador_CRC;
reg[2:0]		contador_EOF;
reg[1:0]		contador_INTERFRAME;

// == CAN Frame fields ==
reg 			field_start_of_frame;
reg[10:0]	field_identifier_a;
reg			field_ide;
reg			field_rtr;
reg			field_reserved_1;
reg			field_reserved_0;
reg[17:0]	field_identifier_b;
reg[3:0]		field_dlc;
reg[63:0]	field_data;
reg[14:0]	field_crc;
reg			field_ack;

reg			rtr_srr_temp;
reg[14:0]	calculated_crc;
reg			crc_enable;

//can_crc can_crc_rx
//(
//	.clk(clock),
//	.data(rx_bit),
//	.enable(crc_enable & sample_point), //	.enable(crc_enable & sample_point & (~bit_de_stuff)),
//	.initialize(state == IDLE),
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
		
		contador_IDENTIFIER_A	= 4'd0; 
		contador_IDENTIFIER_B	= 5'd0;
		contador_DLC 				= 3'd0;
		contador_DATA				= 6'd0;
		contador_CRC				= 4'd0;
		contador_EOF				= 3'd0;
		contador_INTERFRAME		= 2'd0;
	
		field_start_of_frame 	= 1'b0;
		field_identifier_a		= 11'b0;
		field_ide					= 1'b0;
		field_rtr					= 1'b0;
		field_reserved_1			= 1'b0;
		field_reserved_0			= 1'b0;
		field_identifier_b 		= 18'b0;
		field_dlc					= 4'b0;
		field_data					= 64'b0;
		field_crc					= 15'h0;
		field_ack					= 1'b0;

		rtr_srr_temp				= 1'b0;
		calculated_crc				= 15'h0;
		crc_enable					= 1'b0;
	
	end
endtask

always @(posedge sample_point)
begin
	

end



always @(posedge sample_point) /*TODO: verificar o momento da passagem de estado de acordo com os bits recebidos e o clock*/
begin
	case(state)
		IDLE: 
			if (rx_bit == 1'b0)  //Recebeu dominante, muda de estado!
			begin
				//Entrou no estado START_OF_FRAME:
			
				/*TODO: sincronizar os clocks do módulo de CRC e sample_point, porque pode acontecer de o crc ser calculado mais de uma vez no mesmo bit recebido*/
			
				//Habilita o cálculo do CRC para ser feito pelo módulo do CRC
				crc_enable <= 1'b1;
			
				//Armazena o bit recebido
				field_start_of_frame <= rx_bit;
				
				state <= START_OF_FRAME;
			end
			
		IDENTIFIER_A:
			begin
				//Entrou no estado IDENTIFIER_A:
				
				field_identifier_a <= {10'b0,rx_bit}; //Shift pra esquerda e rx_bit no lsb
				contador_IDENTIFIER_A <= 4'd1;
			
				state <= IDENTIFIER_A;
			end
			
		IDENTIFIER_A:
			if (contador_IDENTIFIER_A < 4'd11)
			begin
			
				field_identifier_a <= {field_identifier_a[10:0],rx_bit}; //Shift pra esquerda e rx_bit no LSB
				contador_IDENTIFIER_A <= contador_IDENTIFIER_A + 4'd1;
				
			end
			else
			begin
				//Entrou no estado RTR_SRR:
			
				contador_IDENTIFIER_A <= 4'd0;
				rtr_srr_temp <= rx_bit;
				
				state <= RTR_SRR;
			end
		
		RTR_SRR:
			//Entrou no estado IDE:
			field_ide <= rx_bit;
			state <= IDE;
			
		IDE:
			if(field_ide == 1'b1)
			begin
				//Entrou do estado contador_IDENTIFIER_B
			
				field_identifier_a <= {17'b0,rx_bit}; //Shift pra esquerda e rx_bit no lsb
				
				state <= IDENTIFIER_B;
				contador_IDENTIFIER_B <= 1;
			end
			else
			begin
				//
			
				state <= RESERVED_0;
			end
			
		IDENTIFIER_B:
			if(contador_IDENTIFIER_B < 5'd18)
			begin
				contador_IDENTIFIER_B = contador_IDENTIFIER_B + 1;
			end
			else
			begin
				state <= RTR;
			end
						
		RTR:
			state <= RESERVED_1;
			
		RESERVED_1:
			state <= RESERVED_0;
			
		RESERVED_0: 
			begin
				state <= DLC;
				contador_DLC = 3'd1;
			end
			
		DLC:
			if(contador_DLC < 3'd4)
			begin
				contador_DLC = contador_DLC + 1;
			end
			else
			begin
				state <= DATA;
				contador_DATA <= 1;
			end
			
		DATA:  
			if(contador_DATA < 8*field_dlc) // Talvez seja melhor usar deslocamento para esquerda 3x
			begin
				contador_DATA = contador_DATA + 1;
			end
			else
			begin
				crc_enable <= 1'b0;
			
				state <= CRC;
				contador_CRC <= 1;
			end	
			
		CRC: 
			if(contador_CRC < 4'd15)
			begin
				contador_CRC <= contador_CRC + 1;
			end
			else
			begin
				if (field_crc != calculated_crc)
				begin
						state <= ERROR; //Erro de CRC
				end
			
				state <= CRC_DELIMITER;
			end
			
		CRC_DELIMITER:
			state <= ACK_SLOT;
		ACK_SLOT:  	
			state <= ACK_DELIMITER;
		ACK_DELIMITER:
			begin
				state <= END_OF_FRAME;
				contador_EOF <= 1;
			end
		END_OF_FRAME: 
			if(contador_EOF < 3'd7)
			begin
				contador_EOF = contador_EOF + 1;
			end
			else
			begin
				state <= INTERFRAME;
				contador_INTERFRAME = 1;
			end
			
		INTERFRAME:  
			if(contador_INTERFRAME < 2'd3)
			begin
				contador_INTERFRAME = contador_INTERFRAME + 1;
			end
			else
			begin	
				state <= IDLE;
			end
			
		ERROR: /*TODO*/ 			
		;	
		default:/*TODO*/
			state <= IDLE;
		
	endcase
end
	
/*	
always @(posedge clock)
begin
	case(state)
		IDLE: 
		
		START_OF_FRAME:
		
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