
`ifndef CAN_DECODER
`define CAN_DECODER

//====================================================================
//====================== Includes ====================================
//====================================================================

`include "can_crc.v"

//====================================================================
//============== Declaração do módulo ================================
//====================================================================

module can_decoder(
	clock,			// Clock do circuito
	reset,			// Reset (em nível lógico 1)

	rx_bit,  		// Sinal com o bit lido no barramento
	sample_point,  	// Indica quando o bit deve ser lido (na transicao deste sinal de 0 para 1)

	error_in,		// Sinal de erro (O modulo pode ser avisado por outros que houve um erro)
	error_out,		// Sinal de erro (O modulo pode avisar aos outros que houve um erro)
	
	// Deixar os campos como saida do modulo para ver a saída nos testes
	
	field_start_of_frame,
	field_id_a,
	field_ide,
	field_rtr,
	field_srr,		// Campo do frame CAN extendido (rtr_srr_temp)
	field_reserved1,
	field_reserved0,
	field_id_b,		// Campo do frame CAN extendido
	field_dlc,
	field_data,
	field_crc,
	field_crc_delimiter,
	field_ack_slot
);


// == Input pins/ports == 
input wire	rx_bit;
input wire	sample_point;

input wire clock;
input wire reset;

input wire error_in;

// == Output pins/ports == 
output wire error_out;

//====================================================================
//===================== Constantes ===================================
//====================================================================

parameter len_id_a 			= 4'd11;
parameter len_id_b 			= 5'd18;
parameter len_dlc 			= 3'd4;
parameter len_crc			= 4'd15;
parameter len_eof			= 3'd7;
parameter len_interframe	= 2'd2; //De acordo com a especificação são 3 bits recessivos. Mas na prática pode acontecer a transmissão de frames em sequência em que o terceiro bit do Intermission já pode ser o Start of Frame do próximo frame, então são 2. Vide documento can2spec.pdf, seção 9.1, item 2.

parameter len_flags_min 	= 4'd6;
parameter len_flags_max 	= 4'd12;
parameter len_delimiter 	= 4'd8;

//====================================================================
//===================== Variáveis ====================================
//====================================================================

//== Variáveis para Lógica de controle do decoder ==

reg[4:0]	last_rx_bits; 		//Usado para verificação do bit stuffing (5 bits).
wire		bit_de_stuffing;	//Flag que indica se o bit recebido atual é um bit stuffing

/** Contador dos bits já recebidos nos estados relacionados **/
reg[3:0] 	contador_id_a;		
reg[4:0] 	contador_id_b;
reg[2:0] 	contador_dlc;
reg[5:0]	contador_data;
reg[3:0]	contador_crc;
reg[2:0]	contador_eof;
reg[1:0]	contador_interframe; //CHECK: Dois bits ou três bits.

reg[3:0]	contador_flags;
reg[3:0]	contador_delimiter;

//== Campos do frame ==

output	reg 		field_start_of_frame;
output	reg[10:0]	field_id_a;
output	reg			field_srr;
output	reg			field_ide;
output	reg			field_rtr;
output	reg			field_reserved1;
output	reg			field_reserved0;
output	reg[17:0]	field_id_b;
output	reg[3:0]	field_dlc;
output	reg[63:0]	field_data;
output	reg[14:0]	field_crc;
output	reg			field_crc_delimiter;
output	reg			field_ack_slot;

reg			rtr_srr_temp;
wire[14:0]	calculated_crc;
wire		crc_enable;

//== Estados ==

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
reg 	state_interframe;

reg		state_error_flags;
reg		state_error_delimiter;

reg		state_overload_flags;
reg		state_overload_delimiter;

//== Módulo CRC ==

can_crc i_can_crc
(
	.clock(clock),
	.data_in(rx_bit),
	.enable(crc_enable & sample_point & (~bit_de_stuffing)),
	.reset(crc_initialize),
	.crc(calculated_crc)
);

// == Behaviour == 



//====================================================================
//== Lógica combinacional para gerenciamento da máquina de estados ===
//====================================================================

assign	last_bit_interframe			= 	state_interframe	& (contador_interframe == len_interframe -1);
	
assign 	go_state_idle				= 	(sample_point	& rx_bit 	& last_bit_interframe) | reset;
assign 	go_state_id_a				= 	sample_point	& ~rx_bit	& (state_idle 			| last_bit_interframe);
assign 	go_state_rtr_srr_temp		= 	sample_point				& state_id_a  			& (contador_id_a == len_id_a - 1);
assign 	go_state_ide				=	sample_point				& state_rtr_srr_temp;
assign 	go_state_id_b				=	sample_point	& rx_bit	& state_ide;
assign 	go_state_rtr				=	sample_point				& state_id_b			& (contador_id_b == len_id_b -1);
assign 	go_state_reserved1			=	sample_point				& state_rtr;
assign 	go_state_reserved0			=	sample_point				& ( state_reserved1 | (~rx_bit & state_ide));
assign 	go_state_dlc				=	sample_point				& state_reserved0;
assign 	go_state_data				=	sample_point				& state_dlc				& (contador_dlc == len_dlc-1) & ({field_dlc[2:0],rx_bit} != 0);
assign 	go_state_crc				=	sample_point				& 
																					((state_dlc	&	(((contador_dlc == len_dlc -1) & ({field_dlc[2:0],rx_bit} == 0)) | field_rtr)) | 
																					(state_data		& ((contador_data == (8 * field_dlc)-1))));
assign 	go_state_crc_delimiter		=	sample_point				& state_crc				& (contador_crc == len_crc -1);
assign 	go_state_ack_slot			=	sample_point	& rx_bit	& state_crc_delimiter;
assign 	go_state_ack_delimiter		=	sample_point	& ~rx_bit	& state_ack_slot;
assign 	go_state_eof				=	sample_point	& rx_bit	& state_ack_delimiter;
assign 	go_state_interframe			=	sample_point				& state_eof				& (contador_eof == len_eof-1);
	
assign	bit_error_srr				=	sample_point	& rx_bit	& state_ide				& ~rtr_srr_temp;
assign	bit_error_crc_delimiter		=	sample_point	& ~rx_bit	& state_crc_delimiter;
assign	bit_error_ack_slot			=	sample_point	& rx_bit	& state_ack_slot;
assign	bit_error_ack_delimiter		=	sample_point	& ~rx_bit	& state_ack_delimiter;
assign	bit_error_eof				=	sample_point	& ~rx_bit	& state_eof;
assign	bit_error_interframe		=	sample_point	& ~rx_bit	& state_interframe;
assign	bit_crc_error				= 	sample_point				& state_crc_delimiter 	& (calculated_crc != field_crc);
	
assign	enable_bitstuffing			=	state_id_a | state_rtr_srr_temp | state_ide | state_id_b | 
										state_rtr | state_reserved1 | state_reserved0 | state_dlc | 
										state_data | state_crc;
assign	bit_de_stuffing				=	enable_bitstuffing & ((last_rx_bits == 5'h00) & rx_bit) | ((last_rx_bits == 5'h1F) & ~rx_bit);

assign	bit_error_bit_stuffing		= 	enable_bitstuffing	& (({last_rx_bits,rx_bit}==6'h00) | ({last_rx_bits,rx_bit}==6'h3F));

assign	go_state_error_flags 		=	bit_error_srr | bit_error_crc_delimiter | bit_error_ack_slot |
										bit_error_ack_delimiter | bit_error_eof | bit_error_interframe |
										bit_error_bit_stuffing | bit_crc_error;
								
assign	go_state_error_delimiter	=	sample_point	& rx_bit	& state_error_flags		& (contador_flags >= len_flags_min && contador_flags < len_flags_max);

assign	go_state_overload_flags		=	sample_point	& ~rx_bit	& 
												((state_eof & contador_eof == len_eof-1) |
												(state_error_delimiter & contador_delimiter == len_delimiter-1) |
												(state_interframe & contador_interframe < len_interframe) |
												(state_overload_delimiter & contador_delimiter==len_delimiter-1));

assign	go_state_overload_delimiter	=	sample_point	& rx_bit	& state_overload_flags	& (contador_flags >= len_flags_min && contador_flags < len_flags_max);




assign	crc_initialize		=	//fixme - testarei com outros valores go_state_idle | (state_interframe && contador_interframe == len_interframe -1);
												state_idle
												| state_interframe;
assign	crc_enable			=	// fixme - Testarei com outros valores  state_id_a | state_rtr_srr_temp | state_ide | state_id_b | state_rtr |  (state_interframe & last_bit_interframe) 
												state_id_a 				
												| state_rtr_srr_temp 
												| state_ide 
												| state_id_b 
												| state_rtr 
												| state_reserved1
												| state_reserved0
												| state_dlc
												| state_data;
											


//====================================================================
//===================== Gerenciamento do bit stuffing ================
//====================================================================

// Salva os últimos 5 bits lidos para o bit stuffing
always @(posedge clock or posedge reset)
begin
if(reset)
  last_rx_bits <= 5'b10101; //Inicia com valores alternados só pra não facilitar a condição do bit stuffing.
else if (sample_point)
  last_rx_bits <= {last_rx_bits[3:0],rx_bit};
end

//====================================================================
//===================== Gerenciamento de impressão erros =============
//====================================================================

// Impressão dos erros
/*
always @(posedge clock or posedge reset)
begin
if(reset)
  ;
else if (state_error_flags)

	//Casos dos erros de forma
	if(bit_error_srr)			$display("Erro de SRR");
	if(bit_error_crc_delimiter)	$display("Erro de CRC delimiter");
	if(bit_error_ack_slot)		$display("Erro de ACK slot");
	if(bit_error_ack_delimiter)	$display("Erro de ACK delimiter");
	if(bit_error_eof)			$display("Erro de EOF");
	if(bit_error_interframe)	$display("Erro de interframe");
	if(bit_crc_error)			$display("Erro de CRC");
end
*/
//====================================================================
//===================== Gerenciamento dos estados ====================
//====================================================================

// Estado Error Flags (quando for detectado algum erro no frame durante o processo de decodificação)
always @(posedge clock or posedge reset)
begin
if(reset)
	state_error_flags <= 1'b0;
else if(go_state_error_delimiter)  //Sai do estado se a flag do próximo estiver ativa!
	state_error_flags <= 1'b0;	
else if(go_state_error_flags)
	state_error_flags <= 1'b1; //Entra no estado!
end

// Estado Error Delimiter
always @(posedge clock or posedge reset)
begin
if(reset)
	state_error_delimiter <= 1'b0;
else if(go_state_idle | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_error_delimiter <= 1'b0;
else if(go_state_error_delimiter)
	state_error_delimiter <= 1'b1; //Entra no estado!
end

// Estado Overload Flags
always @(posedge clock or posedge reset)
begin
if(reset)
	state_overload_flags <= 1'b0;
else if(go_state_overload_delimiter)  //Sai do estado se a flag do próximo estiver ativa!
	state_overload_flags <= 1'b0;
else if(go_state_overload_flags)
	state_overload_flags <= 1'b1; //Entra no estado!
end

// Estado Overload Delimiter
always @(posedge clock or posedge reset)
begin
if(reset)
	state_overload_delimiter <= 1'b0;
else if(go_state_idle | go_state_overload_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_overload_delimiter <= 1'b0;
else if(go_state_overload_delimiter)
	state_overload_delimiter <= 1'b1; //Entra no estado!
end

// Estado idle (start of frame)
always @(posedge clock or posedge reset)
begin
if(reset)
	state_idle <= 1'b1;
else if(go_state_id_a | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_idle <= 1'b0;
else if(go_state_idle)
	state_idle <= 1'b1; //Entra no estado!
end

// Estado id_a
always @(posedge clock or posedge reset)
begin
if(reset)
	state_id_a <= 1'b0;
else if(go_state_rtr_srr_temp | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_id_a <= 1'b0;
else if(go_state_id_a)
	state_id_a <= 1'b1; //Entra no estado!
end

// Estado rtr_srr_temp
always @(posedge clock or posedge reset)
begin
if(reset)
	state_rtr_srr_temp <= 1'b0;
else if(go_state_ide | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_rtr_srr_temp <= 1'b0;
else if(go_state_rtr_srr_temp)
	state_rtr_srr_temp <= 1'b1; //Entra no estado!
end


// Estado ide
always @(posedge clock or posedge reset)
begin
if(reset)
	state_ide <= 1'b0;
else if(go_state_id_b | go_state_reserved0 | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_ide <= 1'b0;
else if(go_state_ide)
	state_ide <= 1'b1; //Entra no estado!
end

// Estado id_b
always @(posedge clock or posedge reset)
begin
if(reset)
	state_id_b <= 1'b0;
else if(go_state_rtr | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_id_b <= 1'b0;
else if(go_state_id_b)
	state_id_b <= 1'b1; //Entra no estado!
end


// Estado rtr
always @(posedge clock or posedge reset)
begin
if(reset)
	state_rtr <= 1'b0;
else if(go_state_reserved1 | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_rtr <= 1'b0;
else if(go_state_rtr)
	state_rtr <= 1'b1; //Entra no estado!
end

// Estado reserved1
always @(posedge clock or posedge reset)
begin
if(reset)
	state_reserved1 <= 1'b0;
else if(go_state_reserved0 | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_reserved1 <= 1'b0;
else if(go_state_reserved1)
	state_reserved1 <= 1'b1; //Entra no estado!
end

// Estado reserved0
always @(posedge clock or posedge reset)
begin
if(reset)
	state_reserved0 <= 1'b0;
else if(go_state_dlc | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_reserved0 <= 1'b0;
else if(go_state_reserved0)
	state_reserved0 <= 1'b1; //Entra no estado!
end


// Estado dlc
always @(posedge clock or posedge reset)
begin
if(reset)
	state_dlc <= 1'b0;
else if(go_state_data | go_state_crc | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_dlc <= 1'b0;
else if(go_state_dlc)
	state_dlc <= 1'b1; //Entra no estado!
end

// Estado data
always @(posedge clock or posedge reset)
begin
if(reset)
	state_data <= 1'b0;
else if(go_state_crc | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_data <= 1'b0;
else if(go_state_data)
	state_data <= 1'b1; //Entra no estado!
end

// Estado crc
always @(posedge clock or posedge reset)
begin
if(reset)
	state_crc <= 1'b0;
else if(go_state_crc_delimiter | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_crc <= 1'b0;
else if(go_state_crc)
	state_crc <= 1'b1; //Entra no estado!
end

// Estado crc_delimiter
always @(posedge clock or posedge reset)
begin
if(reset)
	state_crc_delimiter <= 1'b0;
else if(go_state_ack_slot | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_crc_delimiter <= 1'b0;
else if(go_state_crc_delimiter)
	state_crc_delimiter <= 1'b1; //Entra no estado!
end

// Estado ACK slot
always @(posedge clock or posedge reset)
begin
if(reset)
	state_ack_slot <= 1'b0;
else if(go_state_ack_delimiter | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_ack_slot <= 1'b0;
else if(go_state_ack_slot)
	state_ack_slot <= 1'b1; //Entra no estado!
end

// Estado ACK delimiter
always @(posedge clock or posedge reset)
begin
if(reset)
	state_ack_delimiter <= 1'b0;
else if(go_state_eof | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_ack_delimiter <= 1'b0;
else if(go_state_ack_delimiter)
	state_ack_delimiter <= 1'b1; //Entra no estado!
end


// Estado EOF 
always @(posedge clock or posedge reset)
begin
if(reset)
	state_eof <= 1'b0;
else if(go_state_overload_flags | go_state_interframe | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_eof <= 1'b0;
else if(go_state_eof)
	state_eof <= 1'b1; //Entra no estado!
end


// Estado interframe 
always @(posedge clock or posedge reset)
begin
if(reset)
	state_interframe <= 1'b0;
else if(go_state_idle | go_state_id_a  | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_interframe <= 1'b0;
else if(go_state_interframe)
	state_interframe <= 1'b1; //Entra no estado!
end

//====================================================================
//============= Preenchimento dos campos do frame ====================
//====================================================================
/* TODO - Zerar os contadores em um else final, ou seja, manter sempre zero quando não está contando*/
/* Preencher field_start_of_frame e field_crc_delimiter*/
/* Criar Campo ack_delimiter */

// ========== Campos sem bit stuffing =============

// Campo Error Flags
always @(posedge clock or posedge reset)
begin
if(reset)
	contador_flags <= 4'b0;
else if(sample_point & state_error_flags)
	contador_flags <= contador_flags + 4'd1;
end

// Campo Error Delimiter
always @(posedge clock or posedge reset)
begin
if(reset)
	contador_delimiter <= 4'b0;
else if(sample_point & state_error_delimiter)
	contador_delimiter <= contador_delimiter + 4'd1;
end

// Campo Overload Flags
always @(posedge clock or posedge reset)
begin
if(reset)
	contador_flags <= 4'b0;
else if(sample_point & state_overload_flags)
	contador_flags <= contador_flags + 1;
end

// Campo Overload Delimiter
always @(posedge clock or posedge reset)
begin
if(reset)
	contador_delimiter <= 4'b0;
else if(sample_point & state_overload_delimiter)
	contador_delimiter <= contador_delimiter + 1;
end

// ========== Campos com bit stuffing =============

// Campo id_a
always @ (posedge clock or posedge reset)
begin
if (reset)
begin
	field_id_a <= 11'h0;
	contador_id_a <= 4'd0;
end
else if (sample_point & state_id_a & (~bit_de_stuffing))
begin
	field_id_a <= {field_id_a[9:0], rx_bit};
	contador_id_a <= contador_id_a + 1;
end
end


// Campo rtr_srr_temp bit
always @ (posedge clock or posedge reset)
begin
if (reset)
	rtr_srr_temp <= 1'b0;
else if (sample_point & state_rtr_srr_temp & (~bit_de_stuffing))
	rtr_srr_temp <= rx_bit;
end

// Campo ide bit
always @ (posedge clock or posedge reset)
begin
  if (reset)
    field_ide <= 1'b0;
  else if (sample_point & state_ide & (~bit_de_stuffing))
    field_ide <= rx_bit;
end

// Campo id_b
always @ (posedge clock or posedge reset)
begin
if (reset)
begin
	field_id_b <= 11'h0;
	contador_id_b <= 5'd0;
	field_srr <= 1'b0;
end
else if (sample_point & state_id_b & (~bit_de_stuffing))
begin
	if(contador_id_b == 1)
		field_srr <= rtr_srr_temp;
	
	field_id_b <= {field_id_b[16:0], rx_bit};
	contador_id_b <= contador_id_b + 1;
end
end

// Campo rtr bit
always @ (posedge clock or posedge reset)
begin
if (reset)
	field_rtr <= 1'b0;
else if (sample_point & state_rtr 		& (~bit_de_stuffing) & field_ide)	// Frame extendido
	field_rtr <= rx_bit;	
else if (sample_point & state_reserved0 & (~bit_de_stuffing) & ~field_ide)	// Frame normal
	field_rtr <= rtr_srr_temp;
end

// Campo reserved1
always @ (posedge clock or posedge reset)
begin
if (reset)
	field_reserved1 <= 1'b0;
else if (sample_point & state_reserved1 & (~bit_de_stuffing))
	field_reserved1 <= rx_bit;
end

// Campo reserved0 
always @ (posedge clock or posedge reset)
begin
if (reset)
	field_reserved0 <= 1'b0;
else if (sample_point & state_reserved0 & (~bit_de_stuffing))
	field_reserved0 <= rx_bit;
end

// Campo Data Length Count (DLC)
always @ (posedge clock or posedge reset)
begin
  if (reset)
  begin
    field_dlc <= 4'b0;
	contador_dlc <= 3'd0;
  end
  else if (sample_point & state_dlc & (~bit_de_stuffing))
  begin
    field_dlc <= {field_dlc[2:0], rx_bit};
    contador_dlc <= contador_dlc + 1;
  end
end


// Campo Data
always @ (posedge clock or posedge reset)
begin
  if (reset)
  begin
    field_data <= 64'h0;
	contador_data <= 6'd0;
  end
  else if (sample_point & state_data & (~bit_de_stuffing))
  begin
    field_data <= {field_data[62:0], rx_bit};
	contador_data <= contador_data + 1;
  end
end

// Campo CRC
always @ (posedge clock or posedge reset)
begin
  if (reset)
  begin
    field_crc <= 15'h0;
	contador_crc <= 0;
  end
  else if (sample_point & state_crc & (~bit_de_stuffing))
  begin
    field_crc <= {field_crc[13:0], rx_bit};
	contador_crc <= contador_crc + 1;
  end
end


// Campo ACK
always @ (posedge clock or posedge reset)
begin
  if (reset)
    field_ack_slot <= 1'b0;
  else if (sample_point & state_ack_slot & (~bit_de_stuffing))
    field_ack_slot <= rx_bit;
end

// Campo EOF
always @ (posedge clock or posedge reset)
begin
  if (reset)
	contador_eof <= 0;
  else if (sample_point & state_eof & (~bit_de_stuffing))
    contador_eof <= contador_eof + 1;
end

// Campo intermission
always @ (posedge clock or posedge reset)
begin
  if (reset)
	contador_interframe <= 0;
  else if (sample_point & state_interframe & (~bit_de_stuffing))
    contador_interframe <= contador_interframe + 1;
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
//				contador_dlc = 3'd1;
//			end
//			
//		DLC:
//			if(contador_dlc < 3'd4)
//			begin
//				contador_dlc = contador_dlc + 1;
//			end
//			else
//			begin
//				state <= DATA;
//				contador_data <= 1;
//			end
//			
//		DATA:  
//			if(contador_data < 8*field_dlc) // Talvez seja melhor usar deslocamento para esquerda 3x
//			begin
//				contador_data = contador_data + 1;
//			end
//			else
//			begin
//				crc_enable <= 1'b0;
//			
//				state <= CRC;
//				contador_crc <= 1;
//			end	
//			
//		CRC: 
//			if(contador_crc < 4'd15)
//			begin
//				contador_crc <= contador_crc + 1;
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
//				contador_eof <= 1;
//			end
//		END_OF_FRAME: 
//			if(contador_eof < 3'd7)
//			begin
//				contador_eof = contador_eof + 1;
//			end
//			else
//			begin
//				state <= INTERFRAME;
//				contador_interframe = 1;
//			end
//			
//		INTERFRAME:  
//			if(contador_interframe < 2'd3)
//			begin
//				contador_interframe = contador_interframe + 1;
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


`endif