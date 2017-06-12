
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
	field_ack_slot,
	field_ack_delimiter,
	
	rtr_srr_temp
);


// == Input pins/ports == 
input wire	rx_bit;
input wire	sample_point;

input wire clock;
input wire reset;

//====================================================================
//===================== Constantes ===================================
//====================================================================

parameter len_id_a 			= 4'd11;
parameter len_id_b 			= 5'd18;
parameter len_dlc 			= 3'd4;
parameter len_crc			= 4'd15;
parameter len_eof			= 3'd7;
parameter len_intermission	= 2'd3; //De acordo com a especificação são 3 bits recessivos. Mas na prática pode acontecer a transmissão de frames em sequência em que o terceiro bit do Intermission já pode ser o Start of Frame do próximo frame, então são 2. Vide documento can2spec.pdf, seção 9.1, item 2.

parameter len_flags_min 	= 4'd6;
parameter len_flags_max 	= 4'd12;
parameter len_delimiter 	= 4'd8;


//`define COLOR_BLACK_INI         	`"\033[1;30m`"
//`define COLOR_RED_INI	        	`"\033[1;31m`"
//`define COLOR_GREEN_INI	        `"\033[1;32m`"
//`define COLOR_YELLOW_INI			`"\033[1;33m`"
//`define COLOR_BLUE_INI	        `"\033[1;34m`"
//`define COLOR_MAGENTA_INI       	`"\033[1;35m`"
//`define COLOR_CYAN_INI	        `"\033[1;36m`"
//`define	COLOR_WHITE_INI	        `"\033[1;37m`"
//
//`define COLOR_END               	`"\033[m`"
//
`define COLOR_BLACK(str)        {"\033[1;30m",str, "\033[m"}
`define COLOR_RED(str)          {"\033[1;31m",str, "\033[m"}
`define COLOR_GREEN(str)		{"\033[1;32m",str, "\033[m"}
`define COLOR_YELLOW(str)		{"\033[1;33m",str, "\033[m"}
`define COLOR_BLUE(str)	        {"\033[1;34m",str, "\033[m"}
`define COLOR_MAGENTA(str)      {"\033[1;35m",str, "\033[m"}
`define COLOR_CYAN(str)	        {"\033[1;36m",str, "\033[m"}
`define	COLOR_WHITE(str)        {"\033[1;37m",str, "\033[m"}

parameter	COLOR_RED_INI 		= "\033[1;31m";
parameter	COLOR_END           = "\033[m";


//`define		DISPLAY(msg,params) $display("%sDEBUG: 		ID_a = b%b (0x%X)%s", params);

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
reg[1:0]	contador_intermission; //CHECK: Dois bits ou três bits.

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
output	reg 		field_ack_delimiter;

output	reg			rtr_srr_temp;
wire[14:0]	calculated_crc;
wire		crc_enable;

//== Estados ==

reg		state_idle;
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
reg 	state_intermission;

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



//====================================================================
//== Lógica combinacional para gerenciamento da máquina de estados ===
//====================================================================
	
assign 	go_state_idle				= 	reset | 
										(sample_point	& rx_bit 							& state_intermission	& contador_intermission == len_intermission-1);
											
assign 	go_state_id_a				= 	sample_point	& ~rx_bit							& (state_idle 			| 
																							(state_intermission	& contador_intermission == len_intermission-1));
																							
assign 	go_state_rtr_srr_temp		= 	sample_point				& ~bit_de_stuffing		& state_id_a  			& (contador_id_a == len_id_a-1);
assign 	go_state_ide				=	sample_point				& ~bit_de_stuffing		& state_rtr_srr_temp;
assign 	go_state_id_b				=	sample_point	& rx_bit	& ~bit_de_stuffing		& state_ide;
assign 	go_state_rtr				=	sample_point				& ~bit_de_stuffing		& state_id_b			& (contador_id_b == len_id_b-1);
assign 	go_state_reserved1			=	sample_point				& ~bit_de_stuffing		& state_rtr;
assign 	go_state_reserved0			=	sample_point				& ~bit_de_stuffing		& ( state_reserved1 | (~rx_bit & state_ide));
assign 	go_state_dlc				=	sample_point				& ~bit_de_stuffing		& state_reserved0;
assign 	go_state_data				=	sample_point				& ~bit_de_stuffing		& state_dlc		& (contador_dlc == len_dlc-1) & ({field_dlc[2:0],rx_bit} != 0 & ~field_rtr);
assign 	go_state_crc				=	sample_point				& ~bit_de_stuffing		& 
																							((state_dlc		& (contador_dlc == len_dlc-1) & ({field_dlc[2:0],rx_bit} == 4'b0 | field_rtr)) | 
																							(state_data		& ((contador_data == (8 * field_dlc)-1))));
																	
assign 	go_state_crc_delimiter		=	sample_point				& ~bit_de_stuffing		& state_crc				& (contador_crc == len_crc-1) ;
assign 	go_state_ack_slot			=	sample_point	& rx_bit							& state_crc_delimiter;
assign 	go_state_ack_delimiter		=	sample_point	& ~rx_bit							& state_ack_slot;
assign 	go_state_eof				=	sample_point	& rx_bit							& state_ack_delimiter;
assign 	go_state_intermission		=	sample_point	& rx_bit							& 
																							((state_eof					& contador_eof == len_eof-1) |
																							(state_error_delimiter 		& contador_delimiter == len_delimiter-1) |
																							(state_overload_delimiter 	& contador_delimiter == len_delimiter-1));
	
assign	bit_error_srr				=	sample_point	& rx_bit	& ~bit_de_stuffing 		& state_ide				& ~rtr_srr_temp;
assign	bit_error_crc_delimiter		=	sample_point	& ~rx_bit	& ~bit_de_stuffing		& state_crc_delimiter;
assign	bit_error_ack_slot			=	sample_point	& rx_bit							& state_ack_slot;
assign	bit_error_ack_delimiter		=	sample_point	& ~rx_bit							& state_ack_delimiter;
assign	bit_error_eof				=	sample_point	& ~rx_bit							& state_eof;
assign	bit_error_intermission		=	sample_point	& ~rx_bit							& state_intermission	& (contador_intermission != len_intermission-1);
assign	bit_error_crc_dont_match	= 	sample_point										& state_ack_delimiter 	& (calculated_crc != field_crc); //O erro de CRC é verificado no estado ACK Delimiter. CAN2Spec - Seção 6.2 - pg 23
assign	bit_error_flags				=	sample_point										& (state_overload_flags | state_error_flags) & 							
																																		((rx_bit	& contador_flags < len_flags_min-1) |
																																		(~rx_bit	& contador_flags >= len_flags_max));
	
assign	enable_bitstuffing			=	state_id_a |
										state_rtr_srr_temp |
										state_ide |
										state_id_b | 
										state_rtr |
										state_reserved1 |
										state_reserved0 |
										state_dlc | 
										state_data |
										state_crc |
										state_crc_delimiter; // No caso de o crc terminar com 11111 o proximo bit é de bit stuffing. Por isso, e pelo jeito que estamos tratando, o estado de crc_del deve ser adicionado a esta lista
										
assign	bit_de_stuffing				=	sample_point		&	enable_bitstuffing	& (((last_rx_bits == 5'h00) & rx_bit) | ((last_rx_bits == 5'h1F) & ~rx_bit));

assign	bit_error_bit_stuffing		= 	sample_point		&	enable_bitstuffing	& (((last_rx_bits == 5'h00) & ~rx_bit) | ((last_rx_bits == 5'h1F) & rx_bit));

assign	go_state_error_flags 		=	bit_error_srr |
										bit_error_crc_delimiter | 
										bit_error_ack_slot |
										bit_error_ack_delimiter | 
										bit_error_eof | 
										bit_error_bit_stuffing | 
										bit_error_crc_dont_match |
										bit_error_flags;
								
assign	go_state_error_delimiter	=	sample_point	& rx_bit	& state_error_flags		& (contador_flags >= len_flags_min && contador_flags < len_flags_max);

assign	go_state_overload_flags		=	sample_point	& ~rx_bit	& 
												((state_eof & contador_eof == len_eof-1) |
												(state_error_delimiter & contador_delimiter == len_delimiter-1) |
												(state_intermission & contador_intermission < len_intermission-1) |
												(state_overload_delimiter & contador_delimiter==len_delimiter-1));

assign	go_state_overload_delimiter	=	sample_point	& rx_bit	& state_overload_flags	& (contador_flags >= len_flags_min && contador_flags < len_flags_max);

assign	crc_initialize		=	reset | 
										(sample_point	& rx_bit 							& state_intermission	& contador_intermission == len_intermission-2);

assign	crc_enable			=	state_id_a |
								state_rtr_srr_temp |
								state_ide |
								state_id_b |
								state_rtr |
								state_reserved1 |
								state_reserved0 |
								state_dlc |
								state_data;

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
//===================== Gerenciamento de impressão ===================
//====================================================================

// Impressão dos estados

always @(state_idle)
begin
if(state_idle & rx_bit)			//Entrando no estado com rx_bit=1
	$display("%s", `COLOR_GREEN("DEBUG: Estado IDLE"));
else if(state_idle & ~rx_bit)	//Entrando no estado com rx_bit=0
	$display("%s", `COLOR_GREEN("DEBUG: Estado Start of Frame"));
end

always @(state_id_a)
begin
if(state_id_a) 	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado ID_a (11 bits)"));
else			//Saindo do estado
	$display("DEBUG: 		ID_a = b%b (0x%X)", field_id_a, field_id_a);
end

always @(state_rtr_srr_temp)
begin
if(state_rtr_srr_temp)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado RTR_SRR_temp"));
else				//Saindo do estado
	$display("DEBUG: 		RTR_SRR_temp = b%b", rtr_srr_temp);
end

always @(state_ide)
begin
if(state_ide)		//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado IDE"));
else				//Saindo do estado
	$display("DEBUG: 		IDE = b%b", field_ide);
end

always @(state_id_b)
if(state_id_b) 	//Entrando no estado
begin
	$display("DEBUG: 		SRR = b%b", rtr_srr_temp); //SRR = RTR_SRR_temp (Frame extendido)
	$display("%s", `COLOR_GREEN("DEBUG: Estado ID_b (18 bits) (Frame extendido)"));
end
else			//Saindo do estado
begin
	$display("DEBUG: 		ID_b = b%b (0x%X)", field_id_b, field_id_b);
	$display("DEBUG: 		ID_ab (full) = b%b (0x%X)", {field_id_a,field_id_b}, {field_id_a,field_id_b});
end

always @(state_rtr)
begin
if(state_rtr)		//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado RTR (Frame extendido)"));
else				//Saindo do estado
	$display("DEBUG: 		RTR = b%b", field_rtr);
end

always @(state_reserved1)
begin
if(state_reserved1)		//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado Reserved1 (Frame extendido)"));
else					//Saindo do estado
	$display("DEBUG: 		Reserved1 = b%b", field_reserved1);
end

always @(state_reserved0)
begin
if(state_reserved0)		//Entrando no estado
begin
	if(~field_ide)
	begin
		$display("DEBUG: 		RTR = b%b", rtr_srr_temp); //RTR=RTR_SRR_temp (Frame base)
	end
	$display("%s", `COLOR_GREEN("DEBUG: Estado Reserved0"));
end
else					//Saindo do estado
	$display("DEBUG: 		Reserved0 = b%b", field_reserved0);
end

always @(state_dlc)
begin
if(state_dlc)			//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado DLC"));
else					//Saindo do estado
	$display("DEBUG: 		DLC = b%b (0x%X, %d)", field_dlc,field_dlc,field_dlc);
end

always @(state_data)
begin
if(state_data)			//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado DATA"));
else					//Saindo do estado
	$display("DEBUG: 		DATA = b%b (0x%X)", field_data,field_data);
end

always @(state_crc)
begin
if(state_crc)			//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado CRC"));
else					//Saindo do estado
	$display("DEBUG: 		CRC = b%b (0x%X)", field_crc,field_crc);
end

always @(state_crc_delimiter)
begin
if(state_crc_delimiter)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado CRC Delimiter"));
else					//Saindo do estado
	$display("DEBUG: 		CRC Delimiter = b%b", field_crc_delimiter);
end

always @(state_ack_slot)
begin
if(state_ack_slot)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado ACK Slot"));
else					//Saindo do estado
	$display("DEBUG: 		ACK slot = b%b", field_ack_slot);
end

always @(state_ack_delimiter)
begin
if(state_ack_delimiter)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado ACK Delimiter"));
else					//Saindo do estado
	$display("DEBUG: 		ACK Delimiter = b%b", field_ack_delimiter);
end

always @(state_eof)
begin
if(state_eof)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado End of Frame"));
end

always @(state_overload_flags)
begin
if(state_overload_flags)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado Overload Flags"));
end

always @(state_overload_delimiter)
begin
if(state_overload_delimiter)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado Overload Delimiter"));
end

always @(state_error_flags)
begin
if(state_error_flags)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado Error Flags"));
end

always @(state_error_delimiter)
begin
if(state_error_delimiter)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado Error Delimiter"));
end

always @(state_intermission)
begin
if(state_intermission)	//Entrando no estado
	$display("%s", `COLOR_GREEN("DEBUG: Estado Intermission"));
end

// Impressão dos erros

always @(bit_error_srr)
begin
if(bit_error_srr)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error SRR = 0"));
end

always @(bit_error_crc_delimiter)
begin
if(bit_error_crc_delimiter)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error CRC Delimiter = 0"));
end

always @(bit_error_ack_slot)
begin
if(bit_error_ack_slot)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error ACK Slot = 1"));
end

always @(bit_error_ack_delimiter)
begin
if(bit_error_ack_delimiter)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error ACK Delimiter = 0"));
end

always @(bit_error_eof)
begin
if(bit_error_eof)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error EOF contem bit 0"));
end

always @(bit_error_intermission)
begin
if(bit_error_intermission)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error Intermission contem bit 0"));
end

always @(bit_error_bit_stuffing)
begin
if(bit_error_bit_stuffing)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error Bit Stuffing"));
end

always @(bit_error_crc_dont_match)
begin
if(bit_error_crc_dont_match)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error CRC não bate"));
end

always @(bit_error_flags)
begin
if(bit_error_flags)	//Quando a flag for setada pra 1
	$display("%s", `COLOR_RED("DEBUG: Error campo de flags com contem bit 1 ou contador_flags > 12"));
end

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
else if(go_state_intermission | go_state_error_flags | go_state_overload_flags) //Sai do estado se a flag do próximo estiver ativa!
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
else if(go_state_intermission | go_state_error_flags | go_state_overload_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_overload_delimiter <= 1'b0;
else if(go_state_overload_delimiter)
	state_overload_delimiter <= 1'b1; //Entra no estado!
end

// Estado idle (start of frame)
always @(posedge clock or posedge reset)
begin
if(reset)
	state_idle <= 1'b1;
else if(go_state_id_a) //Sai do estado se a flag do próximo estiver ativa!
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
else if(go_state_overload_flags | go_state_intermission | go_state_error_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_eof <= 1'b0;
else if(go_state_eof)
	state_eof <= 1'b1; //Entra no estado!
end


// Estado intermission 
always @(posedge clock or posedge reset)
begin
if(reset)
	state_intermission <= 1'b0;
else if(go_state_idle | go_state_id_a | go_state_overload_flags) //Sai do estado se a flag do próximo estiver ativa!
	state_intermission <= 1'b0;
else if(go_state_intermission)
	state_intermission <= 1'b1; //Entra no estado!
end

//====================================================================
//============= Preenchimento dos campos do frame ====================
//====================================================================

// ========== Campos sem bit stuffing =============

// Para ser sintetizavel, uma "variavel" só pode ser modificada em um único bloco always

// Campo Error Flags & Overload Flags
always @(posedge clock or posedge reset)
begin
if(reset)
	contador_flags <= 4'b0;
else if(sample_point & state_error_flags)
	contador_flags <= contador_flags + 4'd1;
else if(sample_point & state_overload_flags)
	contador_flags <= contador_flags + 4'd1;
else if(sample_point)
	contador_flags <= 4'd0;
end


// Campo Error Delimiter & Overload Delimiter
always @(posedge clock or posedge reset)
begin
if(reset)
	contador_delimiter <= 4'b0;
else if(sample_point & state_error_delimiter)
	contador_delimiter <= contador_delimiter + 4'd1;
else if(sample_point & state_overload_delimiter)
	contador_delimiter <= contador_delimiter + 4'd1;
else if (sample_point)
	contador_delimiter <= 4'd0;
end


// Campo Start of Frame
always @ (posedge clock or posedge reset)
begin
if (reset)
	field_start_of_frame <= 1'bx;
else if (sample_point & state_idle & ~rx_bit)
	field_start_of_frame <= 1'b0;
end

// Campo Ack Delimiter
always @ (posedge clock or posedge reset)
begin
if (reset)
	field_ack_delimiter <= 1'bx;
else if (sample_point & state_ack_delimiter)
	field_ack_delimiter <= rx_bit;
end

// Campo CRC delimiter
always @ (posedge clock or posedge reset)
begin
if (reset)
	field_crc_delimiter <= 1'bx;
else if (sample_point & state_crc_delimiter & (~bit_de_stuffing)) 
	field_crc_delimiter <= rx_bit;
end


// Campo ACK
always @ (posedge clock or posedge reset)
begin
  if (reset)
    field_ack_slot <= 1'b0;
  else if (sample_point & state_ack_slot)
    field_ack_slot <= rx_bit;
end

// Campo EOF
always @ (posedge clock or posedge reset)
begin
  if (reset)
	contador_eof <= 0;
  else if (sample_point & state_eof)
    contador_eof <= contador_eof + 1;
  else if (sample_point)
    contador_eof <= 0;
end

// Campo intermission
always @ (posedge clock or posedge reset)
begin
  if (reset)
	contador_intermission <= 0;
  else if (sample_point & state_intermission )
    contador_intermission <= contador_intermission + 1;
  else if (sample_point)
    contador_intermission <= 0;
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
else if (sample_point & ~state_id_a)
	contador_id_a <= 4'd0;
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
else if (sample_point & ~state_id_b)
	contador_id_b <= 5'd0;
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
	//Se DLC == 4'b1XXX, aplica máscara 4'b1000 para garantir ser no máximo 8 o valor do DLC.
	//Nota: field_dlc[2] representará o bit de indice 3 após a execução da linha abaixo.
    field_dlc <= (~field_dlc[2]) ? ({field_dlc[2:0], rx_bit}) : ({field_dlc[2:0], rx_bit} & 4'b1000);
    contador_dlc <= contador_dlc + 1;
  end
  else if (sample_point & ~state_dlc)
	contador_dlc <= 3'd0;
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
  else if (sample_point & ~state_data)
	contador_data <= 6'd0;
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
  else if (sample_point & ~state_crc)
	contador_crc <= 0;
end


	
endmodule	


`endif