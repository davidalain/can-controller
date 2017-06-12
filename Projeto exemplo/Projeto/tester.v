`timescale 1ns/100ps

//====================================================================
//====================== Includes ====================================
//====================================================================

`include "can_decoder.v"

//====================================================================
//============== Declaração do módulo ================================
//====================================================================

module tester(
	finish			//Tem que ter pelo menos pino de IO para poder simular.
);
	
output wire finish;

//====================================================================
//===================== Constantes ===================================
//====================================================================

parameter MAX_FRAME_LEN	= 512;

parameter [MAX_FRAME_LEN-1:0] DATA_BITS = 'b011001110010000100010101010101010101010101010101010101010101010101010101010101010100000000000011111111;

parameter NUM_BITS_TO_SEND = 102;


//====================================================================
//===================== Variáveis ====================================
//====================================================================

reg clk,rst,sample_point;
integer bit_index_sent;
integer clk2sample;
reg rx_bit;
wire field_start_of_frame; 
wire [10:0] field_id_a; 
wire field_ide;
wire field_rtr; 
wire field_srr; 
wire field_reserved1;
wire field_reserved0; 
wire [17:0]field_id_b;
wire [3:0]field_dlc;
wire [63:0]field_data; 
wire [14:0]field_crc;
wire field_crc_delimiter;
wire field_ack_slot;
wire field_ack_demiliter;


can_decoder i_can_decoder
(	.clock(clk),			// Clock do circuito
	.reset(rst),			// Reset (em nível lógico 1)

	.rx_bit(rx_bit),  		// Sinal com o bit lido no barramento
	.sample_point(sample_point),  	// Indica quando o bit deve ser lido (na transicao deste sinal de 0 para 1)
	
	// Deixar os campos como saida do modulo para ver a saída nos testes
	
	.field_start_of_frame	(field_start_of_frame),
	.field_id_a				(field_id_a),
	.field_ide				(field_ide),
	.field_rtr				(field_rtr),
	.field_srr				(field_srr),					// Campo do frame CAN extendido (srr = rtr_srr_temp)
	.field_reserved1		(field_reserved1),
	.field_reserved0		(field_reserved0),
	.field_id_b				(field_id_b),					// Campo do frame CAN extendido
	.field_dlc				(field_dlc),
	.field_data				(field_data),
	.field_crc				(field_crc),
	.field_crc_delimiter	(field_crc_delimiter),
	.field_ack_slot			(field_ack_slot),
	.field_ack_delimiter	(field_ack_delimiter),
	.rtr_srr_temp			(rtr_srr_temp)
);

//====================================================================
//====================== Comportamento ===============================
//====================================================================

assign finish = rst;

// inicia
initial
begin
	/* ---------------------------------- */
	/*     PREENCHER ESTAS VARIAVEIS      */
	/* ---------------------------------- */
	
	/* ---------------------------------- */
	/* ---------------------------------- */
	/* ---------------------------------- */
	
	clk <= 1'b0;
	clk2sample <= 1'b0;
	sample_point <= 1'b0;
	rst <= 1'b1;
	bit_index_sent <= NUM_BITS_TO_SEND-1;
	rx_bit <= DATA_BITS[NUM_BITS_TO_SEND-1];
	#200
	
	$display("DEBUG: =============== INICIOU ==================");
	rst <= 1'b0;
end

//manda os bits
// Após o sample point
always @(negedge sample_point)
if(rst == 0)
begin
// Se o bit que foi enviado no posedge foi maior que 0
	if(bit_index_sent > 0)
	begin
		// O proximo bit enviado sera 
		rx_bit = DATA_BITS[bit_index_sent - 1];
		// Atualiza o indice do bit que sera enviado
		bit_index_sent = bit_index_sent - 1;
	end
	// Se o ultimo bit enviado foi o indice 0, entao acabou. o rx_bit ficara apenas recessivo
	else
		rx_bit = 1;
end



//generate clock
always
begin
	#100 clk = !clk;
end
	
//generate sample_point
always @(posedge clk)
	if (clk2sample < (10-1)) 
	begin
		clk2sample <= clk2sample + 1;
		sample_point <= 0;
	end
	else
	begin
		clk2sample <= 0;
		sample_point <= 1;
	end

//always @(negedge clk)
//begin
//	sample_point <= 0;
//end
	

	
	

endmodule


