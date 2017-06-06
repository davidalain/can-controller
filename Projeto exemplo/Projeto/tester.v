`timescale 1ns/1ps

module tester();

reg clk,rst,sample_point;
reg [90:0] data_bits;
integer num_bits_to_send;
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
wire field_ack;
wire error_in, error_out;



can_decoder i_can_decoder
(	.clock(clk),			// Clock do circuito
	.reset(rst),			// Reset (em nível lógico 1)

	.rx_bit(rx_bit),  		// Sinal com o bit lido no barramento
	.sample_point(sample_point),  	// Indica quando o bit deve ser lido (na transicao deste sinal de 0 para 1)

	.error_in(error_in),		// Sinal de erro (O modulo pode ser avisado por outros que houve um erro)
	.error_out(error_out),		// Sinal de erro (O modulo pode avisar aos outros que houve um erro)
	
	// Deixar os campos como saida do modulo para ver a saída nos testes
	
	.field_start_of_frame (field_start_of_frame),
	.field_id_a           (field_id_a),
	.field_ide            (field_ide),
	.field_rtr            (field_rtr),
	.field_srr			    (field_srr),					// Campo do frame CAN extendido (rtr_srr_temp)
	.field_reserved1      (field_reserved1),
	.field_reserved0      (field_reserved0),
	.field_id_b			    (field_id_b),					// Campo do frame CAN extendido
	.field_dlc            (field_dlc),
	.field_data           (field_data),
	.field_crc            (field_crc),
	.field_crc_delimiter  (field_crc_delimiter),
	.field_ack            (field_ack)
);

// inicia
initial
begin
	/* ---------------------------------- */
	/*     PREENCHER ESTAS VARIAVEIS      */
	/* ---------------------------------- */
	//OBS: Modificar tamanho máximo de data_bits, se necessário.
	data_bits <= 90'b010110101010101010000110010101001001010111100001010101010010101010;
	num_bits_to_send <= 60;
	
	/* ---------------------------------- */
	/* ---------------------------------- */
	/* ---------------------------------- */
	#1
	clk <= 1'b0;
	clk2sample <= 1'b0;
	sample_point <= 1'b0;
	rst <= 1'b1;
	bit_index_sent <= num_bits_to_send-1;
	rx_bit <= data_bits[num_bits_to_send-1];
	#15
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
		rx_bit = data_bits[bit_index_sent - 1];
		// Atualiza o indice do bit que sera enviado
		bit_index_sent = bit_index_sent - 1;
	end
	// Se o ultimo bit enviado foi o indice 0, entao acabou. o rx_bit ficara apenas recessivo
	else
		rx_bit = 1;
end



//generate clock
always
	#5 clk = !clk;
	
//generate sample_point
always @(posedge clk)
	if (clk2sample < 5) 
	begin
		clk2sample <= clk2sample + 1;
		sample_point <= 0;
	end
	else
	begin
		clk2sample <= 0;
		sample_point <= 1;
	end

always @(negedge clk)
begin
	sample_point <= 0;
end
	

	
	

endmodule


