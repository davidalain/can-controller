//====================================================================
//====================== Includes ====================================
//====================================================================

//`include "can_decoder.v"

//====================================================================
//============== Declaração do módulo ================================
//====================================================================

module main_teste(
	clock
);

input wire clock;

//====================================================================
//===================== Constantes ===================================
//====================================================================

parameter len_start_of_frame	= 1'd1;
parameter len_id_a 				= 4'd11;
parameter len_rtr 				= 1'd1;
parameter len_srr 				= 1'd1;
parameter len_ide 				= 1'd1;
parameter len_reserved1 		= 1'd1;
parameter len_reserved0			= 1'd1;
parameter len_id_b 				= 5'd18;
parameter len_dlc 				= 3'd4;
parameter len_crc				= 4'd15;
parameter len_crc_delimiter		= 1'd1;
parameter len_ack_slot			= 1'd1;
parameter len_ack_delimiter		= 1'd1;
parameter len_end_of_frame		= 3'd7;

parameter len_interframe		= 2'd2; //De acordo com a especificação são 3 bits recessivos. Mas na prática pode acontecer a transmissão de frames em sequência em que o terceiro bit do Intermission já pode ser o Start of Frame do próximo frame, então são 2. Vide documento can2spec.pdf, seção 9.1, item 2.


//====================================================================
//===================== Variáveis ====================================
//====================================================================

reg 		field_start_of_frame;
reg[10:0]	field_id_a;
reg			field_srr;
reg			field_ide;
reg			field_rtr;
reg			field_reserved1;
reg			field_reserved0;
reg[17:0]	field_id_b;
reg[3:0]	field_dlc;
reg[63:0]	field_data;
reg[14:0]	field_crc;
reg			field_crc_delimiter;
reg			field_ack_slot;
reg			field_ack_delimiter;
reg[6:0]	field_end_of_frame;

reg[127:0]	frame;
reg[7:0]	frame_len;
reg[7:0]	frame_id;
reg[7:0] 	index;

reg			reset;
reg			rx_bit;
reg			sample_point;
reg			error_in;

//====================================================================
//============ Lógica combinacional  =================================
//====================================================================

assign sample_point = clock;  //Testando por enquanto com mesma fonte de clock
assign error_in = 0;				//Sem erros informados por outros módulos

//====================================================================
//================= Inicialização do módulo ==========================
//====================================================================

can_decoder can_decoder_inst
(
	.clock(clock) ,	// input  clock_sig
	.reset(reset) ,	// input  reset_sig
	.rx_bit(rx_bit) ,	// input  rx_bit_sig
	.sample_point(sample_point) ,	// input  sample_point_sig
	.error_in(error_in),	// input  error_in_sig
	.error_out(error_out_sig) ,	// output  error_out_sig
	.field_start_of_frame(field_start_of_frame_sig) ,	// output  field_start_of_frame_sig
	.field_id_a(field_id_a_sig) ,	// output [10:0] field_id_a_sig
	.field_ide(field_ide_sig) ,	// output  field_ide_sig
	.field_rtr(field_rtr_sig) ,	// output  field_rtr_sig
	.field_srr(field_srr_sig) ,	// output  field_srr_sig
	.field_reserved1(field_reserved1_sig) ,	// output  field_reserved1_sig
	.field_reserved0(field_reserved0_sig) ,	// output  field_reserved0_sig
	.field_id_b(field_id_b_sig) ,	// output [17:0] field_id_b_sig
	.field_dlc(field_dlc_sig) ,	// output [3:0] field_dlc_sig
	.field_data(field_data_sig) ,	// output [63:0] field_data_sig
	.field_crc(field_crc_sig) ,	// output [14:0] field_crc_sig
	.field_crc_delimiter(field_crc_delimiter_sig) ,	// output  field_crc_delimiter_sig
	.field_ack_slot(field_ack_slot_sig) 	// output  field_ack_slot_sig
);

//========



//====================================================================
//================= Tasks criação dos frames =========================
//====================================================================

task configTeste0;
	begin
		field_start_of_frame 	<= 1'b0;		//Sempre dominante (0)
		field_id_a 				<= 11'b0;	//ID
		field_rtr				<= 1'b0;		//RTR=0 (0 = Data Frame, 1 = Remote Frame)
		field_srr 				<= 1'bx;		//Não usado no Frame padrão (só no frame extendido)
		field_ide				<= 1'b0;		//IDE=0 (0 = Frame base, 1 = Frame Extendido)
		field_reserved1			<= 1'bx;		//Não usado no Frame base (só no frame extendido)
		field_reserved0			<= 1'b0;		//Deve ser dominante (0), mas aceita ambos os valores
		field_id_b				<= 18'bx;	//Não usado no Frame padrão (só no frame extendido).
		field_dlc				<= 4'd1;		//Quantidade bytes no campo data.
		field_data				<= 8'hFF;	//
		field_crc				<= 15'h3FFF;	//CRC (0x3FFF = não calculado, vai dar erro de CRC)
		field_crc_delimiter		<= 1'b1;		//Deve ser recessivo (1)
		field_ack_slot			<= 1'b1;		//É enviado recessivo (1) e o nó receptor que seta dominante (0)!
		field_ack_delimiter		<= 1'b1;		//Deve ser recessivo (1)
		field_end_of_frame		<= 7'h3F;	//Deve ser recessivo (1)
		
		frame <= {
			field_start_of_frame, 
			field_id_a, 
			field_rtr, 
			field_ide, 
			field_reserved0, 
			field_dlc, 
			field_data[field_dlc*8-1:0], 
			field_crc, 
			field_crc_delimiter, 
			field_ack_slot,
			field_ack_delimiter,
			field_end_of_frame,
		};
		
		frame_len <= 
			len_start_of_frame + 
			len_id_a +
			len_rtr +
			len_ide +
			len_reserved0 +
			len_dlc +
			(field_dlc*8) + //data len
			len_crc +
			len_crc_delimiter +
			len_ack_slot +
			len_ack_delimiter +
			len_end_of_frame;
			
	end
endtask

initial
begin
	frame_id <= 0;
	index <= 0;
	reset <= 1;
end

always @ (posedge clock or posedge reset)
begin
	case(frame_id)
		0: 
			if(reset == 1)
			begin
				configTeste0();
				reset <= 0;
			end
			else
			begin
				rx_bit <= frame[frame_len - index];
				index <= index + 1;
			end
		
		1: 
			if(reset == 1)
			begin
				//configTeste1();
				reset <= 0;
			end
			else
			begin
				rx_bit <= frame[frame_len - index];
				index <= index + 1;
			end
		
	endcase
end


endmodule














