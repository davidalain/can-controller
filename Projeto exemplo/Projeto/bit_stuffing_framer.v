
/*
module bit_stuffing_framer(
	rx_bit,
	clock,
	reset,
	error
);

// == Input pins/ports == 
input	wire 	rx_bit;	//Received bit from BTL block (BTL = Bit Timing Logic)
input	wire	clock;	//Received clock pulse from BTL block, indicating there is a bit to read
input	wire 	reset;	

// == Output pins/ports == 
output wire error;

// == Local variables == 

reg[127:0] 		rx_frame;
integer 			rx_index;

reg[127:0] 		tx_frame;
integer			tx_index;

//================ CAN frame fields =============

reg[11:0]		id;
reg[17:0]		id_b;
reg				rtr;
reg				ide;
reg				srr;
reg[3:0]			dlc;
reg[63:0]		data;

//===============================================

// == Behaviour == 

initial
begin
	resetAll();
end

//===============================================================

task resetAll;
	begin
	
		rx_frame = 128'b0; 	//FIXME Pode fazer isso?
		rx_index = 0;
		tx_frame = 128'b0;	//FIXME Pode fazer isso?
		tx_index = 0;

	end
endtask

//===============================================================
always
begin
	if(reset)
	begin
		resetAll();
	end
end

//===============================================================
always @(posedge clock)
begin
	
	//TODO: Implementar a condição de mudança da flag de checagem do bit stuffing
	
	
	if((rx_index > 5) && (check_bit_stuffing == 1))
	begin
	
		//Last 5 bits is equals to 0 and current bit is equals to 1 => Current is a BIT STUFFING (do not store in frame)
		if(rx_frame[rx_index +: 5] == 5'b0)
		begin
		
			if(rx_bit == 1'b1) //Current is a BIT STUFFING (do NOT store in frame)
			begin
			
				$display ("Bit stuffing: bit 1 at offset " + rx_index);
				
			end else
			begin
				
				//Frame de erro
				
			end
			
		end
		
		//last 5 bits is equals to 1 and current bit is equals to 0 => Current is a BIT STUFFING (do not store in frame)
		else if(rx_frame[rx_index +: 5] == 5'b1)
		begin
		
			if(rx_bit == 1'b0)
			begin
			
				$display ("Bit stuffing: bit 0 at offset " + rx_index);
			
			end else
			begin
			
				//Frame de erro
			
			end
			
		end
		
		//Current bit is NOT a BIT STUFFING (do store in frame)
		else
		begin
		
			rx_frame[rx_index] = rx_bit;
			rx_index = rx_index + 1;
		
		end
		
	end
	
end


endmodule
*/