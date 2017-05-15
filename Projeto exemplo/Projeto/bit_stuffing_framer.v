

module bit_stuffing_framer(
	rx_bit,		//Received bit from BTL block (BTL = Bit Timing Logic)
	clock,		//Received clock pulse from BTL block, indicating there is a bit to read
	tx_bit,		//Transmitting bit to BTL block
	
	reset
);

// == Input pins/ports == 
input 	rx_bit;
input		clock;

input		reset;

// == Output pins/ports == 
output 	tx_bit;

// == Data types == 
wire 				rx_bit;
wire 				clock;
wire 				tx_bit;

reg[0:127] 		rx_frame;
integer 			rx_index;

reg[0:127] 		tx_frame;
integer			tx_index;

reg[0:127] 		ready_rx_frame[0:7];
reg[0:127] 		ready_tx_frame[0:7];


// == Behaviour == 

initial
begin
	rx_frame = 128'b0; 	//FIXME Pode fazer isso?
	rx_index = 0;
	tx_frame = 128'b0;	//FIXME Pode fazer isso?
	tx_index = 0;
end

always
begin
	if(reset)
	begin
		
		rx_frame = 128'b0;
		rx_index = 0;

		tx_frame = 128'b0;
		tx_index = 0;
		
	end
end

always @(posedge clock)
begin

	rx_frame[rx_index] = rx_bit;
	
	if(rx_index > 5)
	begin
	
		//Current bit is equals to 1 and last 5 bits is equals to 0 => Current is a BIT STUFFING (do not store in frame)
		if((rx_bit == 1'b1) &&
			((	rx_frame[rx_index-1] |
				rx_frame[rx_index-2] |
				rx_frame[rx_index-3] | 
				rx_frame[rx_index-4] | 
				rx_frame[rx_index-5]) == 1'b0))
		begin
			//Do something here??
			$display ("Bit stuffing: bit 1 at offset " + rx_index);
		end
		
		//Current bit is equals to 0 and last 5 bits is equals to 1 => Current is a BIT STUFFING (do not store in frame)
		else if((rx_bit == 1'b0) &&
				((	rx_frame[rx_index-1] & 
					rx_frame[rx_index-2] & 
					rx_frame[rx_index-3] & 
					rx_frame[rx_index-4] & 
					rx_frame[rx_index-5]) == 1'b1))
		begin
			//Do something here??
			$display ("Bit stuffing: bit 0 at offset " + rx_index);
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