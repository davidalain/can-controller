transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -vlog01compat -work work +incdir+C:/Users/luisf/Documents/can-controller/Projeto\ exemplo/Projeto {C:/Users/luisf/Documents/can-controller/Projeto exemplo/Projeto/can_crc.v}
vlog -vlog01compat -work work +incdir+C:/Users/luisf/Documents/can-controller/Projeto\ exemplo/Projeto {C:/Users/luisf/Documents/can-controller/Projeto exemplo/Projeto/can_decoder.v}

