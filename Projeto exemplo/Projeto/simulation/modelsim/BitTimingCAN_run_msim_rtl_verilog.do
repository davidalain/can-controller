transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -vlog01compat -work work +incdir+D:/Repositorio_de_Projetos/6-DOUTORADO/Automotive\ Networking/CAN\ Controller\ -\ Projeto\ Automotive\ Networking/can-controller/Projeto\ exemplo/Projeto {D:/Repositorio_de_Projetos/6-DOUTORADO/Automotive Networking/CAN Controller - Projeto Automotive Networking/can-controller/Projeto exemplo/Projeto/can_crc.v}
vlog -vlog01compat -work work +incdir+D:/Repositorio_de_Projetos/6-DOUTORADO/Automotive\ Networking/CAN\ Controller\ -\ Projeto\ Automotive\ Networking/can-controller/Projeto\ exemplo/Projeto {D:/Repositorio_de_Projetos/6-DOUTORADO/Automotive Networking/CAN Controller - Projeto Automotive Networking/can-controller/Projeto exemplo/Projeto/can_decoder.v}
vlog -vlog01compat -work work +incdir+D:/Repositorio_de_Projetos/6-DOUTORADO/Automotive\ Networking/CAN\ Controller\ -\ Projeto\ Automotive\ Networking/can-controller/Projeto\ exemplo/Projeto {D:/Repositorio_de_Projetos/6-DOUTORADO/Automotive Networking/CAN Controller - Projeto Automotive Networking/can-controller/Projeto exemplo/Projeto/tester.v}

