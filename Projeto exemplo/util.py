import sys
def allEqual(lastFive):
  lastC = lastFive[0]
  for c in lastFive:
    if(c!=lastC):
	  return False
  lastC = c
  return True	

def destuff (binString):
  lastFive=[]
  stringFinal = []
  for c in binString:
     # Se nao houve nem 5 bits, nao ha destuff
    if (len(lastFive) < 5):
	  lastFive.append(c)
	  stringFinal.append(c)
    else:
	# se os ultimos bits forem iguais, nao salvamos o bit atual
      if(allEqual(lastFive)):
        if(c == lastFive[0]):
		  print('Erro de bit stuffing')
	# se nao ha destuff, entao salva o bit atual
      else:
        stringFinal.append(c)
	#independente de salvar ou nao o bit, ele eh adicionado na lista dos ultimos 5	
      lastFive.append(c)	
  return stringFinal
		


def stuff (binString):
  lastFive=[]
  stringFinal = []
  for c in binString:
     # Se nao houve nem 5 bits, nao ha destuff
    if (len(lastFive) < 5):
	  lastFive.append(c)
	  stringFinal.append(c)
    else:
	# se os ultimos bits forem iguais, havera bit stuffing
      if(allEqual(lastFive)):
        if (lastFive[0] == '1'):
          stringFinal.append('0')
          lastFive.append('0')		  
	  stringFinal.append(c)
      lastFive.append(c)  
  return stringFinal
  
  
def stuffedBin2DestuffedHex(string):
  return (hex( int( ''.join(destuff(string)) , 2 ) ))

print(stuffedBin2DestuffedHex(sys.argv[1]))