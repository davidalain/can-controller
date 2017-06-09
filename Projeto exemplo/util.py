import sys

hexConst = ['1','0','0','0','1','0','1','1','0','0','1','1','0','0','1']

def maior (x,y):
  if(x>y):
    return x
  else:
    return y
	
def xorStringBinaria(str1,str2):
  #print(''.join(str1) + ' - ' + ''.join(str2))
  resultado = []
  for i in range (0,maior(len(str1),len(str2))):
    if(str1[i] == str2[i]):
      resultado.append('0')
    else:
      resultado.append('1')
 # print(''.join(resultado))
  return resultado
  
def calcCRC(stringBinaria):
  crcFinal = ['0','0','0','0','0','0','0','0','0','0','0','0','0','0','0']
  index = 0
  for c in stringBinaria:
    if(c == crcFinal[0]):
      crcnxt = '0'	
    else:
      crcnxt = '1'
    crcFinal.pop(0)
    crcFinal.append('0')
    if(crcnxt == '1'):
      crcFinal = xorStringBinaria(crcFinal, hexConst)
    #    print(c)
    index = index +1
    print(str(index) + ' - ' +str(c)+' - ' +  hex(int((''.join(crcFinal)),2) ) )
  return crcFinal
  
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
  index = 0
  for c in binString:
    index = index + 1
     # Se nao houve nem 5 bits, nao ha destuff
    if (len(lastFive) < 5):
	  lastFive.append(c)
	  stringFinal.append(c)
    else:
	# se os ultimos bits forem iguais, nao salvamos o bit atual
      if(allEqual(lastFive)):
        if(c == lastFive[0]):
		  print('Erro de bit stuffing - ' + str(index))
	# se nao ha destuff, entao salva o bit atual
      else:
        stringFinal.append(c)
	#independente de salvar ou nao o bit, ele eh adicionado na lista dos ultimos 5	
      lastFive.append(c)
      lastFive.pop(0)	  
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


print(''.join(destuff(sys.argv[1])))  
print(''.join(calcCRC(destuff(sys.argv[1]))))