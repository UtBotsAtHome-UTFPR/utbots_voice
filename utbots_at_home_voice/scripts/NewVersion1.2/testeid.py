# Teste do Identificador de frases usando TF-IDF
#
# (c) )2020 UTFPR/CT/DAINF - Jose Buiar
#
# 1.1 - 09nov20 - Versão Inicial
#
# Execucao : $ testeid frase a ser identificada
#
import sys 
import JabTFIDFIdent
import JabTextIdent

indice, probabilidade, pergunta, resposta = JabTFIDFIdent.identificar( ' '.join(sys.argv[1:]) )

print ( "TF-IDF %d\t%5.1f%%\n%s\n%s " % (indice, probabilidade, pergunta, resposta) )
#if ( probabilidade < 0.01 ) :
indice, probabilidade, pergunta, resposta = \
		JabTextIdent.identificar( ' '.join(sys.argv[1:]), 'd2v_e10_v300.model' )
print ( "Doc2Vec %d\t%5.1f%%\n%s\n%s " % (indice, probabilidade, pergunta, resposta) )
