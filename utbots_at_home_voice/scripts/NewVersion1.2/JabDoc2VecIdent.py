# Identificador de frases usando Doc2Vec
#
# (c) )2020 UTFPR/CT/DAINF - Jose Buiar
#
# 1.2 - 09nov20 - Retornando Resposta Correta
#
import nltk
from gensim.models.doc2vec import Doc2Vec
from nltk.tokenize import word_tokenize
import pandas as pd
arqFrasesCorretas = 'FrasesCorretas.csv'

def identificar(frase, arqModelo):
	#Carrega base ja treinada
	model= Doc2Vec.load(arqModelo)
	test_data = nltk.word_tokenize(frase.lower()) # converte para lowercase
	v1 = model.infer_vector(test_data)
	similar_doc =  model.docvecs.most_similar([v1]) # procura frases similades na base treinada
	df = pd.read_csv(arqFrasesCorretas, delimiter=';',  encoding = "utf8", error_bad_lines=False)
	for sim in similar_doc:
		docId=int(sim[0])
		encontrado =  df.loc[df['id'] == docId] # frase similar encontrada
		prob = sim[1]
		return docId, prob, encontrado['text'].iloc[0].strip(), encontrado['answer'].iloc[0].strip()
