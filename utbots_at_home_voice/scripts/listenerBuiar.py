import rospy
from std_msgs.msg import String
#from JabTextIdent import *
#import nltk,sys,os
#from gensim.models.doc2vec import Doc2Vec
#import pandas as pd
arqFrasesCorretas = 'FrasesCorretas.csv'
arqModelo = 'd2v_e1_v300.model'

def identificar(frase):
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
		qtd = qtd + 1
		return docId, prob, encontrado['text'].iloc[0].strip()

def callback(data):
#    indice, probabilidade, fraseEncontrada = identificar( data.data )
    fraseEncontrada = data.data
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', fraseEncontrada)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('text_recognized', String, callback)
    rospy.spin()
