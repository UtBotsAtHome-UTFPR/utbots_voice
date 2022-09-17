import rospy
from std_msgs.msg import String
import nltk,sys,os
from gensim.models.doc2vec import Doc2Vec

from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import linear_kernel
from nltk import word_tokenize          
from nltk.stem import WordNetLemmatizer
import pandas as pd

arqFrasesCorretas = 'FrasesCorretas.csv'
arqRespostas = 'Respostas.csv'
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
		return docId, prob, encontrado['text'].iloc[0].strip()

def callback(data):
    #fraseEncontrada = data.data
    indice, probabilidade, fraseEncontrada = identificar( data.data )
    rospy.loginfo('I heard      : %s', data.data)
    rospy.loginfo('I understand : %s', fraseEncontrada)
    rospy.loginfo('')

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('text_recognized', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
