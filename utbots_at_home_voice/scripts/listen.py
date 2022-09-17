#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import nltk,sys,os
from gensim.models.doc2vec import Doc2Vec

from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import linear_kernel
from nltk import word_tokenize          
from nltk.stem import WordNetLemmatizer
import pandas as pd
#nltk.download('wordnet')

arqFrasesCorretas = '/home/fabro/catkin_ws/src/utbots_at_home_voice/scripts/FrasesCorretas.csv'
arqRespostas = '/home/fabro/catkin_ws/src/utbots_at_home_voice/scripts/Respostas.csv'
arqModelo = '/home/fabro/catkin_ws/src/utbots_at_home_voice/scripts/d2v_e1_v300.model'

	
class LemmaTokenizer:
    ignore_tokens = [',', '.', ';', ':', '"', '``', "''", '`']
    def __init__(self):
        self.wnl = WordNetLemmatizer()
    def __call__(self, doc):
        return [self.wnl.lemmatize(t) for t in word_tokenize(doc) if t not in self.ignore_tokens]

def identificar(frase):
	tokenizer=LemmaTokenizer()
	df = pd.read_csv(arqFrasesCorretas, delimiter=';',  encoding = "utf8", error_bad_lines=False)
	corretas=[]
	respostas=[]
	for index, row in df.iterrows():
		corretas.append(row['text'])
		respostas.append(row['answer'])
	vectorizer = TfidfVectorizer(tokenizer=tokenizer)
	doc_vectors = vectorizer.fit_transform([frase] + corretas)
	cosine_similarities = linear_kernel(doc_vectors[0:1], doc_vectors).flatten()
	document_scores = [item.item() for item in cosine_similarities[1:]]
	max_value = max(document_scores)
	max_index = document_scores.index(max_value)
	return max_index, max_value, corretas[max_index], respostas[max_index]

def identificarDV(frase):
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
    #indice, probabilidade, fraseEncontrada = identificar( data.data )
    indice, probabilidade, fraseEncontrada, respostaEncontrada = identificar( data.data )
    rospy.loginfo('I heard      : %s', data.data)
    rospy.loginfo('I understand : %s', fraseEncontrada)
    rospy.loginfo('The answer is: %s', respostaEncontrada)
    rospy.loginfo('')

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('text_recognized', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
