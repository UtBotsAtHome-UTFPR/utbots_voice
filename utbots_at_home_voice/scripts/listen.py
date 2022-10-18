#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import nltk
import sys
import os
from gensim.models.doc2vec import Doc2Vec

from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import linear_kernel
from nltk import word_tokenize
from nltk.stem import WordNetLemmatizer
import pandas as pd

# nltk.download('wordnet')
# nltk.download('punkt')
# nltk.download('omw-1.4')


class LemmaTokenizer:
    ignore_tokens = [',', '.', ';', ':', '"', '``', "''", '`']

    def __init__(self):
        self.wnl = WordNetLemmatizer()

    def __call__(self, doc):
        return [self.wnl.lemmatize(t) for t in word_tokenize(doc) if t not in self.ignore_tokens]


class Listen:
	def __init__(self):
		rospy.init_node('listener', anonymous=True)

		print("\nArquivos:")
		self.arqFrasesCorretas = rospy.get_param("~arqFrasesCorretas")
		print(" - ", self.arqFrasesCorretas)
		self.arqRespostas = rospy.get_param("~arqRespostas")
		print(" - ", self.arqRespostas)
		self.arqModelo = rospy.get_param("~arqModelo")
		print(" - ", self.arqModelo)

		self.esperandoPergunta = False

		self.pub_resposta = rospy.Publisher('/tts', String, queue_size=1)
		self.sub_recognized = rospy.Subscriber('text_recognized', String, self.callback)

		self.loop()

	def identificar(self, frase):
		tokenizer = LemmaTokenizer()
		df = pd.read_csv(self.arqFrasesCorretas, delimiter=';',
						encoding="utf8", on_bad_lines="warn")
		corretas = []
		respostas = []
		for index, row in df.iterrows():
			corretas.append(row['text'])
			respostas.append(row['answer'])
		vectorizer = TfidfVectorizer(tokenizer=tokenizer)
		doc_vectors = vectorizer.fit_transform([frase] + corretas)
		cosine_similarities = linear_kernel(
			doc_vectors[0:1], doc_vectors).flatten()
		document_scores = [item.item() for item in cosine_similarities[1:]]
		max_value = max(document_scores)
		max_index = document_scores.index(max_value)
		return max_index, max_value, corretas[max_index], respostas[max_index]


	def identificarDV(self, frase):
		# Carrega base ja treinada
		model = Doc2Vec.load(self.arqModelo)
		test_data = nltk.word_tokenize(frase.lower())  # converte para lowercase
		v1 = model.infer_vector(test_data)
		# procura frases similades na base treinada
		similar_doc = model.docvecs.most_similar([v1])
		df = pd.read_csv(self.arqFrasesCorretas, delimiter=';',
						encoding="utf8", error_bad_lines=False)
		for sim in similar_doc:
			docId = int(sim[0])
			encontrado = df.loc[df['id'] == docId]  # frase similar encontrada
			prob = sim[1]
			return docId, prob, encontrado['text'].iloc[0].strip()

	def callback(self, data):
		#fraseEncontrada = data.data
		#indice, probabilidade, fraseEncontrada = identificar( data.data )
		indice, probabilidade, fraseEncontrada, respostaEncontrada = self.identificar(
			data.data)

		rospy.loginfo('I heard      : %s', data.data)
		rospy.loginfo('I understand : %s', fraseEncontrada)
		rospy.loginfo('The answer is: %s', respostaEncontrada)
		rospy.loginfo('')

		if fraseEncontrada == "Apollo" or fraseEncontrada == "Follow" or fraseEncontrada == "Hello" or fraseEncontrada == "Barlow" or fraseEncontrada == "Hello Apollo":
			self.esperandoPergunta = True
			rospy.loginfo('Apollo: someone called me!')
			self.pub_resposta.publish(respostaEncontrada)
		elif self.esperandoPergunta == True:
			rospy.loginfo('Apollo: publishing answer to listed question')
			self.esperandoPergunta = False
			self.pub_resposta.publish(respostaEncontrada)


	def loop(self):
		rospy.spin()


if __name__ == '__main__':
    Listen()
