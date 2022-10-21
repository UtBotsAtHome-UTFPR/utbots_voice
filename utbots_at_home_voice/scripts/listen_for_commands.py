#!/usr/bin/env python3
from time import sleep
import rospy
from std_msgs.msg import String
import nltk
import sys
import os
from gensim.models.doc2vec import Doc2Vec
from custom_msg.msg import set_angles

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

		self.pub_setAngle = rospy.Publisher('/cmd_3R', set_angles, queue_size=1)

		print("\nArquivos:")
		self.arqFrasesCorretas = rospy.get_param("~arqFrasesCorretas")
		print(" - ", self.arqFrasesCorretas)
		self.arqRespostas = rospy.get_param("~arqRespostas")
		print(" - ", self.arqRespostas)
		self.arqModelo = rospy.get_param("~arqModelo")
		print(" - ", self.arqModelo)

		self.esperandoPergunta = False
		self.failed_call = False

		self.pub_command = rospy.Publisher('/voice_commands', String, queue_size=1)
		self.sub_recognized = rospy.Subscriber('text_recognized', String, self.callback)
		self.pub_resposta = rospy.Publisher('/tts', String, queue_size=1)

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

		data.data = data.data.lower()
		rospy.loginfo(data.data)

		fraseEncontrada = data.data
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
			msg_setAngle = set_angles()
			msg_setAngle.set_Kp_GAR = 0.3
			if respostaEncontrada == "Hold":
				self.pub_resposta.publish("Closing hand now, bruh. When you're ready, call my name, wait and say follow me")
				self.pub_command.publish("Hold")
				msg_setAngle.set_GAR = 120
				self.pub_setAngle.publish(msg_setAngle)
				rospy.loginfo('Apollo: someone called me!')
			elif respostaEncontrada == "Carry my luggage":
				self.pub_resposta.publish("Ok. Put the bag in my hand, call my name, wait and say hold")
				msg_setAngle.set_GAR = 180
				self.pub_setAngle.publish(msg_setAngle)
			elif respostaEncontrada == "Follow me":
				self.pub_resposta.publish("I will follow you, bruh. When we reach the car, call my name, wait and say stop")
				self.pub_command.publish("Follow me")
			elif(respostaEncontrada == "Stop"):
				self.pub_resposta.publish("I will stop now, bruh. I will open my hand, pick up your bag, please")
				self.pub_command.publish("Stop")
				msg_setAngle.set_GAR = 180
				self.pub_setAngle.publish(msg_setAngle)
			else:
				self.pub_resposta.publish(respostaEncontrada)
		sleep(3)

	def loop(self):
		rospy.spin()
'''
		if ("apollo" in data.data) or (data.data == "follow") or ("hello" in data.data) or ("barlow" in data.data) or ("hello apollo" in data.data):
			self.esperandoPergunta = True
			rospy.loginfo('Apollo: someone called me!')
			self.pub_resposta.publish("Yes bruh?")
		elif self.esperandoPergunta == True:
			self.esperandoPergunta = False
			if(("road" in data.data) or ("hold" in data.data) or ("old" in data.data) or ("cold" in data.data) or ("kohl's" in data.data) or ("close" in data.data) or ("holt" in data.data)):
				self.pub_resposta.publish("Closing hand now, bruh. When you're ready, call my name, wait and say follow me")
				self.pub_command.publish("Hold")
				rospy.loginfo('Apollo: someone called me!')
			elif(("carry" in data.data) or ("luggage" in data.data)):
				self.pub_resposta.publish("Ok. Put the bag in my hand, call my name, wait and say hold")
				rospy.loginfo('Apollo: someone called me!')
			elif(("follow" in data.data) or ("hollow" in data.data) or ("colonie" in data.data) or ("colony" in data.data)):
				self.pub_resposta.publish("I will follow you, bruh. When we reach the car, call my name, wait and say stop")
				self.pub_command.publish("Follow me")
			elif(("stop" in data.data)):
				self.pub_resposta.publish("I will stop now, bruh. I will open my hand, pick up your bag, please")
				self.pub_command.publish("Stop")
			elif(("i love you" in data.data)):
				self.pub_resposta.publish("I love you too, bruh.")
			elif(("thank you" in data.data)):
				self.pub_resposta.publish("You're welcome, bruh.")
			elif(("66" in data.data)):
				self.pub_resposta.publish("Commencing Jedi extermination. Beg and you will not be spared")
			else:
				self.pub_resposta.publish("Sorry, I could not understand you. I will remain stopped. Please repeat, bruh.")
				self.pub_command.publish("Stop")
			self.failed_call = False
		elif(self.failed_call == False):
			self.failed_call = True
			self.pub_resposta.publish("Sorry, I could not understand you. Call my name if you want something.")
			self.pub_command.publish("Stop")
'''


if __name__ == '__main__':
    Listen()
