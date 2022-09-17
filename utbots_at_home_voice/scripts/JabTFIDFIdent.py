# Identificador de frases usando TF-IDF
#
# (c) )2020 UTFPR/CT/DAINF - Jose Buiar
#
# 1.0 - 08nov20 - Versão Inicial
#
arquivoCorretas="FrasesCorretas.csv"  

from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import linear_kernel
from nltk import word_tokenize          
from nltk.stem import WordNetLemmatizer
import nltk
import pandas as pd

class LemmaTokenizer:
    ignore_tokens = [',', '.', ';', ':', '"', '``', "''", '`']
    def __init__(self):
        self.wnl = WordNetLemmatizer()
    def __call__(self, doc):
        return [self.wnl.lemmatize(t) for t in word_tokenize(doc) if t not in self.ignore_tokens]

def identificar(frase):
	tokenizer=LemmaTokenizer()
	df = pd.read_csv(arquivoCorretas, delimiter=';',  encoding = "utf8", error_bad_lines=False)
	documents=[]
	corretas=[]
	for index, row in df.iterrows():
		documents.append(row['text'])
		corretas.append(row['text'])
	vectorizer = TfidfVectorizer(tokenizer=tokenizer)
	doc_vectors = vectorizer.fit_transform([frase] + documents)
	cosine_similarities = linear_kernel(doc_vectors[0:1], doc_vectors).flatten()
	document_scores = [item.item() for item in cosine_similarities[1:]]
	max_value = max(document_scores)
	max_index = document_scores.index(max_value)
	return max_index, max_value, corretas[max_index]
