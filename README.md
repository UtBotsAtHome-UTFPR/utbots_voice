# apollo_voice

## Para clonar
- ```git clone --recurse-submodules https://github.com/UtBot-UTFPR/apollo_voice.git```

## Pacotes
- ### voztts (para text-to-speech)
    - MaryTTS
        - O que é
            - API de TTS
        - Instalar Docker
            - ```sudo apt install docker```
        - Baixar imagem do MaryTTS
            - ```sudo docker pull synesthesiam/marytts:5.2```
        - Rodar MaryTTS
            - ```sudo docker run -it -p 59125:59125 synesthesiam/marytts:5.2 --voice cmu-slt-hsmm```
        - Ver no navegador se o server está disponível
            - ```http://localhost:59125/```
- ### utbots_at_home_voice (para speech-to-text)
    - PENDENTE