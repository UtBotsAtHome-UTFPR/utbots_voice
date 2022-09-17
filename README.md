# apollo_voice

## Para clonar
- ```git clone --recurse-submodules https://github.com/UtBot-UTFPR/apollo_voice.git```

## Pacotes
- ### voztts (para text-to-speech)
    - Pré-requisitos
        - ```sudo apt install docker mplayer```
    - MaryTTS
        - O que é
            - API de TTS
        - Baixar imagem do MaryTTS
            - ```sudo docker pull synesthesiam/marytts:5.2```
        - Rodar MaryTTS
            - ```sudo docker run -it -p 59125:59125 synesthesiam/marytts:5.2 --voice cmu-slt-hsmm```
        - Ver no navegador se o server está disponível
            - ```http://localhost:59125/```
    - Rodar MaryTTS e nodo ROS
        - Launch
            - ```roslaunch voztts vozttp.launch```
        - Para testar o text-to-speech
            - ```rostopic pub /tts std_msgs/String "HELLO WORLD"```
        - Para testar a emoção (mudar emoção e depois publicar outra mensagem tts)
            - ```rostopic pub /emotion std_msgs/String "rage"```
        - Tópicos do nodo voztts
            - Subscribers
                - /tts (std_msgs/String)
                    - Texto a ser sintetizado
                - /emotion (std_msgs/String)
                    - Altera o tom de voz
                    - Emoções possíveis
                        - "rage"
                        - "annoyance"
                        - "anger"
                        - "idle"
                        - "terror"
                        - "fear"
                        - "apprehension"
                        - "ecstasy"
                        - "joy"
                        - "serenity"
                        - "grief"
                        - "sadness"
                        - "pensiveness"
                        - "vigilance"
                        - "antecipation"
                        - "interest"
                        - "loathing"
                        - "disgust"
                        - "boredom"
                        - "amazement"
                        - "surprise"
                        - "distraction"
                        - "admiration"
                        - "trust"
                        - "acceptance"
        
- ### utbots_at_home_voice (para speech-to-text)
    - PENDENTE