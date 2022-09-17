# apollo_voice

## Para clonar
- ```git clone --recurse-submodules https://github.com/UtBot-UTFPR/apollo_voice.git```

## Pacotes
- ### voztts (para text-to-speech)
    - MaryTTS
        - O que é
            - API de TTS
        - Como usar
            - Baixar o zip
                - https://drive.google.com/file/d/1tAt8VRdEwLlYiHbex58DdL1mr8M3muGN/view
            - Mover o zip para
                - ```voztts/marytts```
            - Descomprimir
                - ```roscd voztts/marytts```
                - ```unzip marytts-installer-5.2-mod.zip```
            - Mudar versão do Gradle (pode ser necessário instalar o Gradle pelo apt)
                - ```cd marytts-installer-5.2-mod```
                - ```gradle wrapper --gradle-version 5.1.1```
            - Iniciar o MaryTTS server
                - ```./marytts -i```
            - Ver no navegador se o server está disponível:
                - ```http://localhost:59125/```
            - CASO precise reinstalar o marytts (demora para baixar arquivos):
                - ```./marytts install```
- ### utbots_at_home_voice (para speech-to-text)
    - PENDENTE