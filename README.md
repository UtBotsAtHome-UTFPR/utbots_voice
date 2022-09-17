# apollo_voice

## Para clonar
- ```bash git clone --recurse-submodules https://github.com/UtBot-UTFPR/apollo_voice.git```

## Pacotes
- ### voztts (para text-to-speech)
    - MaryTTS
        - O que é:
            - API de TTS
        - Caminho dos arquivos
            - ```bash voztts/marytts```
        - Como usar
            - Descomprimir
                - ```bash roscd voztts/marytts```
                - ```bash unzip marytts-installer-5.2-mod.zip```
            - Mudar versão do Gradle (pode ser necessário instalar o Gradle pelo apt)
                - ```bash cd marytts-installer-5.2-mod```
                - ```bash gradle wrapper --gradle-version 5.1.1```
            - Iniciar o MaryTTS server
                - ```bash ./marytts -i```
            - Ver no navegador se o server está disponível:
                - ```http://localhost:59125/```
            - CASO precise reinstalar o marytts (demora para baixar arquivos):
                - ```bash ./marytts install```
- ### utbots_at_home_voice (para speech-to-text)
    - PENDENTE