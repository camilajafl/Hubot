#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import os
import time
import requests
from langserve import RemoteRunnable
from tempfile import NamedTemporaryFile
import playsound
import json 

try:
    import speech_recognition as sr
    from langdetect import detect
except ImportError:
    sr = None
    detect = None

class AIChatNode(Node):
    def __init__(self, test_mode: bool):
        super().__init__('ai_chat_node')
        self.test_mode = test_mode

        # Token e expiração
        self.token = None
        self.token_expiry = 0

        self.session  = requests.Session()

        # Base HTTP:
        # RAILWAY:
        self.base = "https://hubot-api-lara-production.up.railway.app"
        # LOCALHOST:
        #self.base = "http://localhost:8000"

        # Inicializa STT
        if sr:
            self.recognizer = sr.Recognizer()
            self.recognizer.dynamic_energy_threshold = True
            self.recognizer.energy_threshold = int(os.getenv('ENERGY_THRESHOLD', '300'))
        else:
            self.recognizer = None

        self.chat_history: list[str] = []

        # Listar microfones disponíveis
        if sr:
            try:
                mics = sr.Microphone.list_microphone_names()
                self.get_logger().info("Microfones disponíveis:")
                for i, name in enumerate(mics):
                    self.get_logger().info(f"  {i}: {name}")
            except Exception as e:
                self.get_logger().warn(f"Não foi possível listar microfones: {e}")

        # Instancia o client da API com token inicial
        self._refresh_token()

        # Timer para escutar continuamente
        if not test_mode:
            self.create_timer(5.0, self.listen_and_respond)

    def _get_valid_token(self):
        now = time.time()
        if not self.token or now >= self.token_expiry:
            self._refresh_token()
        return self.token

    def _refresh_token(self):
        resp = self.session.get(
            f"{self.base}/get_access_token",
            headers={'X-token': os.getenv('SECRET_KEY','')},
            timeout=5.0
        )
        resp.raise_for_status()
        data = resp.json()
        self.token = data.get('access_token', '')
        expires_in = data.get('expires_in', 300)
        self.token_expiry = time.time() + expires_in - 5
        

    def listen_and_respond(self):
        mic_index = int(os.getenv('MIC_DEVICE_INDEX', '-1'))
        mic_args = {'device_index': mic_index} if mic_index >= 0 else {}
        with sr.Microphone(**mic_args) as mic:
            self.recognizer.adjust_for_ambient_noise(mic, duration=0.5)
            self.get_logger().info('Ouvindo… fale algo')
            audio = self.recognizer.listen(mic, phrase_time_limit=5)
        try:
            # Converte áudio para WAV bytes
            wav_data = audio.get_wav_data()

            # 1) Garante token fresco para STT (não reutilizar o já consumido)
            self._refresh_token()
            token_stt = self.token

            # Prepara arquivo para Whisper: bytes em BytesIO com atributo name
            import io
            audio_file = io.BytesIO(wav_data)
            audio_file.name = 'audio.wav'

            # Monta payload multipart usando campo 'payload' para corresponder ao endpoint
            files = {'payload': (audio_file.name, audio_file, 'audio/wav')}

            # 2) Chama o endpoint /stt/ com token recém obtido
            resp = self.session.post(
                f"{self.base}/stt/",
                headers={'temp-token': token_stt},
                files=files,
                timeout=10.0
            )
            resp.raise_for_status()
            query = resp.json().get('text', '')
        except Exception as e:
            self.get_logger().warn(f'STT falhou: {e}')
            return

        # Continua o fluxo normal
        self._call_ai(query)
    
    def _call_ai(self, query: str):
        # 1) pega token só para o chat
        self._refresh_token()
        token_chat = self.token

        # 2) cria o RemoteRunnable e dispara o stream
        hubot = RemoteRunnable(f"{self.base}/chat", headers={'temp-token': token_chat})

        collected = ""
        print("\n>> IA: ", end="", flush=True)
        for chunk in hubot.stream({
            "message": query,
            "chat_history": self.chat_history
        }):
            # chunk pode ser str ou dict{'delta': 'texto'}
            delta = chunk['delta'] if isinstance(chunk, dict) else str(chunk)
            collected += delta
            print(delta, end="", flush=True)
        print()  # só pra pular linha no final

        # 3) atualiza o histórico
        self.chat_history.append(f"Você: {query}")
        self.chat_history.append(f"IA: {collected}")

        # 4) token para TTS e toca a resposta inteira
        self._refresh_token()
        token_tts = self.token
        resp = self.session.post(
            f"{self.base}/tts/",
            json={"text": collected},
            headers={"temp-token": token_tts},
            timeout=20.0
        )
        resp.raise_for_status()

        with NamedTemporaryFile(delete=False, suffix=".mp3") as tmp:
            tmp.write(resp.content)
            tmp.flush()
        playsound.playsound(tmp.name)
        os.remove(tmp.name)


    def run_test_loop(self):
        self.get_logger().info("Modo TESTE: digite sua pergunta ou 'sair'")
        while True:
            query = input('Você: ')
            if query.strip().lower() in ('sair','exit','quit'):
                break
            if not query.strip():
                continue
            self._call_ai(query)


def main(args=None):
    print("mudou")
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true', help='Modo on-PC via teclado')
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = AIChatNode(test_mode=parsed.test)
    try:
        if parsed.test:
            node.run_test_loop()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
