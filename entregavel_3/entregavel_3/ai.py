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

import threading
import io
import wave
import simpleaudio as sa

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

        self.is_speaking = False

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
        if self.is_speaking: #ignora microfone 
            return
        
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
            self.get_logger().info(f"Você disse: {query}")

        except Exception as e:
            self.get_logger().warn(f'STT falhou: {e}')
            return

        # Continua o fluxo normal
        self._call_ai(query)
    
    def tocar_audio_em_thread(self, wav_bytes):
        try:
            self.is_speaking = True

            start_decode = time.time()
            with io.BytesIO(wav_bytes) as audio_stream:
                with wave.open(audio_stream, 'rb') as wave_read:
                    wave_obj = sa.WaveObject(
                        wave_read.readframes(wave_read.getnframes()),
                        wave_read.getnchannels(),
                        wave_read.getsampwidth(),
                        wave_read.getframerate()
                    )
            end_decode = time.time()
            print(f"[TTS-thread] Decodificação WAV: {end_decode - start_decode:.2f} segundos")

            start_play = time.time()
            play_obj = wave_obj.play()
            play_obj.wait_done()
            end_play = time.time()
            print(f"[TTS-thread] Reprodução do áudio: {end_play - start_play:.2f} segundos")

            total = end_play - start_decode
            print(f"[TTS-thread] Tempo TOTAL na thread: {total:.2f} segundos")
        finally:
            self.is_speaking = False


    
    def _call_ai(self, query: str):
        # Token novo só para o chat
        self._refresh_token()
        token_chat = self.token

        # Instancia RemoteRunnable
        hubot = RemoteRunnable(f"{self.base}/chat", headers={'temp-token': token_chat})

        collected = ""
        print("\n>> IA: ", end="", flush=True)
        try:
            for chunk in hubot.stream({
                "message": query,
                "chat_history": self.chat_history
            }):
                delta = chunk['delta'] if isinstance(chunk, dict) else str(chunk)
                collected += delta
                print(delta, end="", flush=True)
        except Exception as e:
            self.get_logger().error(f"Erro no stream do chat: {e}")
            return
        print()

        # Atualiza o histórico no formato correto
        self.chat_history.append({"author": "user", "content": query})
        self.chat_history.append({"author": "ai", "content": collected})

        # TTS ---------------------------------------------------------------------
        # TTS ---------------------------------------------------------------------
        start_tts_total = time.time()

        # Token novo só para o TTS
        self._refresh_token()
        token_tts = self.token

        start_req = time.time()
        resp = self.session.post(
            f"{self.base}/tts/",
            json={"text": collected},
            headers={"temp-token": token_tts},
            timeout=30.0
        )
        resp.raise_for_status()
        end_req = time.time()
        print(f"[TTS] Requisição à API: {end_req - start_req:.2f} segundos")

        # Toca o áudio em uma thread separada
        threading.Thread(target=self.tocar_audio_em_thread, args=(resp.content,)).start()

        # Marca tempo total até delegar para thread
        end_tts_total = time.time()
        print(f"[TTS] Tempo TOTAL (até início da reprodução): {end_tts_total - start_tts_total:.2f} segundos")


        print("", flush=True)


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
    print("mudou?")
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
