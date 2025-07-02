#!/usr/bin/env python3
import os
import time
import argparse
import requests
import rclpy
from rclpy.node import Node
from langserve import RemoteRunnable
from tempfile import NamedTemporaryFile
import playsound
try:
    import speech_recognition as sr
except ImportError:
    sr = None

class AIChatNode(Node):
    def __init__(self, test_mode: bool):
        super().__init__('ai_chat_node')
        self.test_mode = test_mode
        self.base = os.getenv('API_BASE', 'https://hubot-api-lara-production.up.railway.app')
        self.token = None
        self.token_expiry = 0

        if sr:
            self.recognizer = sr.Recognizer()
            self.recognizer.dynamic_energy_threshold = True
            self.recognizer.energy_threshold = int(os.getenv('ENERGY_THRESHOLD', '300'))
        else:
            self.recognizer = None

        self._refresh_token()
        if not test_mode and self.recognizer:
            self.create_timer(5.0, self.listen_and_respond)

    def _refresh_token(self):
        secret = os.getenv('SECRET_KEY', '')
        resp = requests.get(f"{self.base}/get_access_token", headers={'X-token': secret}, timeout=5)
        resp.raise_for_status()
        data = resp.json()
        self.token = data['access_token']
        self.token_expiry = time.time() + data.get('expires_in', 300) - 5
        self.ai = RemoteRunnable(f"{self.base}/chat", headers={'temp-token': self.token})

    def _get_token(self):
        if not self.token or time.time() >= self.token_expiry:
            self._refresh_token()
        return self.token

    def listen_and_respond(self):
        idx = int(os.getenv('MIC_DEVICE_INDEX', -1))
        with sr.Microphone(device_index=idx if idx>=0 else None) as mic:
            self.recognizer.adjust_for_ambient_noise(mic, 0.5)
            audio = self.recognizer.listen(mic, phrase_time_limit=5)
        try:
            query = self.recognizer.recognize_google(audio, language=os.getenv('STT_LANGUAGE', 'pt-BR'))
        except Exception:
            return
        self._call_ai(query)

    def _call_ai(self, query: str):
        self.get_logger().info(f'Ouvindo: {query}')
        try:
            token = self._get_token()
            result = self.ai.invoke({'message': query, 'chat_history': []})
        except Exception as e:
            self.get_logger().error(f'Erro IA: {e}')
            return

        resposta = result['text'] if isinstance(result, dict) else str(result)

        try:
            self._refresh_token()
            resp = requests.post(f"{self.base}/tts/", json={'text': resposta}, headers={'temp-token': self.token}, timeout=10)
            resp.raise_for_status()
            with NamedTemporaryFile(delete=False, suffix='.mp3') as tmp:
                tmp.write(resp.content)
                tmp.flush()
            playsound.playsound(tmp.name)
        except Exception as e:
            self.get_logger().error(f'TTS/áudio: {e}')
            print(f'>> IA: {resposta}')
        finally:
            try: os.remove(tmp.name)
            except: pass

    def run_test_loop(self):
        self.get_logger().info('Modo TESTE: digite sua pergunta ou sair')
        while True:
            q = input('Você: ').strip()
            if q.lower() in ('sair', 'exit', 'quit'):
                break
            if q:
                self._call_ai(q)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = AIChatNode(test_mode=args.test)
    try:
        if args.test:
            node.run_test_loop()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
