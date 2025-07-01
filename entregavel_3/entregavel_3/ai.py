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

# Bibliotecas para STT e detecção de idioma
try:
    import speech_recognition as sr
    from langdetect import detect
except ImportError:
    sr = None
    detect = None

# Bibliotecas para TTS de alta qualidade
ELEVEN_AVAILABLE = False
try:
    from elevenlabs import set_api_key, generate, play
    ELEVEN_AVAILABLE = True
except ImportError:
    ELEVEN_AVAILABLE = False

# Fallback TTS
try:
    from gtts import gTTS
    import playsound
    import tempfile
    TTS_FALLBACK = 'gtts'
except ImportError:
    TTS_FALLBACK = None

class AIChatNode(Node):
    def __init__(self, test_mode: bool):
        super().__init__('ai_chat_node')
        self.test_mode = test_mode

        # Token e expiração
        self.token = None
        self.token_expiry = 0

        #Base HTTP:
            #RAILWAY:
        self.base = "https://hubot-api-lara-production.up.railway.app"
            #LOCALHOST:
        #self.base = "http://localhost:8000"


        # Inicializa STT
        if sr:
            self.recognizer = sr.Recognizer()
            self.recognizer.dynamic_energy_threshold = True
            self.recognizer.energy_threshold = int(os.getenv('ENERGY_THRESHOLD', '300'))
        else:
            self.recognizer = None

        # Configura TTS ElevenLabs se disponível
        self.use_eleven = False
        if ELEVEN_AVAILABLE:
            api_key = os.getenv('ELEVENLABS_API_KEY', '')
            if api_key:
                set_api_key(api_key)
                self.use_eleven = True
                self.eleven_voice = os.getenv('ELEVEN_VOICE', 'Rachel')
        self.use_fallback = (not self.use_eleven) and (TTS_FALLBACK == 'gtts')

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
        # Se não tem token ou expirou, renova
        if not self.token or now >= self.token_expiry:
            self._refresh_token()
        return self.token

    def _refresh_token(self):
        # Solicita novo token e guarda expiração
        secret = os.getenv('SECRET_KEY', '')
        resp = requests.get(
            f"{self.base}/get_access_token",
            headers={'X-token': secret},
            timeout=5.0
        )
        resp.raise_for_status()
        data = resp.json()
        self.token = data.get('access_token', '')
        expires_in = data.get('expires_in', 300)
        self.token_expiry = time.time() + expires_in - 5  # um buffer de 5s
        # Recria o client com novo header
        self.ai = RemoteRunnable(
            f"{self.base}/chat",
            headers={'temp-token': self.token}
        )

    def listen_and_respond(self):
        mic_index = int(os.getenv('MIC_DEVICE_INDEX', '-1'))
        mic_args = {'device_index': mic_index} if mic_index >= 0 else {}
        with sr.Microphone(**mic_args) as mic:
            self.recognizer.adjust_for_ambient_noise(mic, duration=0.5)
            self.get_logger().info('Ouvindo… fale algo')
            audio = self.recognizer.listen(mic, phrase_time_limit=5)
        try:
            stt_lang = os.getenv('STT_LANGUAGE', 'pt-BR')
            query = self.recognizer.recognize_google(audio, language=stt_lang)
        except Exception as e:
            self.get_logger().warn(f'STT falhou: {e}')
            return
        self._call_ai(query)

    def _call_ai(self, query: str):
    

        # 1) Log da pergunta
        self.get_logger().info(f'Pergunta: {query}')

        # 2) Usa o token atual para chamar o chat
        token_chat = self._get_valid_token()
        try:
            result = self.ai.invoke({"message": query, "chat_history": []})
        except Exception as e:
            self.get_logger().error(f'Erro ao chamar IA: {e}')
            return

        # 3) Extrai o texto da resposta
        if isinstance(result, dict):
            resposta = result.get('text', '')
        else:
            resposta = str(result)

        # 4) (Opcional) detectar idioma
        lang_code = None
        if detect:
            try:
                lang_code = detect(resposta)
            except:
                lang_code = None

        # 5) Obtém um NOVO token para o TTS
        #    Isso faz um GET /get_access_token e popula self.token
        try:
            self._refresh_token()
        except Exception as e:
            self.get_logger().error(f'Erro ao renovar token para TTS: {e}')
            print(f'>> IA sem áudio: {resposta}')
            return
        token_tts = self.token

        # 6) Chama o endpoint /tts/ com o token fresco
        try:
            resp = requests.post(
                f"{self.base}/tts/",
                json={"text": resposta},
                headers={"temp-token": token_tts},
                timeout=10.0
            )
            resp.raise_for_status()
        except Exception as e:
            self.get_logger().error(f'Erro ao chamar TTS: {e}')
            print(f'>> IA sem áudio: {resposta}')
            return

        # 7) Grava e toca o MP3
        with NamedTemporaryFile(delete=False, suffix=".mp3") as tmp:
            tmp.write(resp.content)
            tmp.flush()
        try:
            playsound.playsound(tmp.name)
        except Exception as e:
            self.get_logger().error(f'Erro ao reproduzir áudio: {e}')
        finally:
            os.remove(tmp.name)

        # 8) Também imprime no console
        print(f"\n>> IA: {resposta}\n")



    def _fallback_tts(self, text: str, lang_code: str):
        try:
            tts = gTTS(text=text, lang=lang_code or 'pt')
            tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.mp3')
            tts.save(tmp.name)
            playsound.playsound(tmp.name)
            os.remove(tmp.name)
        except Exception as e:
            self.get_logger().error(f'Erro gTTS fallback: {e}')

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
