#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import os
import time
import requests
from langserve import RemoteRunnable
from tempfile import NamedTemporaryFile
from std_msgs.msg import String

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
        
        # 1o tenta usar o parâmetro ROS(YAML), se não, variável de ambiente, se não erro (vazio)
        self.declare_parameter(
            "SECRET_KEY", os.getenv('SECRET_KEY','')
        )

        # lê a chave secreta 
        self.secret_key = self.get_parameter("SECRET_KEY").get_parameter_value().string_value


        if not self.secret_key:
            self.get_logger().error(
                "SECRET_KEY não definido nem como parâmetro ROS nem como variável de ambiente."
            )
            raise RuntimeError("SECRET_KEY ausente")

        # Token e expiração
        self.token = None
        self.token_expiry = 0

        self.session  = requests.Session()

        self.is_speaking = False
        self.ai_busy = False
        self.stop_conversation = False

        #variavelpara guardar audio em reproducao
        self.current_play_obj = None

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

        
        
        #subscriber para ativar ia via tópico
        self.trigger_sub = self.create_subscription(
            String,
            'ai_trigger',
            self.trigger_callback,
            10
        )

        # Cria um publisher para o tópico /ai_status
        self.ai_status_pub = self.create_publisher(String, 'ai_status', 10)

        # Instancia o client da API com token inicial
        self._refresh_token()

        self._publish_status("idle")
        

    def trigger_callback(self, msg: String):
        """
        Callback chamado quando alguém publica em /ai_trigger.
        """
        comando = msg.data.strip().lower()
        print(f"[AI] >>> Trigger recebido no tópico /ai_trigger com data='{comando}'")

        # 1) STOP tem prioridade absoluta: pode ser chamado a qualquer momento
        if comando == "stop":
            self.get_logger().info("[AI] Comando STOP recebido. Encerrando conversa imediatamente.")
            print("[AI] >>> STOP recebido. Marcando stop_conversation = True.")
            self.stop_conversation = True

            # se estiver tocando áudio, para na hora
            if self.current_play_obj is not None:
                try:
                    self.current_play_obj.stop()
                    print("[AI] >>> Áudio TTS interrompido via STOP.")
                except Exception as e:
                    self.get_logger().warn(f"Falha ao tentar parar áudio: {e}")

            # força status visual para idle (OlhosNode volta a mostrar 'Conversar')
            self._publish_status("idle")
            return

        # 2) Daqui pra baixo são comandos de INÍCIO de conversa

        # Se estiver em modo de teste, não roda automático
        if self.test_mode:
            self.get_logger().info("[AI] Trigger recebido, mas estou em modo TESTE (teclado).")
            return

        # Se já estiver ocupado ou falando, ignora (para START, FALAR, etc.)
        if self.ai_busy or self.is_speaking:
            print("[AI] >>> IA ocupada (ai_busy ou is_speaking=True). Ignorando trigger.")
            return

        if comando not in ("start", "fala", "falar"):
            print(f"[AI] >>> Comando '{msg.data}' não reconhecido como trigger. Ignorando.")
            return

        print("[AI] >>> Trigger VÁLIDO! Vou iniciar conversa_loop() em uma thread.")

        def worker():
            try:
                self.ai_busy = True
                print("[AI] >>> (worker) Iniciando conversa_loop()...")
                self.conversa_loop(max_turns=3)
                print("[AI] >>> (worker) conversa_loop() terminou.")
            finally:
                self.ai_busy = False
                self._publish_status("idle")

        threading.Thread(target=worker, daemon=True).start()


    def _publish_status(self, status: str):
        """
        Publica o estado da IA em /ai_status.
        Possíveis valores: 'idle', 'listening', 'thinking', 'speaking'
        """
        try:
            msg = String()
            msg.data = status
            self.ai_status_pub.publish(msg)
            print(f"[AI-STATUS] {status}")
        except Exception as e:
            self.get_logger().warn(f"Falha ao publicar status da IA: {e}")


    def _get_valid_token(self):
        now = time.time()
        if not self.token or now >= self.token_expiry:
            self._refresh_token()
        return self.token

    def _refresh_token(self):
        resp = self.session.get(
            f"{self.base}/get_access_token",
            headers={'X-token': self.secret_key},
            timeout=5.0
        )
        resp.raise_for_status()
        data = resp.json()
        self.token = data.get('access_token', '')
        expires_in = data.get('expires_in', 300)
        self.token_expiry = time.time() + expires_in - 5
        

    def listen_and_respond(self) -> bool:
        """
        Executa UM turno de interação:
        - Ouve o usuário
        - Faz STT
        - Chama a IA
        Retorna:
            True  -> sessão pode continuar (ex.: ouviu algo e respondeu)
            False -> sessão deve encerrar (ex.: erro, silêncio, etc.)
        """
        self._publish_status("listening")
        print("[AI] >>> Entrou em listen_and_respond() — começando a ouvir o microfone...")

        # Se já estiver falando, não faz sentido ouvir de novo
        if self.is_speaking:
            self.get_logger().info("Ainda estou falando, não vou ouvir agora.")
            return False
        
        mic_index = int(os.getenv('MIC_DEVICE_INDEX', '-1'))
        mic_args = {'device_index': mic_index} if mic_index >= 0 else {}
        with sr.Microphone(**mic_args) as mic:
            self.recognizer.adjust_for_ambient_noise(mic, duration=0.5)
            self.get_logger().info('Ouvindo… fale algo')
            try:
                audio = self.recognizer.listen(mic, phrase_time_limit=5)
            except Exception as e:
                self.get_logger().warn(f"Falha ao ouvir microfone: {e}")
                self._publish_status("idle")
                return False

        try:
            # Converte áudio para WAV bytes
            wav_data = audio.get_wav_data()

            # 1) Garante token fresco para STT (não reutilizar o já consumido)
            self._refresh_token()
            token_stt = self.token

            # Prepara arquivo para Whisper: bytes em BytesIO com atributo name
            audio_file = io.BytesIO(wav_data)
            audio_file.name = 'audio.wav'

            # Monta payload multipart
            files = {'payload': (audio_file.name, audio_file, 'audio/wav')}

            # 2) Chama o endpoint /stt/ com token recém obtido
            resp = self.session.post(
                f"{self.base}/stt/",
                headers={'temp-token': token_stt},
                files=files,
                timeout=10.0
            )
            resp.raise_for_status()
            query = resp.json().get('text', '').strip()
            self.get_logger().info(f"Você disse: {query}")

            if not query:
                self.get_logger().info("STT retornou vazio (silêncio ou ruído). Encerrando sessão.")
                self._publish_status("idle")
                return False

            # Terminou STT com sucesso -> pensando
            self._publish_status("thinking")

        except Exception as e:
            self.get_logger().warn(f'STT falhou: {e}')
            self._publish_status("idle")
            return False

        # Continua o fluxo normal (chama IA e TTS)
        self._call_ai(query)
        # Aqui NÃO voltamos para idle ainda: isso é feito pela thread de TTS
        return True

    
    def conversa_loop(self, max_turns: int = 3):
        self.stop_conversation = False
        """
        Loop de conversa com vários turnos.
        - Chama listen_and_respond() até max_turns
        - Espera terminar de falar antes de ouvir de novo
        - Interrompe se listen_and_respond() retornar False
        """
        self.get_logger().info(f"[AI] Iniciando sessão de conversa com até {max_turns} turnos.")
        turns = 0

        while turns < max_turns:
            if self.stop_conversation:
                self.get_logger().info(f"[AI] Encerrando sessão de conversa (stop_conversation=True no turno {turns+1}).")
                
                break

            # 1 turno: ouvir -> pensar -> iniciar fala
            continuar = self.listen_and_respond()

            if not continuar:
                self.get_logger().info(f"[AI] Encerrando sessão de conversa (listen_and_respond retornou False no turno {turns+1}).")
                break

            turns += 1
            self.get_logger().info(f"[AI] Turno {turns} concluído, aguardando término da fala para novo turno...")

            # Espera terminar de falar antes de ouvir de novo
            while self.is_speaking:
                time.sleep(0.1)

        self.get_logger().info(f"[AI] Sessão de conversa finalizada após {turns} turnos.")

    def tocar_audio_em_thread(self, wav_bytes):
        try:
            self.is_speaking = True
            self._publish_status("speaking")

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

            # começa a tocar e guarda o objeto para poder parar de fora
            start_play = time.time()
            play_obj = wave_obj.play()
            self.current_play_obj = play_obj

            # em vez de play_obj.wait_done(), checamos periodicamente se recebeu STOP
            while play_obj.is_playing():
                if self.stop_conversation:
                    print("[TTS-thread] STOP detectado durante reprodução. Parando áudio.")
                    play_obj.stop()
                    break
                time.sleep(0.05)

            end_play = time.time()
            print(f"[TTS-thread] Reprodução do áudio (até parada/fim): {end_play - start_play:.2f} segundos")

            total = end_play - start_decode
            print(f"[TTS-thread] Tempo TOTAL na thread: {total:.2f} segundos")
        finally:
            self.current_play_obj = None
            self.is_speaking = False
            # deixa o status em idle; o worker da conversa também vai setar idle ao terminar
            self._publish_status("idle")



    
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
        start_tts_total = time.time()

        # Token novo só para o TTS
        self._refresh_token()
        token_tts = self.token

        try:
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
        except Exception as e:
            self.get_logger().error(f"Erro no TTS: {e}")
            self._publish_status("idle")
            return

        # Antes de tocar o áudio, marcamos que está FALANDO
        self._publish_status("speaking")

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
