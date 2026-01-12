
import logging
import io
import asyncio
import numpy as np

# We used pip install edge-tts transformers torch ...
# But since we run this synchronously from stt.py, we might need to handle async for edge-tts properly.
# For now, we wrap it.

log = logging.getLogger("agent_mode")

class AgentMode:
    def __init__(self, device="cuda"):
        self.device = device
        self.model = None
        self.tokenizer = None
        self.tts_voice = "ko-KR-SunHiNeural" # Example excellent Korean voice

    def load_model(self):
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer
            import torch

            log.info(f"Loading Qwen2.5-0.5B-Instruct on {self.device}...")
            model_name = "Qwen/Qwen2.5-0.5B-Instruct"
            
            # Load tokenizer
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            
            # Load model
            # Assuming FP16 for GPU or Float32 for CPU to save memory/speed if possible
            torch_dtype = torch.float16 if self.device == "cuda" else torch.float32
            
            self.model = AutoModelForCausalLM.from_pretrained(
                model_name,
                torch_dtype=torch_dtype,
                device_map=self.device,
                trust_remote_code=True
            )
            log.info("Qwen Model loaded.")
            
        except ImportError:
            log.error("Transformers/Torch not installed. Please install: pip install transformers torch accelerate")
        except Exception as e:
            log.error(f"Failed to load LLM: {e}")

    def generate_response(self, text: str):
        if not self.model or not self.tokenizer:
            return "모델이 로드되지 않았습니다."

        try:
            # Simple prompt template for Chat
            messages = [
                {"role": "system", "content": "당신은 가정용 AI 비서입니다. 친절하고 간결하게 답변하세요. 한 문장 정도로 짧게 대답하는 것을 선호합니다."},
                {"role": "user", "content": text}
            ]
            text_input = self.tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True
            )
            model_inputs = self.tokenizer([text_input], return_tensors="pt").to(self.device)

            generated_ids = self.model.generate(
                model_inputs.input_ids,
                max_new_tokens=128,
                do_sample=True,
                temperature=0.7
            )
            
            generated_ids = [
                output_ids[len(input_ids):] for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
            ]

            response = self.tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
            log.info(f"LLM Response: {response}")
            return response
        except Exception as e:
            log.error(f"LLM generation failed: {e}")
            return "죄송해요, 오류가 발생했어요."

    async def _tts_gen(self, text, output_file):
        import edge_tts
        communicate = edge_tts.Communicate(text, self.tts_voice)
        await communicate.save(output_file)

    def text_to_audio(self, text: str):
        """
        Generates TTS audio and returns raw bytes (16kHz Mono PCM preferable, or WAV).
        M5 Atom Echo typically handles raw PCM or simple WAV well if we stream it.
        We will return standard bytes.
        """
        try:
            import soundfile as sf
            import librosa
            
            # edge-tts saves as mp3 usually. We need to convert to PCM 16k mono.
            tmp_mp3 = "temp_tts.mp3"
            
            # internal async loop execution
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                
            loop.run_until_complete(self._tts_gen(text, tmp_mp3))
            
            # Convert Web Audio/MP3 to 16000Hz Mono PCM-16
            data, samplerate = librosa.load(tmp_mp3, sr=16000, mono=True)
            # Clip and convert to int16
            data = np.clip(data, -1.0, 1.0)
            pcm_16 = (data * 32767).astype(np.int16)
            
            return pcm_16.tobytes()
            
        except ImportError:
            log.error("Install edge-tts, librosa, soundfile: pip install edge-tts librosa soundfile")
            return b""
        except Exception as e:
            log.error(f"TTS failed: {e}")
            return b""
