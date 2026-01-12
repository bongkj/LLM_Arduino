# TODO: CUDA 11.8 -> 12.x 업그레이드 필요 (GPU 동작 위함)
# TODO: tiny 모델 대신 다른 무거운 모델 사용 가능

import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")

import socket, struct, time, wave, re, logging, json, threading
from queue import Queue, Full, Empty
import numpy as np
from faster_whisper import WhisperModel
import yaml

# New Modules
from robot_mode import RobotMode
from agent_mode import AgentMode

HOST = "0.0.0.0"
PORT = 5001
SR = 16000

# ===== 모델 설정 =====
MODEL_SIZE = "tiny"
# PREFER_DEVICE is used for Whisper (faster-whisper)
PREFER_DEVICE = "cuda"   # GPU 원하면 "cuda", 아니면 "cpu"
CPU_FALLBACK_COMPUTE = "int8"

# ===== Protocol =====
PTYPE_START     = 0x01
PTYPE_AUDIO     = 0x02
PTYPE_END       = 0x03
PTYPE_PING      = 0x10  # ESP32 keepalive
PTYPE_CMD       = 0x11  # PC -> ESP32 JSON
PTYPE_AUDIO_OUT = 0x12  # PC -> ESP32 Audio (PCM)

UNSURE_POLICY = "NOOP"   # or "WIGGLE"

# ===== Logging =====
logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
log = logging.getLogger("stt")

# ===== Global State =====
ACTIONS_CONFIG = []
current_mode = "agent" # default mode

# Helper Instances
robot_handler = None
agent_handler = None

def clamp(v, lo, hi): return max(lo, min(hi, v))

def recv_exact(conn, n: int):
    buf = b""
    while len(buf) < n:
        try:
            chunk = conn.recv(n - len(buf))
        except socket.timeout:
            continue
        if not chunk:
            return None
        buf += chunk
    return buf

def send_packet(conn, ptype: int, payload: bytes = b"", lock=None) -> bool:
    try:
        if payload is None: payload = b""
        if len(payload) > 65535: # Standard max for 2byte len, actually protocol specific
             # Simple chunking logic might be needed if really large, but for now cap it or assume logic handles it.
             # Actually protocol says 1B type + 2B len. Max len is 65535.
             # If audio is larger, we should split it.
             pass 

        if lock:
            with lock:
                # If payload > 65k, we might need a loop, but let's assume caller chunks or we chunk here.
                # Just simplified chunking for safety:
                offset = 0
                total = len(payload)
                if total == 0:
                     conn.sendall(struct.pack("<BH", ptype & 0xFF, 0))
                     return True
                
                while offset < total:
                    chunk_size = min(total - offset, 60000) # safe margin under 65535
                    chunk = payload[offset:offset+chunk_size]
                    header = struct.pack("<BH", ptype & 0xFF, len(chunk))
                    conn.sendall(header + chunk)
                    offset += chunk_size
        else:
            # Same logic without lock
            offset = 0
            total = len(payload)
            if total == 0:
                    conn.sendall(struct.pack("<BH", ptype & 0xFF, 0))
                    return True
            while offset < total:
                chunk_size = min(total - offset, 60000)
                chunk = payload[offset:offset+chunk_size]
                header = struct.pack("<BH", ptype & 0xFF, len(chunk))
                conn.sendall(header + chunk)
                offset += chunk_size

        return True
    except Exception as e:
        log.warning(f"send_packet failed ptype=0x{ptype:02X}: {e}")
        return False

def send_action(conn: socket.socket, action_dict: dict, lock: threading.Lock):
    payload = json.dumps(action_dict, ensure_ascii=False).encode("utf-8")
    ok = send_packet(conn, PTYPE_CMD, payload, lock=lock)
    if ok:
        log.info(f"➡️ CMD to ESP32: {action_dict}")

def send_audio(conn: socket.socket, pcm_bytes: bytes, lock: threading.Lock):
    # Sends audio commands. 
    # Packet type PTYPE_AUDIO_OUT will be handled by Arduino to play sound.
    ok = send_packet(conn, PTYPE_AUDIO_OUT, pcm_bytes, lock=lock)
    if ok:
        log.info(f"➡️ AUDIO to ESP32: {len(pcm_bytes)} bytes")

def save_wav(path: str, pcm_f32: np.ndarray, sr: int = SR):
    x = np.clip(pcm_f32, -1.0, 1.0)
    x16 = (x * 32767.0).astype(np.int16)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(x16.tobytes())

def qc(pcm_f32: np.ndarray):
    peak = float(np.max(np.abs(pcm_f32))) if pcm_f32.size else 0.0
    rms = float(np.sqrt(np.mean(pcm_f32**2) + 1e-12)) if pcm_f32.size else 0.0
    rms_db = 20.0 * np.log10(rms + 1e-12)
    clip = float(np.mean(np.abs(pcm_f32) >= 0.999)) * 100.0
    return rms_db, peak, clip

def trim_energy(pcm: np.ndarray, sr: int, top_db: float = 35.0, pad_ms: int = 140):
    if pcm.size == 0: return pcm.astype(np.float32, copy=False)
    frame = int(sr * 0.02)  # 20ms
    hop = frame
    n = pcm.size
    if n < frame: return pcm.astype(np.float32, copy=False)

    rms = []
    for i in range(0, n - frame + 1, hop):
        x = pcm[i:i+frame]
        rms.append(np.sqrt(np.mean(x*x) + 1e-12))
    rms = np.array(rms, dtype=np.float32)
    thr = float(np.max(rms)) * (10 ** (-top_db / 20.0))

    idx = np.where(rms > thr)[0]
    if idx.size == 0: return pcm.astype(np.float32, copy=False)

    start_f = int(idx[0]); end_f = int(idx[-1])
    start = max(0, start_f * hop - int(sr * pad_ms / 1000))
    end   = min(n, (end_f * hop + frame) + int(sr * pad_ms / 1000))
    return pcm[start:end].astype(np.float32, copy=False)

def normalize_to_dbfs(pcm: np.ndarray, target_dbfs: float = -22.0, max_gain_db: float = 18.0):
    rms = float(np.sqrt(np.mean(pcm*pcm) + 1e-12))
    rms_db = 20.0 * np.log10(rms + 1e-12)
    gain_db = np.clip(target_dbfs - rms_db, -6.0, max_gain_db)
    gain = 10 ** (gain_db / 20.0)
    return np.clip(pcm * gain, -1.0, 1.0).astype(np.float32, copy=False)

def clean_text(text: str):
    t = (text or "").strip()
    t = re.sub(r"[,，]{3,}", ",", t)
    t = re.sub(r"([,，。.!?])\1{1,}", r"\1", t)
    t = re.sub(r"\s+", " ", t).strip()
    if len(t) >= 8:
        punct = sum(ch in ",，。.!?…" for ch in t)
        if punct / max(1, len(t)) > 0.35:
            return ""
    t = re.sub(r"[,，]+$", "", t).strip()
    return t

def load_commands_config():
    global ACTIONS_CONFIG
    try:
        with open("commands.yaml", "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            ACTIONS_CONFIG = data.get("commands", [])
            log.info(f"Loaded {len(ACTIONS_CONFIG)} commands from commands.yaml")
    except Exception as e:
        log.error(f"Failed to load commands.yaml: {e}")
        ACTIONS_CONFIG = []

# ===== Whisper Model =====
model_lock = threading.Lock()
model = None
device_in_use = None

def load_stt_model(device: str):
    global model, device_in_use
    log.info(f"loading STT model: {MODEL_SIZE} on {device} ...")
    m = WhisperModel(MODEL_SIZE, device=device, compute_type=("int8" if device=="cpu" else "int8"),
                     cpu_threads=1, num_workers=1)
    model = m
    device_in_use = device
    log.info(f"STT model loaded on {device}")

def ensure_stt_model():
    global model
    if model is None:
        try:
            load_stt_model(PREFER_DEVICE)
        except Exception as e:
            log.warning(f"GPU init failed -> fallback CPU: {e}")
            load_stt_model("cpu")

def safe_transcribe(pcm_f32: np.ndarray):
    ensure_stt_model()
    pcm_f32 = np.ascontiguousarray(pcm_f32, dtype=np.float32)

    def _run():
        segments, info = model.transcribe(
            pcm_f32,
            language="ko",
            beam_size=5,
            temperature=0.0,
            condition_on_previous_text=False,
            repetition_penalty=1.15,
            no_repeat_ngram_size=3,
            log_prob_threshold=-1.2,
            no_speech_threshold=0.6,
            compression_ratio_threshold=2.4,
            vad_filter=True,
            vad_parameters=dict(
                threshold=0.5,
                min_speech_duration_ms=200,
                min_silence_duration_ms=150,
                speech_pad_ms=120,
            ),
            suppress_blank=True,
        )
        segments = list(segments)
        return segments, info

    with model_lock:
        try:
            return _run()
        except RuntimeError as e:
            msg = str(e)
            if "cublas64_12.dll" in msg or "cublas" in msg:
                log.error("CUDA runtime missing/broken -> switching to CPU now.")
                load_stt_model("cpu")
                return _run()
            raise

def handle_connection(conn, addr):
    global current_mode, robot_handler, agent_handler

    log.info(f"📡 connected: {addr}")
    conn.settimeout(0.5)
    try:
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except Exception:
        pass

    send_lock = threading.Lock()
    jobs = Queue(maxsize=4)

    state = {"sid": 0, "current_angle": DEFAULT_ANGLE_CENTER}
    state_lock = threading.Lock()

    def worker():
        global current_mode
        while True:
            job = jobs.get()
            if job is None:
                return
            sid, data = job
            sec = len(data) / 2 / SR
            
            # Short audio filtering
            if sec < 0.45:
                # If short, maybe just ignore or send NOOP
                if current_mode == "robot":
                    action = {"action": "NOOP" if UNSURE_POLICY=="NOOP" else "WIGGLE",
                              "sid": sid, "meaningful": False, "recognized": False}
                    send_action(conn, action, send_lock)
                continue

            pcm = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            rms_db, peak, clip = qc(pcm)
            log.info(f"QC sid={sid} rms={rms_db:.1f}dBFS peak={peak:.3f} clip={clip:.2f}%")

            if rms_db < -45.0:
                 if current_mode == "robot":
                    action = {"action": "NOOP" if UNSURE_POLICY=="NOOP" else "WIGGLE",
                              "sid": sid, "meaningful": False, "recognized": False}
                    send_action(conn, action, send_lock)
                 continue

            pcm = trim_energy(pcm, SR)
            pcm = normalize_to_dbfs(pcm, target_dbfs=-22.0)

            ts = time.strftime("%Y%m%d_%H%M%S")
            wav_path = f"wav_logs/sid{sid}_{ts}_{len(pcm)/SR:.2f}s.wav"
            save_wav(wav_path, pcm, SR)
            log.info(f"saved wav: {wav_path}")

            text = ""
            try:
                segments, info = safe_transcribe(pcm)
                text = clean_text("".join(seg.text for seg in segments))
            except Exception as e:
                log.exception(f"transcribe failed sid={sid}: {e}")
                continue

            if text:
                log.info(f"🗣️ {text} (Mode: {current_mode})")
            else:
                log.info("🗣️ (empty/filtered)")

            with state_lock:
                cur = state["current_angle"]

            # 1. Check for Mode Switch first
            # We use robot_handler's parser to check because it holds the YAML config
            # We need to quickly check if 'SWITCH_MODE' is triggered.
            # Using robot_handler for this is fine as it's just regex/keyword matching.
            sys_action, meaningful, _ = robot_handler.process_text(text, cur)
            
            if meaningful and sys_action.get("action") == "SWITCH_MODE":
                new_mode = sys_action.get("mode")
                if new_mode in ["robot", "agent"]:
                    current_mode = new_mode
                    log.info(f"🔄 Mode Switched to: {current_mode}")
                    
                    # Notify user
                    notify_text = f"{new_mode} 모드로 변경합니다."
                    if current_mode == "agent":
                         # Agent mode -> Speak confirmation
                         wav_bytes = agent_handler.text_to_audio(notify_text)
                         if wav_bytes:
                             send_audio(conn, wav_bytes, send_lock)
                    else:
                        # Robot mode -> Maybe Wiggle or just accept
                        # But we can also speak if we want "로봇 모드입니다"
                        # For simple robot, we just act.
                        # Using TTS for confirmation is better?
                        # Let's try sending TTS confirmation even in Robot mode if possible,
                        # but Robot probably expects Actions.
                        # We will just print log for now or maybe Wiggle.
                         action = {"action": "WIGGLE", "sid": sid}
                         send_action(conn, action, send_lock)
                continue

            # 2. Per Mode logic
            if current_mode == "robot":
                # Re-run process_text to get actual robot command if it wasn't a switch
                # (Actually we already ran it above)
                action = sys_action 
                # If using sys_action from above, it's correct because robot_handler handles all commands
                
                action["sid"] = sid
                action["meaningful"] = meaningful
                action["recognized"] = bool(text)

                if meaningful and "angle" in action:
                     with state_lock:
                        state["current_angle"] = action["angle"]

                send_action(conn, action, send_lock)

            elif current_mode == "agent":
                if not text: continue
                # LLM Generation
                response = agent_handler.generate_response(text)
                if response:
                    # TTS
                    wav_bytes = agent_handler.text_to_audio(response)
                    if wav_bytes:
                        send_audio(conn, wav_bytes, send_lock)
                    else:
                        log.error("TTS returned empty bytes")


    threading.Thread(target=worker, daemon=True).start()

    audio_buf = bytearray()

    while True:
        # ===== packet read =====
        # Packet handling is same
        t = recv_exact(conn, 1)
        if t is None:
            log.info("disconnect")
            break
        ptype = t[0]

        raw_len = recv_exact(conn, 2)
        if raw_len is None:
            log.info("disconnect(len)")
            break
        (plen,) = struct.unpack("<H", raw_len)

        payload = b""
        if plen:
            payload = recv_exact(conn, plen)
            if payload is None:
                log.info("disconnect(payload)")
                break

        # ===== handle =====
        if ptype == PTYPE_PING:
            continue

        if ptype == PTYPE_START:
            with state_lock:
                state["sid"] += 1
                sid = state["sid"]
            audio_buf = bytearray()
            log.info(f"🎙️ START (sid={sid})")

        elif ptype == PTYPE_AUDIO:
            audio_buf.extend(payload)
            if len(audio_buf) > int(12 * SR * 2):
                log.warning("buffer too large -> force END")
                ptype = PTYPE_END

        if ptype == PTYPE_END:
            with state_lock:
                sid = state["sid"]
            data = bytes(audio_buf)
            sec = len(data) / 2 / SR
            log.info(f"🛑 END (sid={sid}) bytes={len(data)} sec={sec:.2f}")

            try:
                jobs.put_nowait((sid, data))
            except Full:
                log.warning("job queue full -> drop utterance")
            audio_buf = bytearray()

    try:
        jobs.put_nowait(None)
    except Exception:
        pass

def main():
    global robot_handler, agent_handler
    
    # Load Commands
    load_commands_config()
    
    # Init Handlers
    robot_handler = RobotMode(ACTIONS_CONFIG)
    agent_handler = AgentMode(PREFER_DEVICE)
    
    # Pre-load models? WHisper loads on first call or ensure_stt
    # Agent LLM load:
    agent_handler.load_model()
    
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)

    log.info(f"Server started on {PORT}. Default Mode: {current_mode}")

    while True:
        log.info("ready for next connection...")
        try:
            conn, addr = srv.accept()
        except KeyboardInterrupt:
            break
        except Exception as e:
            log.error(f"accept failed: {e}")
            time.sleep(1)
            continue
            
        try:
            handle_connection(conn, addr)
        except Exception as e:
            log.exception(f"connection handler error: {e}")
        finally:
            try: conn.close()
            except Exception: pass
            log.info(f"🔌 disconnected: {addr}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()
