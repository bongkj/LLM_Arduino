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

HOST = "0.0.0.0"
PORT = 5001
SR = 16000

# ===== 모델 설정 =====
MODEL_SIZE = "tiny"
PREFER_DEVICE = "cuda"   # GPU 원하면 "cuda", 아니면 "cpu"
CPU_FALLBACK_COMPUTE = "int8"

# ===== Protocol =====
PTYPE_START = 0x01
PTYPE_AUDIO = 0x02
PTYPE_END   = 0x03
PTYPE_PING  = 0x10  # ESP32 keepalive
PTYPE_CMD   = 0x11  # PC -> ESP32 JSON

UNSURE_POLICY = "NOOP"   # or "WIGGLE"

SERVO_MIN = 0
SERVO_MAX = 180
DEFAULT_ANGLE_CENTER = 90
DEFAULT_ANGLE_LEFT = 30
DEFAULT_ANGLE_RIGHT = 150
DEFAULT_STEP = 20

# ===== Logging =====
logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
log = logging.getLogger("stt")

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
        if len(payload) > 65535: payload = payload[:65535]
        header = struct.pack("<BH", ptype & 0xFF, len(payload))
        if lock:
            with lock:
                conn.sendall(header + payload)
        else:
            conn.sendall(header + payload)
        return True
    except Exception as e:
        log.warning(f"send_packet failed ptype=0x{ptype:02X}: {e}")
        return False

def send_action(conn: socket.socket, action_dict: dict, lock: threading.Lock):
    payload = json.dumps(action_dict, ensure_ascii=False).encode("utf-8")
    ok = send_packet(conn, PTYPE_CMD, payload, lock=lock)
    if ok:
        log.info(f"➡️ CMD to ESP32: {action_dict}")

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


# yaml 라이브러리 필요 (pip install pyyaml)
import yaml

ACTIONS_CONFIG = []

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

def parse_action_from_text(text: str, current_angle: int):
    t = (text or "").strip()
    if not t:
        return ({"action": "NOOP"} if UNSURE_POLICY == "NOOP" else {"action": "WIGGLE"}, False, current_angle)

    # YAML 설정 기반 매칭
    for cmd in ACTIONS_CONFIG:
        matched = False
        captured_val = None

        # 1. 키워드 매칭
        if "keywords" in cmd:
            for k in cmd["keywords"]:
                if k in t:
                    matched = True
                    break
        
        # 2. 패턴 매칭 (키워드가 없거나 매칭 안됐을 때)
        if not matched and "pattern" in cmd:
            m = re.search(cmd["pattern"], t)
            if m:
                matched = True
                if cmd.get("use_captured") and m.lastindex and m.lastindex >= 1:
                    try:
                        captured_val = int(m.group(1))
                    except:
                        pass
        
        if matched:
            a_type = cmd.get("action", "NOOP")
            servo_idx = cmd.get("servo", 0)
            
            # 값 계산
            if a_type == "SERVO_SET":
                angle = cmd.get("angle")
                if cmd.get("use_captured") and captured_val is not None:
                    angle = captured_val
                if angle is None: angle = DEFAULT_ANGLE_CENTER
                
                final_angle = clamp(angle, SERVO_MIN, SERVO_MAX)
                return ({"action": "SERVO_SET", "servo": servo_idx, "angle": final_angle}, True, final_angle)

            elif a_type == "SERVO_INC":
                step = cmd.get("value", DEFAULT_STEP)
                final_angle = clamp(current_angle + step, SERVO_MIN, SERVO_MAX)
                return ({"action": "SERVO_SET", "servo": servo_idx, "angle": final_angle}, True, final_angle)

            elif a_type == "SERVO_DEC":
                step = cmd.get("value", DEFAULT_STEP)
                final_angle = clamp(current_angle - step, SERVO_MIN, SERVO_MAX)
                return ({"action": "SERVO_SET", "servo": servo_idx, "angle": final_angle}, True, final_angle)

            else:
                # STOP, ROTATE 등 기타 액션
                return ({"action": a_type, "servo": servo_idx}, True, current_angle)

    return ({"action": "NOOP"} if UNSURE_POLICY == "NOOP" else {"action": "WIGGLE"}, False, current_angle)

# ===== 모델 로딩 (GPU 선호하되, 터지면 CPU로 폴백) =====
model_lock = threading.Lock()
model = None
device_in_use = None

def load_model(device: str):
    global model, device_in_use
    log.info(f"loading model: {MODEL_SIZE} on {device} ...")
    m = WhisperModel(MODEL_SIZE, device=device, compute_type=("int8" if device=="cpu" else "int8"),
                     cpu_threads=1, num_workers=1)
    model = m
    device_in_use = device
    log.info(f"model loaded on {device}")

def ensure_model():
    global model
    if model is None:
        try:
            load_model(PREFER_DEVICE)
        except Exception as e:
            log.warning(f"GPU init failed -> fallback CPU: {e}")
            load_model("cpu")

def safe_transcribe(pcm_f32: np.ndarray):
    """
    segments(list), info 반환.
    cublas 오류 등 GPU 런타임 오류면 CPU로 갈아타고 1회 재시도.
    """
    ensure_model()
    pcm_f32 = np.ascontiguousarray(pcm_f32, dtype=np.float32)

    def _run():
        # generator가 나중에 터지는 걸 잡기 위해 list로 강제 평가
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
                load_model("cpu")
                return _run()
            raise

def handle_connection(conn, addr):
    log.info(f"📡 connected: {addr}")
    conn.settimeout(0.5)  # 수신 스레드가 자주 깨어나도록
    try:
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except Exception:
        pass

    send_lock = threading.Lock()
    jobs = Queue(maxsize=4)

    state = {"sid": 0, "current_angle": DEFAULT_ANGLE_CENTER}
    state_lock = threading.Lock()

    def worker():
        while True:
            job = jobs.get()
            if job is None:
                return
            sid, data = job
            sec = len(data) / 2 / SR
            if sec < 0.45:
                action = {"action": "NOOP" if UNSURE_POLICY=="NOOP" else "WIGGLE",
                          "sid": sid, "meaningful": False, "recognized": False}
                send_action(conn, action, send_lock)
                continue

            pcm = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            rms_db, peak, clip = qc(pcm)
            log.info(f"QC sid={sid} rms={rms_db:.1f}dBFS peak={peak:.3f} clip={clip:.2f}%")

            if rms_db < -45.0:
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

            try:
                segments, info = safe_transcribe(pcm)
                text = clean_text("".join(seg.text for seg in segments))
            except Exception as e:
                log.exception(f"transcribe failed sid={sid}: {e}")
                action = {"action": "NOOP" if UNSURE_POLICY=="NOOP" else "WIGGLE",
                          "sid": sid, "meaningful": False, "recognized": False}
                send_action(conn, action, send_lock)
                continue

            if text:
                log.info(f"🗣️ {text}")
            else:
                log.info("🗣️ (empty/filtered)")

            with state_lock:
                cur = state["current_angle"]

            action, meaningful, new_angle = parse_action_from_text(text, cur)
            action["sid"] = sid
            action["meaningful"] = meaningful
            action["recognized"] = bool(text)

            with state_lock:
                state["current_angle"] = new_angle

            send_action(conn, action, send_lock)

    threading.Thread(target=worker, daemon=True).start()

    audio_buf = bytearray()

    while True:
        # ===== packet read =====
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
            # keepalive. 필요하면 pong도 가능:
            # send_packet(conn, PTYPE_PING, b"", lock=send_lock)
            continue

        if ptype == PTYPE_START:
            with state_lock:
                state["sid"] += 1
                sid = state["sid"]
            audio_buf = bytearray()
            log.info(f"🎙️ START (sid={sid})")

        elif ptype == PTYPE_AUDIO:
            audio_buf.extend(payload)
            # 너무 길면 안전 컷
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
    load_commands_config()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)

    while True:
        log.info("ready for next connection...")
        conn, addr = srv.accept()
        try:
            handle_connection(conn, addr)
        except Exception as e:
            log.exception(f"connection error: {e}")
        finally:
            try: conn.close()
            except Exception: pass

if __name__ == "__main__":
    main()
