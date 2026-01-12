
import re
import logging

# ===== Configuration copied/adapted from stt.py =====
# These could be passed in or shared from a common config, 
# but for separation, we keep local constants or pass them in init.
SERVO_MIN = 0
SERVO_MAX = 180
DEFAULT_ANGLE_CENTER = 90
DEFAULT_STEP = 20
UNSURE_POLICY = "NOOP"

log = logging.getLogger("robot_mode")

def clamp(v, lo, hi): 
    return max(lo, min(hi, v))

class RobotMode:
    def __init__(self, actions_config):
        self.actions_config = actions_config

    def process_text(self, text: str, current_angle: int):
        """
        Returns: (action_dict, meaningful_bool, new_angle)
        """
        t = (text or "").strip()
        if not t:
            return ({"action": "NOOP"} if UNSURE_POLICY == "NOOP" else {"action": "WIGGLE"}, False, current_angle)

        # YAML definition based matching
        for cmd in self.actions_config:
            matched = False
            captured_val = None

            # 1. Keyword match
            if "keywords" in cmd:
                for k in cmd["keywords"]:
                    if k in t:
                        matched = True
                        break
            
            # 2. Pattern match
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
                
                # Logic calculation
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
                    # STOP, ROTATE, etc.
                    return ({"action": a_type, "servo": servo_idx}, True, current_angle)

        return ({"action": "NOOP"} if UNSURE_POLICY == "NOOP" else {"action": "WIGGLE"}, False, current_angle)
