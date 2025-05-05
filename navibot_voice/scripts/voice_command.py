#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sounddevice as sd
import scipy.io.wavfile as wavfile
import openai
import os, re
import dotenv

dotenv.load_dotenv()
openai.api_type = "azure"

# 音频参数
SAMPLE_RATE = 16000  # Hz
DURATION = 5         # seconds
TEMP_WAV_PATH = '/tmp/voice_command.wav'
WHISPER_DEPLOYMENT_NAME = "whisper"

# check if the environment variable is set
if 'AZURE_OPENAI_API_KEY' not in os.environ:
    raise ValueError("Environment variable AZURE_OPENAI_API_KEY is not set.")
if 'AZURE_OPENAI_ENDPOINT' not in os.environ:
    raise ValueError("Environment variable AZURE_OPENAI_API_BASE is not set.")

# set default device (optional)
# sd.default.device = os.getenv("AUDIO_DEVICE_ID", None)

def record_audio(path: str):
    rospy.loginfo(f"start recording for {DURATION}s...")
    audio = sd.rec(int(DURATION * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1)
    sd.wait()
    wavfile.write(path, SAMPLE_RATE, audio)
    rospy.loginfo(f"wav saved to: {path}")

def transcribe(path: str) -> str:
    with open(path, 'rb') as f:
        resp = openai.audio.transcriptions.create(
            file=f,
            model=WHISPER_DEPLOYMENT_NAME,
            response_format="text"
        )
    if isinstance(resp, dict):
        text = resp["text"]
    else:
        text = resp
    rospy.loginfo(f"transcription: {text}")
    return text

def parse_destination(text):
    """
    - take me to the ...
    - go to ...
    - navigate to ...
    - bring me to ...
    - head to ...
    """
    pattern = r"(?:go to|take me to|navigate to|bring me to|head to)\s+(?:the\s)?(?P<dest>[a-zA-Z\s]+)"
    match = re.search(pattern, text, re.IGNORECASE)
    if match:
        return match.group("dest").strip().lower()
    return None

def main():
    rospy.init_node('voice_control')
    pub = rospy.Publisher('/voice_command', String, queue_size=10)
    rate = rospy.Rate(1.0 / DURATION)

    while not rospy.is_shutdown():
        try:
            record_audio(TEMP_WAV_PATH)
            cmd = transcribe(TEMP_WAV_PATH)
            dst = parse_destination(cmd)
            if dst:
                rospy.loginfo(f"destination: {dst}")
                msg = String()
                msg.data = dst
                pub.publish(cmd)
                break
        except Exception as e:
            rospy.logerr(f"voice_cmd error: {e}")
        rate.sleep()

if __name__ == '__main__':
    main()