import openai
import os
from dotenv import load_dotenv

load_dotenv()
openai.api_type = "azure"
openai.api_version = "2024-02-15-preview"

try:
    models = openai.models.list()
    print("✅ API Key 有效！")
    for model in models.data[:5]:
        print(model.id)
except openai.OpenAIError as e:
    print("❌ API Key invalid or else: ", e)


WHISPER_DEPLOYMENT_NAME = "whisper"

file_path = "test.wav"

with open(file_path, "rb") as audio_file:
    transcript = openai.audio.transcriptions.create(
        file=audio_file,
        model=WHISPER_DEPLOYMENT_NAME,  # model= Whisper deployment name on Azure
        response_format="text"
    )

print("transcript:")
print(transcript)