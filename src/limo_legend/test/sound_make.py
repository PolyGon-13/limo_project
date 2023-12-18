from gtts import gTTS
import os

text = "stop"
tts = gTTS(text=text, lang='en')
tts.save("stop.wav")
print("end")
