from gtts import gTTS
import os

text = "sound"
tts = gTTS(text=text, lang='en')
tts.save("fff.mp3")
print("end")
