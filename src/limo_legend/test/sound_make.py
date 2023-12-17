from gtts import gTTS
import os

text = "준식이 바보"
tts = gTTS(text=text, lang='ko')
tts.save("fff.mp3")
print("end")
