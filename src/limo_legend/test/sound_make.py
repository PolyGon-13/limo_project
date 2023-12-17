from gtts import gTTS
import os

text = "보노보노 만세"
tts = gTTS(text=text, lang='en')
tts.save("fff.mp3")
print("end")
