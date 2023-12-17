from gtts import gTTS
import os

text = "공습경보"
tts = gTTS(text=text, lang='ko')
tts.save("fff.mp3")
print("end")
