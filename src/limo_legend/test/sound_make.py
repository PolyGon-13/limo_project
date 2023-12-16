from gtts import gTTS
import os

text = "right"
tts = gTTS(text=text, lang='en')
tts.save("right.mp3")
