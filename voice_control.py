"""Reconnaissance vocale FR"""
import speech_recognition as sr
CMDS={"attrape":"attrape","prends":"attrape","lache":"lache","monte":"monte","descends":"descend","stop":"stop","arrete":"stop","home":"home","go":"go"}
class VoiceController:
    def __init__(self,callback=None,lang="fr-FR"):self.rec=sr.Recognizer();self.mic=sr.Microphone();self.cb=callback;self.lang=lang
    def listen(self):
        with self.mic as src:
            self.rec.adjust_for_ambient_noise(src,duration=0.5)
            try:
                audio=self.rec.listen(src,timeout=5)
                text=self.rec.recognize_google(audio,language=self.lang).lower()
                cmd=next((v for k,v in CMDS.items() if k in text),text)
                if self.cb:self.cb(cmd)
            except:pass
