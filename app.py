"""IHM Robot Assistance PMR — Hind Jabrane"""
import tkinter as tk, threading
from bluetooth_serial import BluetoothSerial
from voice_control import VoiceController

class RobotApp:
    def __init__(self):
        self.root=tk.Tk();self.root.title("Robot PMR");self.root.geometry("780x520");self.root.configure(bg="#f5f5fa")
        self.bt=BluetoothSerial();self.voice=VoiceController(callback=self.on_voice);self.build_ui()
    def build_ui(self):
        tk.Label(self.root,text="Robot Assistance PMR",font=("Helvetica",18,"bold"),bg="#2563eb",fg="white",pady=12).pack(fill="x")
        main=tk.Frame(self.root,bg="#f5f5fa");main.pack(fill="both",expand=True,padx=20,pady=15)
        left=tk.LabelFrame(main,text="Bras",font=("Helvetica",11,"bold"),bg="#f5f5fa",padx=10,pady=10)
        left.grid(row=0,column=0,sticky="nsew",padx=(0,10))
        for txt,cmd,col in [("Monte","monte","#2563eb"),("Descend","descend","#2563eb"),("Attrape","attrape","#06d6a0"),("Lache","lache","#06d6a0"),("Home","home","#8b5cf6")]:
            tk.Button(left,text=txt,font=("Helvetica",11),bg=col,fg="white",width=16,height=2,command=lambda c=cmd:self.send(c)).pack(pady=3)
        tk.Button(left,text="ARRET URGENCE",font=("Helvetica",13,"bold"),bg="#ff4444",fg="white",width=16,height=2,command=lambda:self.send("stop")).pack(pady=8)
        right=tk.LabelFrame(main,text="Journal",font=("Helvetica",11,"bold"),bg="#f5f5fa",padx=10,pady=10)
        right.grid(row=0,column=1,sticky="nsew")
        self.log=tk.Text(right,height=20,width=35,font=("Courier",8),bg="#1a1a2e",fg="#aaa",state="disabled");self.log.pack()
        tk.Button(right,text="Ecouter",font=("Helvetica",10),bg="#8b5cf6",fg="white",command=self.listen).pack(pady=5)
    def send(self,cmd):self.bt.send(cmd);self.add_log(f"> {cmd}")
    def on_voice(self,txt):self.add_log(f"Voice: {txt}");self.send(txt)
    def listen(self):threading.Thread(target=self.voice.listen,daemon=True).start()
    def add_log(self,msg):self.log.config(state="normal");self.log.insert("end",msg+"\n");self.log.see("end");self.log.config(state="disabled")
    def run(self):self.bt.connect();self.root.mainloop()

if __name__=="__main__":RobotApp().run()
