"""
Interface graphique de controle du Robot d'Assistance PMR.
Connexion Bluetooth, pilotage manuel, controle vocal, monitoring.
Hind Jabrane - Projet Robot Assistance PMR
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import threading
import time
from datetime import datetime
from bluetooth_serial import BluetoothSerial, ConnectionState
from voice_control import VoiceController

WIN_WIDTH, WIN_HEIGHT = 960, 680
BG = "#f0f2f5"
HEADER_BG = "#1e3a5f"
MOVE_BG = "#2563eb"
ARM_BG = "#06d6a0"
STOP_BG = "#dc2626"
VOICE_BG = "#7c3aed"
LOG_BG, LOG_FG = "#1a1a2e", "#c8c8d0"
STATUS_POLL_MS = 500


class SettingsDialog(tk.Toplevel):
    """Fenetre de reglages port/debit."""
    def __init__(self, parent, port, baud):
        super().__init__(parent)
        self.title("Parametres")
        self.geometry("340x200")
        self.resizable(False, False)
        self.transient(parent)
        self.grab_set()
        self.result = None
        f = ttk.Frame(self, padding=12)
        f.pack(fill="both", expand=True)
        ttk.Label(f, text="Port:").grid(row=0, column=0, sticky="w", pady=3)
        self.port_var = tk.StringVar(value=port or "")
        ttk.Combobox(f, textvariable=self.port_var, width=18,
                     values=[p["device"] for p in BluetoothSerial.scan_ports()]
                     ).grid(row=0, column=1, pady=3, padx=6)
        ttk.Label(f, text="Bauds:").grid(row=1, column=0, sticky="w", pady=3)
        self.baud_var = tk.StringVar(value=str(baud))
        ttk.Combobox(f, textvariable=self.baud_var, width=18,
                     values=["9600","19200","38400","57600","115200"]
                     ).grid(row=1, column=1, pady=3, padx=6)
        bf = ttk.Frame(f)
        bf.grid(row=2, column=0, columnspan=2, pady=12)
        ttk.Button(bf, text="Valider", command=self._ok).pack(side="left", padx=4)
        ttk.Button(bf, text="Annuler", command=self.destroy).pack(side="left", padx=4)

    def _ok(self):
        self.result = {"port": self.port_var.get(), "baud": int(self.baud_var.get())}
        self.destroy()


class RobotApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Robot Assistance PMR")
        self.root.geometry(f"{WIN_WIDTH}x{WIN_HEIGHT}")
        self.root.minsize(780, 560)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.bt = BluetoothSerial()
        self.bt.on_connect(self._on_bt_connect)
        self.bt.on_disconnect(self._on_bt_disconnect)
        self.bt.on_raw_receive(self._on_bt_receive)

        self.voice = VoiceController(callback=self._on_voice_cmd)
        self.voice.on_status(self._on_voice_status)
        self.voice.on_error(self._on_voice_error)

        self.selected_port = tk.StringVar()
        self.speed_var = tk.IntVar(value=50)
        self.mode_var = tk.StringVar(value="manuel")
        self.slider_base = tk.IntVar(value=90)
        self.slider_epaule = tk.IntVar(value=90)
        self.slider_coude = tk.IntVar(value=90)
        self.battery_level = 0
        self._voice_active = False
        self._polling = False

        self._build_menu()
        self._build_ui()
        self._bind_keys()
        self._poll_status()

    def _build_menu(self):
        bar = tk.Menu(self.root)
        self.root.config(menu=bar)
        fm = tk.Menu(bar, tearoff=0)
        fm.add_command(label="Exporter le journal...", command=self._export_log)
        fm.add_separator()
        fm.add_command(label="Parametres...", command=self._open_settings)
        fm.add_separator()
        fm.add_command(label="Quitter", command=self._on_close)
        bar.add_cascade(label="Fichier", menu=fm)
        hm = tk.Menu(bar, tearoff=0)
        hm.add_command(label="A propos", command=self._about)
        bar.add_cascade(label="Aide", menu=hm)

    def _build_ui(self):
        # Bandeau
        hdr = tk.Frame(self.root, bg=HEADER_BG, height=46)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        tk.Label(hdr, text="Robot Assistance PMR", font=("Segoe UI", 15, "bold"),
                 bg=HEADER_BG, fg="white").pack(side="left", padx=12)
        tk.Button(hdr, text="ARRET URGENCE", font=("Segoe UI", 10, "bold"),
                  bg=STOP_BG, fg="white", relief="flat", padx=12,
                  command=self._emergency_stop).pack(side="right", padx=12, pady=7)

        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.columnconfigure(2, weight=2)
        main.rowconfigure(0, weight=1)

        self._build_left(main)
        self._build_center(main)
        self._build_right(main)
        self._build_statusbar()

    def _build_left(self, parent):
        col = ttk.Frame(parent)
        col.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        # Connexion
        cf = ttk.LabelFrame(col, text="Connexion Bluetooth", padding=6)
        cf.pack(fill="x", pady=(0, 6))
        pr = ttk.Frame(cf)
        pr.pack(fill="x", pady=2)
        ttk.Label(pr, text="Port:").pack(side="left")
        ports = [p["device"] for p in BluetoothSerial.scan_ports()]
        auto = BluetoothSerial.find_hc05_port()
        if auto and auto in ports:
            self.selected_port.set(auto)
        elif ports:
            self.selected_port.set(ports[0])
        self.port_combo = ttk.Combobox(pr, textvariable=self.selected_port, values=ports, width=14)
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(pr, text="Scan", width=5, command=self._refresh_ports).pack(side="left")
        self.btn_conn = ttk.Button(cf, text="Connecter", command=self._toggle_conn)
        self.btn_conn.pack(pady=3)
        self.conn_dot = tk.Canvas(cf, width=12, height=12, highlightthickness=0)
        self.conn_dot.pack()
        self._dot = self.conn_dot.create_oval(1, 1, 11, 11, fill="gray", outline="")

        # Mode
        mf = ttk.LabelFrame(col, text="Mode", padding=6)
        mf.pack(fill="x", pady=(0, 6))
        for txt, val in [("Manuel","manuel"),("Semi-auto","semi"),("Vocal","vocal")]:
            ttk.Radiobutton(mf, text=txt, variable=self.mode_var, value=val,
                            command=self._on_mode).pack(anchor="w")

        # Vitesse
        sf = ttk.LabelFrame(col, text="Vitesse", padding=6)
        sf.pack(fill="x", pady=(0, 6))
        ttk.Scale(sf, from_=10, to=100, variable=self.speed_var, orient="horizontal",
                  command=self._on_speed).pack(fill="x")
        self.speed_lbl = ttk.Label(sf, text="50%")
        self.speed_lbl.pack()

        # Vocal
        vf = ttk.LabelFrame(col, text="Controle vocal", padding=6)
        vf.pack(fill="x")
        self.btn_voice = tk.Button(vf, text="Activer la voix", font=("Segoe UI", 9),
                                   bg=VOICE_BG, fg="white", relief="flat",
                                   command=self._toggle_voice)
        self.btn_voice.pack(fill="x", pady=2)
        self.voice_lbl = ttk.Label(vf, text="Inactif", foreground="gray")
        self.voice_lbl.pack()

    def _build_center(self, parent):
        col = ttk.Frame(parent)
        col.grid(row=0, column=1, sticky="nsew", padx=4)

        # Pad directionnel
        df = ttk.LabelFrame(col, text="Deplacement", padding=6)
        df.pack(fill="x", pady=(0, 6))
        pad = ttk.Frame(df)
        pad.pack()
        bs = {"width": 5, "height": 2, "font": ("Segoe UI", 9)}
        dirs = [
            (0, 0, "\\", "MOVE_FWD_LEFT"), (0, 1, "^", "MOVE_FWD"), (0, 2, "/", "MOVE_FWD_RIGHT"),
            (1, 0, "<", "TURN_LEFT"),       (1, 1, "STOP", "STOP"),  (1, 2, ">", "TURN_RIGHT"),
            (2, 0, "/", "MOVE_BWD_LEFT"),   (2, 1, "v", "MOVE_BWD"),(2, 2, "\\", "MOVE_BWD_RIGHT"),
        ]
        for r, c, txt, cmd in dirs:
            bg = STOP_BG if cmd == "STOP" else MOVE_BG
            tk.Button(pad, text=txt, bg=bg, fg="white", **bs,
                      command=lambda x=cmd: self._send(x)).grid(row=r, column=c, padx=2, pady=2)

        # Bras
        af = ttk.LabelFrame(col, text="Bras robotique", padding=6)
        af.pack(fill="x", pady=(0, 6))
        ag = ttk.Frame(af)
        ag.pack(pady=3)
        abtn = {"font": ("Segoe UI", 9), "fg": "white", "relief": "flat", "width": 11, "pady": 2}
        tk.Button(ag, text="Monter", bg=ARM_BG, command=lambda: self._send("ARM_UP"), **abtn).grid(row=0, column=0, padx=2, pady=2)
        tk.Button(ag, text="Descendre", bg=ARM_BG, command=lambda: self._send("ARM_DOWN"), **abtn).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(ag, text="Attraper", bg="#0d9488", command=lambda: self._send("GRIP_CLOSE"), **abtn).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(ag, text="Lacher", bg="#0d9488", command=lambda: self._send("GRIP_OPEN"), **abtn).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(ag, text="Home", bg="#6366f1", command=lambda: self._send("ARM_HOME"), **abtn).grid(row=2, column=0, columnspan=2, padx=2, pady=2)

        # Sliders articulations
        jf = ttk.LabelFrame(col, text="Angles articulations", padding=6)
        jf.pack(fill="x")
        for label, var, jid in [("Base", self.slider_base, "base"),
                                 ("Epaule", self.slider_epaule, "epaule"),
                                 ("Coude", self.slider_coude, "coude")]:
            row = ttk.Frame(jf)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=f"{label}:", width=7).pack(side="left")
            ttk.Scale(row, from_=0, to=180, variable=var, orient="horizontal",
                      command=lambda v, j=jid, va=var: self._on_joint(j, va)).pack(side="left", fill="x", expand=True, padx=3)
            lbl = ttk.Label(row, text="90", width=4)
            lbl.pack(side="left")
            var._lbl = lbl

    def _build_right(self, parent):
        col = ttk.Frame(parent)
        col.grid(row=0, column=2, sticky="nsew", padx=(4, 0))
        lf = ttk.LabelFrame(col, text="Journal", padding=4)
        lf.pack(fill="both", expand=True)
        self.log = scrolledtext.ScrolledText(lf, height=28, width=38, font=("Consolas", 9),
                                             bg=LOG_BG, fg=LOG_FG, state="disabled",
                                             wrap="word", borderwidth=0)
        self.log.pack(fill="both", expand=True)
        self.log.tag_configure("tx", foreground="#60a5fa")
        self.log.tag_configure("rx", foreground="#34d399")
        self.log.tag_configure("voice", foreground="#c084fc")
        self.log.tag_configure("error", foreground="#f87171")
        self.log.tag_configure("sys", foreground="#fbbf24")
        bf = ttk.Frame(lf)
        bf.pack(fill="x", pady=(3, 0))
        ttk.Button(bf, text="Effacer", command=self._clear_log).pack(side="left", padx=2)
        ttk.Button(bf, text="Exporter...", command=self._export_log).pack(side="left", padx=2)

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg="#e2e8f0", height=26)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)
        self.st_conn = tk.Label(bar, text="Deconnecte", bg="#e2e8f0", fg="#64748b", font=("Segoe UI", 9), padx=8)
        self.st_conn.pack(side="left")
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=3, pady=2)
        self.st_bat = tk.Label(bar, text="Batterie: --", bg="#e2e8f0", fg="#64748b", font=("Segoe UI", 9), padx=8)
        self.st_bat.pack(side="left")
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=3, pady=2)
        self.st_mode = tk.Label(bar, text="Mode: manuel", bg="#e2e8f0", fg="#64748b", font=("Segoe UI", 9), padx=8)
        self.st_mode.pack(side="left")
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=3, pady=2)
        self.st_obs = tk.Label(bar, text="Obstacles: --", bg="#e2e8f0", fg="#64748b", font=("Segoe UI", 9), padx=8)
        self.st_obs.pack(side="left")

    # -- Raccourcis clavier --
    def _bind_keys(self):
        self.root.bind("<Up>", lambda e: self._send("MOVE_FWD"))
        self.root.bind("<Down>", lambda e: self._send("MOVE_BWD"))
        self.root.bind("<Left>", lambda e: self._send("TURN_LEFT"))
        self.root.bind("<Right>", lambda e: self._send("TURN_RIGHT"))
        self.root.bind("<space>", lambda e: self._send("STOP"))
        self.root.bind("<Escape>", lambda e: self._emergency_stop())
        for key in ["<KeyRelease-Up>", "<KeyRelease-Down>", "<KeyRelease-Left>", "<KeyRelease-Right>"]:
            self.root.bind(key, lambda e: self._send("STOP"))

    # -- Envoi commandes --
    def _send(self, cmd):
        if not self.bt.is_connected():
            self._add_log("Pas connecte", "error")
            return
        self.bt.send(cmd)
        self._add_log(f"> {cmd}", "tx")

    def _emergency_stop(self):
        if self.bt.is_connected():
            self.bt.send("EMERGENCY_STOP")
            self._add_log("> EMERGENCY_STOP", "error")
        if self._voice_active:
            self.voice.stop_continuous()
            self._voice_active = False
            self.btn_voice.config(text="Activer la voix")

    # -- Connexion BT --
    def _toggle_conn(self):
        if self.bt.is_connected():
            self.bt.disconnect()
        else:
            port = self.selected_port.get()
            if not port:
                messagebox.showwarning("Connexion", "Selectionne un port")
                return
            self.btn_conn.config(state="disabled")
            self._add_log("Connexion en cours...", "sys")
            threading.Thread(target=self._do_connect, args=(port,), daemon=True).start()

    def _do_connect(self, port):
        ok = self.bt.connect(port=port)
        self.root.after(0, self._connect_done, ok)

    def _connect_done(self, ok):
        self.btn_conn.config(state="normal")
        if ok:
            self._add_log(f"Connecte sur {self.bt.port}", "sys")
        else:
            self._add_log("Echec connexion", "error")

    def _refresh_ports(self):
        ports = [p["device"] for p in BluetoothSerial.scan_ports()]
        self.port_combo.config(values=ports)

    def _on_bt_connect(self):
        self.root.after(0, self._update_conn_ui, True)

    def _on_bt_disconnect(self):
        self.root.after(0, self._update_conn_ui, False)

    def _on_bt_receive(self, line):
        self.root.after(0, self._handle_rx, line)

    def _update_conn_ui(self, connected):
        if connected:
            self.conn_dot.itemconfig(self._dot, fill="#22c55e")
            self.btn_conn.config(text="Deconnecter")
            self.st_conn.config(text=f"Connecte ({self.bt.port})", fg="#16a34a")
            self._polling = True
        else:
            self.conn_dot.itemconfig(self._dot, fill="#ef4444")
            self.btn_conn.config(text="Connecter")
            self.st_conn.config(text="Deconnecte", fg="#dc2626")
            self._polling = False

    def _handle_rx(self, line):
        self._add_log(f"< {line}", "rx")
        if line.startswith("STATUS:"):
            self._parse_status(line[7:])

    def _parse_status(self, data):
        """Parse 'BAT=78,OBS_F=35,OBS_R=120,ARM=90/45/60'"""
        try:
            for part in data.split(","):
                if "=" not in part:
                    continue
                k, v = part.split("=", 1)
                k, v = k.strip(), v.strip()
                if k == "BAT":
                    bat = int(v)
                    c = "#16a34a" if bat > 30 else "#dc2626"
                    self.st_bat.config(text=f"Batterie: {bat}%", fg=c)
                elif k == "OBS_F":
                    front = int(v)
                    self.st_obs.config(text=f"Obstacles: av={front}cm", fg="#dc2626" if front < 20 else "#64748b")
                elif k == "ARM":
                    pass  # angles pour affichage futur
        except (ValueError, IndexError):
            pass

    def _poll_status(self):
        if self._polling and self.bt.is_connected():
            self.bt.send("STATUS")
        self.root.after(STATUS_POLL_MS, self._poll_status)

    # -- Vitesse, mode, articulations --
    def _on_speed(self, val):
        s = int(float(val))
        self.speed_lbl.config(text=f"{s}%")
        if self.bt.is_connected():
            self.bt.send(f"SPEED:{s}")

    def _on_mode(self):
        m = self.mode_var.get()
        self.st_mode.config(text=f"Mode: {m}")
        if self.bt.is_connected():
            self.bt.send(f"MODE:{m.upper()}")
        if m == "vocal" and not self._voice_active:
            self._start_voice()
        elif m != "vocal" and self._voice_active:
            self._stop_voice()

    def _on_joint(self, jid, var):
        a = int(var.get())
        if hasattr(var, "_lbl"):
            var._lbl.config(text=str(a))
        if self.bt.is_connected():
            self.bt.send(f"JOINT:{jid}:{a}")

    # -- Vocal --
    def _toggle_voice(self):
        if self._voice_active:
            self._stop_voice()
        else:
            self._start_voice()

    def _start_voice(self):
        self._voice_active = True
        self.btn_voice.config(text="Desactiver la voix", bg="#9333ea")
        self.voice_lbl.config(text="Ecoute...", foreground=VOICE_BG)
        self.voice.start_continuous()
        self._add_log("Vocal active", "voice")

    def _stop_voice(self):
        self._voice_active = False
        self.btn_voice.config(text="Activer la voix", bg=VOICE_BG)
        self.voice_lbl.config(text="Inactif", foreground="gray")
        self.voice.stop_continuous()
        self._add_log("Vocal desactive", "voice")

    def _on_voice_cmd(self, cmd):
        self.root.after(0, self._do_voice_cmd, cmd)

    def _do_voice_cmd(self, cmd):
        self._add_log(f"Vocal: {cmd}", "voice")
        self._send(cmd)

    def _on_voice_status(self, msg):
        self.root.after(0, self.voice_lbl.config, {"text": msg})

    def _on_voice_error(self, msg):
        self.root.after(0, self._add_log, f"Erreur vocale: {msg}", "error")

    # -- Journal --
    def _add_log(self, msg, tag="sys"):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log.config(state="normal")
        self.log.insert("end", f"[{ts}] {msg}\n", tag)
        self.log.see("end")
        self.log.config(state="disabled")

    def _clear_log(self):
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")

    def _export_log(self):
        txt = self.log.get("1.0", "end").strip()
        if not txt:
            return
        path = filedialog.asksaveasfilename(defaultextension=".txt",
                filetypes=[("Texte","*.txt")],
                initialfile=f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(txt)

    # -- Dialogues --
    def _open_settings(self):
        dlg = SettingsDialog(self.root, self.selected_port.get(), self.bt.baud)
        self.root.wait_window(dlg)
        if dlg.result:
            self.selected_port.set(dlg.result["port"])
            self.bt.baud = dlg.result["baud"]
            self._add_log(f"Parametres: {dlg.result}", "sys")

    def _about(self):
        messagebox.showinfo("A propos",
            "Robot Assistance PMR\nPanneau de controle v1.0\n\n"
            "Projet universitaire - Robotique\nHind Jabrane\n\n"
            "Fauteuil roulant robotise, Bluetooth + commande vocale.")

    # -- Fermeture --
    def _on_close(self):
        if self._voice_active:
            self.voice.stop_continuous()
        if self.bt.is_connected():
            self.bt.send("STOP")
            time.sleep(0.1)
        self.bt.close()
        self.root.destroy()

    def run(self):
        self._add_log("Application demarree", "sys")
        self.root.mainloop()


if __name__ == "__main__":
    RobotApp().run()
