"""
Communication serie Bluetooth HC-05 pour le robot PMR.
Gestion de la connexion, envoi/reception, reconnexion auto.
Hind Jabrane - Projet Robot Assistance PMR
"""

import serial
import serial.tools.list_ports
import threading
import queue
import time
import logging
import os
from datetime import datetime

HC05_IDENTIFIERS = ["HC-05", "HC-06", "Bluetooth", "BTHENUM", "rfcomm"]
RESPONSE_OK = "OK:"
RESPONSE_ERR = "ERR:"
KEEPALIVE_CMD = "PING"
KEEPALIVE_INTERVAL = 3.0
ACK_TIMEOUT = 1.5
MAX_RETRIES = 3
RECONNECT_DELAY = 2.0

logger = logging.getLogger("bluetooth_serial")


class ConnectionState:
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"


class CommStats:
    """Compteurs de communication pour le diagnostic."""
    def __init__(self):
        self.messages_sent = 0
        self.messages_received = 0
        self.errors = 0
        self.reconnections = 0
        self.bytes_sent = 0
        self.bytes_received = 0
        self.last_send_time = None
        self.last_recv_time = None

    def reset(self):
        self.__init__()

    def summary(self):
        return (f"TX: {self.messages_sent} ({self.bytes_sent}o) | "
                f"RX: {self.messages_received} ({self.bytes_received}o) | "
                f"Err: {self.errors} | Reconn: {self.reconnections}")


class BluetoothSerial:
    """
    Gestionnaire de communication serie vers le module HC-05.
    Detection auto du port, reconnexion, file d'attente, callbacks.
    """

    def __init__(self, baud=9600, timeout=1.0, log_dir=None):
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._state = ConnectionState.DISCONNECTED
        self._port = None
        self._send_queue = queue.Queue()
        self._running = False
        self._lock = threading.Lock()
        self._handlers = {}
        self._on_connect_cb = []
        self._on_disconnect_cb = []
        self._on_raw_recv_cb = []
        self._recv_buffer = ""
        self.stats = CommStats()
        self._log_file = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._log_file = open(os.path.join(log_dir, f"bt_{stamp}.log"), "a", encoding="utf-8")

    @staticmethod
    def scan_ports():
        """Retourne la liste des ports serie disponibles."""
        return [{"device": p.device, "description": p.description, "hwid": p.hwid}
                for p in serial.tools.list_ports.comports()]

    @staticmethod
    def find_hc05_port():
        """Cherche un port correspondant au HC-05 (COM ou /dev/rfcomm*)."""
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").upper()
            hwid = (p.hwid or "").upper()
            for ident in HC05_IDENTIFIERS:
                if ident.upper() in desc or ident.upper() in hwid:
                    return p.device
        import glob
        for pattern in ["/dev/rfcomm*", "/dev/ttyUSB*"]:
            matches = sorted(glob.glob(pattern))
            if matches:
                return matches[0]
        return None

    def connect(self, port=None, retries=MAX_RETRIES):
        """Ouvre la connexion serie avec retry. Detection auto si port=None."""
        if self.is_connected():
            self.disconnect()
        if port is None:
            port = self.find_hc05_port()
        if port is None:
            logger.error("Aucun port HC-05 trouve")
            return False
        self._state = ConnectionState.CONNECTING
        self._port = port
        for attempt in range(1, retries + 1):
            try:
                logger.info(f"Tentative {attempt}/{retries} sur {port}")
                self.ser = serial.Serial(port=port, baudrate=self.baud,
                                         timeout=self.timeout, write_timeout=self.timeout)
                time.sleep(0.3)
                self._state = ConnectionState.CONNECTED
                self._running = True
                self._start_threads()
                self._notify_connect()
                self._log_comm("SYS", f"Connecte sur {port}")
                return True
            except serial.SerialException as e:
                logger.warning(f"Echec tentative {attempt}: {e}")
                self.stats.errors += 1
                if attempt < retries:
                    time.sleep(RECONNECT_DELAY)
        self._state = ConnectionState.DISCONNECTED
        return False

    def disconnect(self):
        """Ferme proprement la connexion et arrete les threads."""
        self._running = False
        self._state = ConnectionState.DISCONNECTED
        while not self._send_queue.empty():
            try: self._send_queue.get_nowait()
            except queue.Empty: break
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        self.ser = None
        self._notify_disconnect()
        self._log_comm("SYS", "Deconnecte")

    def is_connected(self):
        return self._state == ConnectionState.CONNECTED and self.ser is not None and self.ser.is_open

    @property
    def state(self):
        return self._state

    @property
    def port(self):
        return self._port

    def send(self, cmd):
        """Ajoute une commande a la file d'envoi."""
        if not self.is_connected():
            return False
        self._send_queue.put(cmd)
        return True

    def send_blocking(self, cmd, timeout=ACK_TIMEOUT):
        """Envoie et attend l'acquittement. Retourne la reponse ou None."""
        if not self.is_connected():
            return None
        ack_event = threading.Event()
        response = [None]
        def on_ack(msg):
            response[0] = msg
            ack_event.set()
        self.register_handler(RESPONSE_OK, on_ack)
        self.register_handler(RESPONSE_ERR, on_ack)
        self._do_send(cmd)
        ack_event.wait(timeout=timeout)
        self.unregister_handler(RESPONSE_OK, on_ack)
        self.unregister_handler(RESPONSE_ERR, on_ack)
        return response[0]

    def _do_send(self, cmd):
        if not self.is_connected():
            return
        try:
            data = (cmd + "\n").encode("utf-8")
            self.ser.write(data)
            self.ser.flush()
            self.stats.messages_sent += 1
            self.stats.bytes_sent += len(data)
            self.stats.last_send_time = time.time()
            self._log_comm("TX", cmd)
        except serial.SerialException as e:
            self.stats.errors += 1
            self._handle_connection_lost()

    def _process_received_line(self, line):
        line = line.strip()
        if not line:
            return
        self.stats.messages_received += 1
        self.stats.bytes_received += len(line)
        self.stats.last_recv_time = time.time()
        self._log_comm("RX", line)
        for cb in self._on_raw_recv_cb:
            try: cb(line)
            except Exception as e: logger.error(f"Erreur callback: {e}")
        for prefix, handlers in self._handlers.items():
            if line.startswith(prefix):
                for h in handlers:
                    try: h(line)
                    except Exception as e: logger.error(f"Erreur handler: {e}")

    def register_handler(self, prefix, callback):
        if prefix not in self._handlers:
            self._handlers[prefix] = []
        if callback not in self._handlers[prefix]:
            self._handlers[prefix].append(callback)

    def unregister_handler(self, prefix, callback):
        if prefix in self._handlers:
            try: self._handlers[prefix].remove(callback)
            except ValueError: pass

    def on_connect(self, cb):
        self._on_connect_cb.append(cb)

    def on_disconnect(self, cb):
        self._on_disconnect_cb.append(cb)

    def on_raw_receive(self, cb):
        self._on_raw_recv_cb.append(cb)

    def _notify_connect(self):
        for cb in self._on_connect_cb:
            try: cb()
            except Exception: pass

    def _notify_disconnect(self):
        for cb in self._on_disconnect_cb:
            try: cb()
            except Exception: pass

    def _start_threads(self):
        for target in [self._send_loop, self._recv_loop, self._keepalive_loop]:
            threading.Thread(target=target, daemon=True).start()

    def _send_loop(self):
        while self._running:
            try:
                cmd = self._send_queue.get(timeout=0.1)
                self._do_send(cmd)
            except queue.Empty:
                continue

    def _recv_loop(self):
        while self._running:
            if not self.is_connected():
                time.sleep(0.05)
                continue
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                    self._recv_buffer += raw
                    while "\n" in self._recv_buffer:
                        line, self._recv_buffer = self._recv_buffer.split("\n", 1)
                        self._process_received_line(line)
                else:
                    time.sleep(0.02)
            except serial.SerialException:
                self._handle_connection_lost()

    def _keepalive_loop(self):
        while self._running:
            time.sleep(KEEPALIVE_INTERVAL)
            if self.is_connected():
                self._do_send(KEEPALIVE_CMD)

    def _handle_connection_lost(self):
        if self._state == ConnectionState.RECONNECTING:
            return
        logger.warning("Connexion perdue, reconnexion...")
        self._state = ConnectionState.RECONNECTING
        self._notify_disconnect()
        if self.ser:
            try: self.ser.close()
            except Exception: pass
            self.ser = None
        threading.Thread(target=self._reconnect_loop, daemon=True).start()

    def _reconnect_loop(self):
        attempt = 0
        while self._running and self._state == ConnectionState.RECONNECTING:
            attempt += 1
            try:
                self.ser = serial.Serial(port=self._port, baudrate=self.baud,
                                         timeout=self.timeout, write_timeout=self.timeout)
                time.sleep(0.3)
                self._state = ConnectionState.CONNECTED
                self.stats.reconnections += 1
                self._notify_connect()
                self._log_comm("SYS", f"Reconnecte (tentative {attempt})")
                return
            except serial.SerialException:
                time.sleep(RECONNECT_DELAY)

    def _log_comm(self, direction, message):
        if self._log_file:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self._log_file.write(f"[{ts}] {direction}: {message}\n")
            self._log_file.flush()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()
        return False

    def close(self):
        self.disconnect()
        if self._log_file:
            self._log_file.close()
            self._log_file = None
