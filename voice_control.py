"""
Reconnaissance vocale en francais pour le controle du robot PMR.
Utilise SpeechRecognition + Google Speech API.
Hind Jabrane - Projet Robot Assistance PMR
"""

import speech_recognition as sr
import threading
import time
import logging
from difflib import SequenceMatcher

logger = logging.getLogger("voice_control")

# Commandes vocales francaises -> commandes robot
COMMAND_MAP = {
    # Deplacement
    "avance": "MOVE_FWD", "avancer": "MOVE_FWD", "en avant": "MOVE_FWD",
    "recule": "MOVE_BWD", "reculer": "MOVE_BWD", "en arriere": "MOVE_BWD",
    "gauche": "TURN_LEFT", "a gauche": "TURN_LEFT", "tourne a gauche": "TURN_LEFT",
    "droite": "TURN_RIGHT", "a droite": "TURN_RIGHT", "tourne a droite": "TURN_RIGHT",
    "stop": "STOP", "arrete": "STOP", "arrete toi": "STOP", "halte": "STOP",
    "plus vite": "SPEED_UP", "accelere": "SPEED_UP",
    "ralentis": "SPEED_DOWN", "moins vite": "SPEED_DOWN", "doucement": "SPEED_DOWN",
    # Bras
    "monte": "ARM_UP", "monter": "ARM_UP", "leve le bras": "ARM_UP",
    "descend": "ARM_DOWN", "descends": "ARM_DOWN", "baisse le bras": "ARM_DOWN",
    "attrape": "GRIP_CLOSE", "prends": "GRIP_CLOSE", "saisis": "GRIP_CLOSE",
    "lache": "GRIP_OPEN", "pose": "GRIP_OPEN", "relache": "GRIP_OPEN",
    "bras home": "ARM_HOME", "position initiale": "ARM_HOME",
    # Modes
    "mode manuel": "MODE_MANUAL", "mode auto": "MODE_AUTO",
    "mode automatique": "MODE_AUTO", "mode vocal": "MODE_VOICE",
    # Infos
    "batterie": "STATUS_BATTERY", "etat": "STATUS", "position": "STATUS_POS",
    "aide": "HELP",
    # Urgence
    "urgence": "EMERGENCY_STOP", "arret urgence": "EMERGENCY_STOP",
}

DANGEROUS_COMMANDS = {"EMERGENCY_STOP", "SPEED_UP"}
FUZZY_THRESHOLD = 0.70


def similarity(a, b):
    return SequenceMatcher(None, a.lower(), b.lower()).ratio()


def fuzzy_match(text, command_map, threshold=FUZZY_THRESHOLD):
    """
    Correspondance exacte (substring) d'abord, puis floue.
    Retourne (commande_robot, score) ou (None, 0).
    """
    text_lower = text.lower().strip()
    # Correspondance exacte
    for phrase, cmd in command_map.items():
        if phrase in text_lower:
            return cmd, 1.0
    # Correspondance floue
    best_cmd, best_score = None, 0.0
    for phrase, cmd in command_map.items():
        score = similarity(text_lower, phrase)
        if score > best_score:
            best_score = score
            best_cmd = cmd
    if best_score >= threshold:
        return best_cmd, best_score
    return None, best_score


class VoiceController:
    """
    Controleur vocal pour le robot PMR.
    Ecoute le micro, reconnait le francais via Google Speech,
    et traduit en commandes robot.
    """

    def __init__(self, callback=None, lang="fr-FR"):
        self.recognizer = sr.Recognizer()
        self.lang = lang
        self._command_callback = callback
        self._error_callback = None
        self._status_callback = None
        self.confidence_threshold = 0.5
        self.listen_timeout = 5
        self.phrase_time_limit = 8
        self.energy_threshold = 300
        self.dynamic_energy = True
        self._listening = False
        self._continuous = False
        self._stop_event = threading.Event()
        self.history = []
        self._max_history = 50
        self._mic_index = None
        self._awaiting_confirmation = False
        self._pending_command = None
        self._pending_text = None
        self.tts_enabled = False
        self._tts_engine = None

    @staticmethod
    def list_microphones():
        """Liste les microphones disponibles avec leur index."""
        return [{"index": i, "name": name}
                for i, name in enumerate(sr.Microphone.list_microphone_names())]

    def select_microphone(self, index):
        available = sr.Microphone.list_microphone_names()
        if 0 <= index < len(available):
            self._mic_index = index
            return True
        return False

    def _get_microphone(self):
        if self._mic_index is not None:
            return sr.Microphone(device_index=self._mic_index)
        return sr.Microphone()

    def calibrate(self, duration=1.5):
        """Calibre le seuil de bruit ambiant."""
        self._notify_status("Calibration du bruit ambiant...")
        try:
            with self._get_microphone() as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=duration)
                self.energy_threshold = self.recognizer.energy_threshold
            self._notify_status(f"Calibration OK (seuil: {self.energy_threshold:.0f})")
            return True
        except Exception as e:
            self._notify_error(f"Echec calibration: {e}")
            return False

    def listen(self):
        """Ecoute une seule commande (bloquant)."""
        self._listening = True
        self._notify_status("Ecoute en cours...")
        try:
            with self._get_microphone() as source:
                if self.dynamic_energy:
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.4)
                audio = self.recognizer.listen(source, timeout=self.listen_timeout,
                                                phrase_time_limit=self.phrase_time_limit)
            self._process_audio(audio)
        except sr.WaitTimeoutError:
            self._notify_status("Timeout, rien entendu")
        except Exception as e:
            self._notify_error(str(e))
        finally:
            self._listening = False

    def start_continuous(self):
        """Demarre l'ecoute continue dans un thread."""
        if self._continuous:
            return
        self._stop_event.clear()
        self._continuous = True
        threading.Thread(target=self._continuous_loop, daemon=True).start()
        self._notify_status("Ecoute continue activee")

    def stop_continuous(self):
        self._stop_event.set()
        self._continuous = False
        self._notify_status("Ecoute continue arretee")

    def _continuous_loop(self):
        while not self._stop_event.is_set():
            try:
                self.listen()
            except Exception as e:
                logger.error(f"Erreur boucle continue: {e}")
            time.sleep(0.2)
        self._continuous = False
        self._listening = False

    @property
    def is_listening(self):
        return self._listening

    @property
    def is_continuous(self):
        return self._continuous

    def _process_audio(self, audio):
        """Envoie l'audio a Google Speech, parse le texte, identifie la commande."""
        try:
            result = self.recognizer.recognize_google(audio, language=self.lang, show_all=True)
            if not result or not isinstance(result, dict):
                self._notify_status("Pas de parole reconnue")
                return
            alternatives = result.get("alternative", [])
            if not alternatives:
                return
            best = alternatives[0]
            text = best.get("transcript", "").lower().strip()
            confidence = best.get("confidence", 0.0)
            if not text:
                return

            logger.info(f"Reconnu: '{text}' (confiance: {confidence:.2f})")

            if confidence > 0 and confidence < self.confidence_threshold:
                self._notify_status(f"Confiance trop basse ({confidence:.0%}): '{text}'")
                return

            # Confirmation en cours?
            if self._awaiting_confirmation:
                self._handle_confirmation(text)
                return

            cmd, score = fuzzy_match(text, COMMAND_MAP)
            if cmd is None:
                self._notify_status(f"Commande inconnue: '{text}'")
                return

            self._add_to_history(text, cmd, confidence)

            if cmd in DANGEROUS_COMMANDS:
                self._pending_command = cmd
                self._pending_text = text
                self._awaiting_confirmation = True
                self._notify_status(f"Confirmer '{text}'? Dis 'oui' ou 'non'")
                return

            self._fire_command(cmd, text, confidence)

        except sr.UnknownValueError:
            self._notify_status("Parole non comprise")
        except sr.RequestError as e:
            self._notify_error(f"Service vocal indisponible: {e}")

    def _handle_confirmation(self, text):
        confirm_words = ["oui", "ok", "confirme", "vas-y", "go", "d'accord"]
        deny_words = ["non", "annule", "stop"]
        text_lower = text.lower()
        confirmed = any(w in text_lower for w in confirm_words)
        denied = any(w in text_lower for w in deny_words)
        if confirmed and not denied:
            cmd = self._pending_command
            self._awaiting_confirmation = False
            self._pending_command = None
            self._fire_command(cmd, self._pending_text, 1.0)
            self._pending_text = None
        elif denied:
            self._awaiting_confirmation = False
            self._pending_command = None
            self._pending_text = None
            self._notify_status("Commande annulee")
        else:
            self._notify_status("Dis 'oui' pour confirmer ou 'non' pour annuler")

    def _fire_command(self, cmd, text, confidence):
        if self._command_callback:
            try:
                self._command_callback(cmd)
            except Exception as e:
                logger.error(f"Erreur callback: {e}")
        if self.tts_enabled:
            self._speak(f"Commande: {cmd}")

    def _add_to_history(self, text, cmd, confidence):
        self.history.append({"time": time.time(), "text": text,
                             "command": cmd, "confidence": confidence})
        if len(self.history) > self._max_history:
            self.history.pop(0)

    def get_history(self, n=10):
        return self.history[-n:]

    def clear_history(self):
        self.history.clear()

    def enable_tts(self):
        """Active la synthese vocale (pyttsx3)."""
        try:
            import pyttsx3
            self._tts_engine = pyttsx3.init()
            for voice in self._tts_engine.getProperty("voices"):
                if "french" in voice.name.lower() or "fr" in voice.id.lower():
                    self._tts_engine.setProperty("voice", voice.id)
                    break
            self._tts_engine.setProperty("rate", 160)
            self.tts_enabled = True
        except ImportError:
            logger.warning("pyttsx3 non installe, TTS desactive")
        except Exception as e:
            logger.error(f"Erreur init TTS: {e}")

    def disable_tts(self):
        self.tts_enabled = False
        self._tts_engine = None

    def _speak(self, text):
        if self._tts_engine:
            try:
                self._tts_engine.say(text)
                self._tts_engine.runAndWait()
            except Exception:
                pass

    def on_error(self, callback):
        self._error_callback = callback

    def on_status(self, callback):
        self._status_callback = callback

    def _notify_error(self, msg):
        if self._error_callback:
            try: self._error_callback(msg)
            except Exception: pass

    def _notify_status(self, msg):
        logger.debug(f"Status: {msg}")
        if self._status_callback:
            try: self._status_callback(msg)
            except Exception: pass
