/* ============================================================================
 * Robot d'Assistance pour Personnes a Mobilite Reduite
 * Arduino Mega 2560
 *
 * Systeme embarque sur fauteuil roulant motorise :
 *   - Bras robotique 3 DOF + pince (4 servomoteurs)
 *   - Commande par joystick analogique
 *   - Telecommande Bluetooth (HC-05) depuis application PC
 *   - Detection d'obstacles par 4 capteurs ultrason HC-SR04
 *   - Affichage OLED 128x64 (SSD1306, I2C)
 *   - Reconnaissance vocale via module Bluetooth
 *   - Arret d'urgence materiel (bouton poussoir)
 *
 * Auteur : Hind Jabrane
 * ENSMM Besancon -- Projet Mecatronique
 * ============================================================================ */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include "arm_controller.h"
#include "obstacle_detection.h"
#include "motor_driver.h"

/* --------------------------------------------------------------------------
 * Configuration des broches
 * -------------------------------------------------------------------------- */

// Servomoteurs du bras (PWM)
#define PIN_SERVO_BASE      4
#define PIN_SERVO_EPAULE    5
#define PIN_SERVO_COUDE     6
#define PIN_SERVO_PINCE     7

// Moteurs DC (L298N) -- roue gauche et roue droite
#define PIN_MOTOR_L_EN      2     // PWM vitesse gauche
#define PIN_MOTOR_L_IN1     24    // direction gauche A
#define PIN_MOTOR_L_IN2     26    // direction gauche B
#define PIN_MOTOR_R_EN      3     // PWM vitesse droite
#define PIN_MOTOR_R_IN1     28    // direction droite A
#define PIN_MOTOR_R_IN2     30    // direction droite B

// Bluetooth HC-05
#define PIN_BT_RX           10
#define PIN_BT_TX           11

// Joystick analogique
#define PIN_JOY_X           A0
#define PIN_JOY_Y           A1
#define PIN_JOY_BTN         22    // bouton poussoir joystick
#define JOY_CENTER           512
#define JOY_DEADZONE         50

// Capteurs ultrason (trig, echo) x 4 directions
#define PIN_US_FRONT_TRIG   32
#define PIN_US_FRONT_ECHO   33
#define PIN_US_BACK_TRIG    34
#define PIN_US_BACK_ECHO    35
#define PIN_US_LEFT_TRIG    36
#define PIN_US_LEFT_ECHO    37
#define PIN_US_RIGHT_TRIG   38
#define PIN_US_RIGHT_ECHO   39

// Bouton arret d'urgence
#define PIN_ARRET_URGENCE   23    // INPUT_PULLUP, actif LOW

// LED de statut
#define PIN_LED_STATUS      13
#define PIN_LED_URGENCE     12

// Buzzer
#define PIN_BUZZER          44

// OLED
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_RESET          -1

/* --------------------------------------------------------------------------
 * Parametres de fonctionnement
 * -------------------------------------------------------------------------- */

#define VITESSE_MAX          220   // PWM max moteurs (0-255)
#define VITESSE_MIN          60    // PWM min (en dessous les moteurs sifflent)
#define DISTANCE_SECURITE    30.0  // cm, arret automatique
#define DISTANCE_ALERTE      50.0  // cm, ralentissement progressif
#define DISTANCE_LATERALE    20.0  // cm, alerte laterale
#define PERIODE_OBSTACLE     100   // ms entre lectures ultrason
#define PERIODE_OLED         500   // ms entre rafraichissements ecran
#define PERIODE_STATUS_BT    1000  // ms entre envois statut Bluetooth
#define DEBOUNCE_PINCE       300   // ms anti-rebond bouton pince
#define BAUD_SERIAL          115200
#define BAUD_BLUETOOTH       9600

/* --------------------------------------------------------------------------
 * Variables globales
 * -------------------------------------------------------------------------- */

ArmController bras(PIN_SERVO_BASE, PIN_SERVO_EPAULE, PIN_SERVO_COUDE, PIN_SERVO_PINCE);
ObstacleDetector obstacles;
MotorDriver moteurs(PIN_MOTOR_L_EN, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2,
                     PIN_MOTOR_R_EN, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2);
SoftwareSerial btSerial(PIN_BT_RX, PIN_BT_TX);
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// Etats du systeme
bool arretUrgence = false;
bool obstacleDetecte = false;
int modeActuel = 0;  // 0 = joystick, 1 = bluetooth, 2 = vocal

// Distances mesurees (cm)
float distFront = -1;
float distBack  = -1;
float distLeft  = -1;
float distRight = -1;

// Compteurs de temps (millis)
unsigned long dernierObstacle  = 0;
unsigned long dernierOLED      = 0;
unsigned long dernierStatusBT  = 0;
unsigned long dernierPince     = 0;
unsigned long dernierBip       = 0;

// Vitesse actuelle des moteurs
int vitesseGauche = 0;
int vitesseDroite = 0;

// Compteur de commandes recues depuis le demarrage
unsigned long nbCommandes = 0;

// Historique des obstacles pour le filtrage
float histFront[5] = {-1, -1, -1, -1, -1};
int idxHist = 0;

/* --------------------------------------------------------------------------
 * Prototypes
 * -------------------------------------------------------------------------- */

void verifierArretUrgence();
void lireJoystick();
void traiterCommande(String cmd);
void verifierObstacles();
float filtrerDistance(float nouvelle, float* historique, int taille);
void afficherDemarrage();
void afficherEtat(String ligne1, String ligne2);
void mettreAJourOLED();
void envoyerStatutBT();
void envoyerStatutComplet();
void envoyerDistances();
void envoyerAngles();
void envoyerAide();
void bipAlerte(int duree);

/* --------------------------------------------------------------------------
 * Initialisation
 * -------------------------------------------------------------------------- */

void setup() {
    Serial.begin(BAUD_SERIAL);
    Serial.println(F("=== Robot PMR -- Demarrage ==="));

    // Configuration des broches d'entree/sortie
    pinMode(PIN_JOY_BTN, INPUT_PULLUP);
    pinMode(PIN_ARRET_URGENCE, INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_URGENCE, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);
    digitalWrite(PIN_LED_URGENCE, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    // Initialisation du bras robotique
    Serial.println(F("[1/5] Init bras robotique..."));
    bras.begin();

    // Initialisation des capteurs ultrason
    Serial.println(F("[2/5] Init capteurs ultrason..."));
    obstacles.begin(
        PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO,
        PIN_US_BACK_TRIG, PIN_US_BACK_ECHO,
        PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO,
        PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO
    );

    // Initialisation des moteurs
    Serial.println(F("[3/5] Init moteurs..."));
    moteurs.begin();

    // Initialisation Bluetooth
    Serial.println(F("[4/5] Init Bluetooth..."));
    btSerial.begin(BAUD_BLUETOOTH);

    // Initialisation OLED
    Serial.println(F("[5/5] Init OLED..."));
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("  ERREUR : OLED non detecte sur 0x3C"));
    } else {
        Serial.println(F("  OLED OK"));
    }

    // Position initiale du bras
    bras.goHome();
    delay(500);

    // Ecran d'accueil
    afficherDemarrage();
    bipAlerte(100);
    delay(1500);
    afficherEtat("Pret", "Mode joystick");

    // LED statut allumee = systeme operationnel
    digitalWrite(PIN_LED_STATUS, HIGH);

    Serial.println(F("=== Systeme operationnel ==="));
    Serial.println(F("Tapez 'help' pour la liste des commandes"));
}

/* --------------------------------------------------------------------------
 * Boucle principale
 * -------------------------------------------------------------------------- */

void loop() {
    unsigned long maintenant = millis();

    // --- Priorite 1 : Arret d'urgence ---
    verifierArretUrgence();

    // --- Priorite 2 : Lecture des commandes Bluetooth ---
    if (btSerial.available()) {
        String cmd = btSerial.readStringUntil('\n');
        traiterCommande(cmd);
    }

    // --- Priorite 3 : Lecture des commandes serie (debug/supervision) ---
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        traiterCommande(cmd);
    }

    // --- Controle par joystick (si actif et pas d'urgence) ---
    if (!arretUrgence && modeActuel == 0) {
        lireJoystick();
    }

    // --- Detection d'obstacles periodique ---
    if (maintenant - dernierObstacle >= PERIODE_OBSTACLE) {
        verifierObstacles();
        dernierObstacle = maintenant;
    }

    // --- Rafraichissement OLED periodique ---
    if (maintenant - dernierOLED >= PERIODE_OLED) {
        mettreAJourOLED();
        dernierOLED = maintenant;
    }

    // --- Envoi statut Bluetooth periodique ---
    if (maintenant - dernierStatusBT >= PERIODE_STATUS_BT) {
        envoyerStatutBT();
        dernierStatusBT = maintenant;
    }

    // --- Clignotement LED urgence ---
    if (arretUrgence) {
        digitalWrite(PIN_LED_URGENCE, (maintenant / 250) % 2);
    }

    // --- Bip d'alerte si obstacle proche ---
    if (obstacleDetecte && maintenant - dernierBip > 500) {
        bipAlerte(50);
        dernierBip = maintenant;
    }
}

/* --------------------------------------------------------------------------
 * Arret d'urgence
 * -------------------------------------------------------------------------- */

void verifierArretUrgence() {
    // Le bouton est en INPUT_PULLUP, donc actif quand LOW
    if (digitalRead(PIN_ARRET_URGENCE) == LOW && !arretUrgence) {
        arretUrgence = true;
        moteurs.stop();
        vitesseGauche = 0;
        vitesseDroite = 0;

        Serial.println(F("*** ARRET D'URGENCE ACTIVE ***"));
        btSerial.println(F("URGENCE:ARRET_MATERIEL"));
        afficherEtat("URGENCE", "ARRET MOTEURS");
        digitalWrite(PIN_LED_URGENCE, HIGH);
        bipAlerte(500);
    }
}

/* --------------------------------------------------------------------------
 * Lecture du joystick analogique
 * -------------------------------------------------------------------------- */

void lireJoystick() {
    // Lecture des axes analogiques (0-1023, centre a 512)
    int rawX = analogRead(PIN_JOY_X) - JOY_CENTER;
    int rawY = analogRead(PIN_JOY_Y) - JOY_CENTER;

    // Zone morte autour du centre
    if (abs(rawX) < JOY_DEADZONE) rawX = 0;
    if (abs(rawY) < JOY_DEADZONE) rawY = 0;

    // Commande differentielle (tank drive)
    // Y = avancer/reculer, X = tourner gauche/droite
    int cmdGauche = rawY + rawX;
    int cmdDroite = rawY - rawX;

    // Conversion en PWM (0-255)
    vitesseGauche = map(constrain(abs(cmdGauche), 0, 512), 0, 512, 0, VITESSE_MAX);
    vitesseDroite = map(constrain(abs(cmdDroite), 0, 512), 0, 512, 0, VITESSE_MAX);

    // Seuil minimum pour eviter les sifflements
    if (vitesseGauche > 0 && vitesseGauche < VITESSE_MIN) vitesseGauche = VITESSE_MIN;
    if (vitesseDroite > 0 && vitesseDroite < VITESSE_MIN) vitesseDroite = VITESSE_MIN;

    // Ralentissement progressif si obstacle dans la zone d'alerte
    if (distFront > 0 && distFront < DISTANCE_ALERTE && distFront >= DISTANCE_SECURITE) {
        float facteur = (distFront - DISTANCE_SECURITE) / (DISTANCE_ALERTE - DISTANCE_SECURITE);
        facteur = constrain(facteur, 0.2, 1.0);
        vitesseGauche = (int)(vitesseGauche * facteur);
        vitesseDroite = (int)(vitesseDroite * facteur);
    }

    // Direction : true = avant, false = arriere
    bool dirGauche = (cmdGauche >= 0);
    bool dirDroite = (cmdDroite >= 0);

    // Blocage complet si obstacle frontal trop proche et on avance
    if (distFront > 0 && distFront < DISTANCE_SECURITE && dirGauche && dirDroite) {
        moteurs.stop();
        vitesseGauche = 0;
        vitesseDroite = 0;
        return;
    }

    // Blocage si obstacle arriere trop proche et on recule
    if (distBack > 0 && distBack < DISTANCE_SECURITE && !dirGauche && !dirDroite) {
        moteurs.stop();
        vitesseGauche = 0;
        vitesseDroite = 0;
        return;
    }

    // Application de la commande moteur
    moteurs.drive(vitesseGauche, dirGauche, vitesseDroite, dirDroite);

    // Bouton poussoir du joystick : toggle pince
    unsigned long maintenant = millis();
    if (digitalRead(PIN_JOY_BTN) == LOW && maintenant - dernierPince > DEBOUNCE_PINCE) {
        bras.toggleGripper();
        dernierPince = maintenant;
        Serial.println(bras.isGripperClosed() ? F("Pince fermee") : F("Pince ouverte"));
        btSerial.println(bras.isGripperClosed() ? F("PINCE:FERMEE") : F("PINCE:OUVERTE"));
    }
}

/* --------------------------------------------------------------------------
 * Traitement des commandes (Bluetooth / Serie)
 * -------------------------------------------------------------------------- */

void traiterCommande(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    cmd.toLowerCase();
    nbCommandes++;

    Serial.print(F("CMD #"));
    Serial.print(nbCommandes);
    Serial.print(F(" : "));
    Serial.println(cmd);

    /* --- Commandes du bras robotique --- */

    if (cmd == "attrape" || cmd == "grab") {
        bras.grab();
        btSerial.println(F("OK:GRAB"));
        afficherEtat("Bras", "Prise objet");
    }
    else if (cmd == "lache" || cmd == "release") {
        bras.release();
        btSerial.println(F("OK:RELEASE"));
        afficherEtat("Bras", "Objet lache");
    }
    else if (cmd == "monte" || cmd == "up") {
        bras.goUp();
        btSerial.println(F("OK:UP"));
        afficherEtat("Bras", "Position haute");
    }
    else if (cmd == "descend" || cmd == "down") {
        bras.goDown();
        btSerial.println(F("OK:DOWN"));
        afficherEtat("Bras", "Position basse");
    }
    else if (cmd == "home" || cmd == "repos") {
        bras.goHome();
        btSerial.println(F("OK:HOME"));
        afficherEtat("Bras", "Position repos");
    }
    else if (cmd == "range" || cmd == "stow") {
        bras.stow();
        btSerial.println(F("OK:STOW"));
        afficherEtat("Bras", "Range");
    }
    else if (cmd == "tend" || cmd == "extend") {
        bras.extend();
        btSerial.println(F("OK:EXTEND"));
        afficherEtat("Bras", "Etendu max");
    }
    else if (cmd == "table") {
        // Position pre-enregistree pour atteindre une table
        bras.setBase(90);
        bras.setShoulder(70);
        bras.setElbow(100);
        btSerial.println(F("OK:TABLE"));
        afficherEtat("Bras", "Position table");
    }
    else if (cmd == "sol" || cmd == "floor") {
        // Position pour ramasser au sol
        bras.setBase(90);
        bras.setShoulder(140);
        bras.setElbow(130);
        btSerial.println(F("OK:FLOOR"));
        afficherEtat("Bras", "Position sol");
    }

    /* --- Commandes d'arret et de reprise --- */

    else if (cmd == "stop" || cmd == "arrete") {
        arretUrgence = true;
        moteurs.stop();
        vitesseGauche = 0;
        vitesseDroite = 0;
        btSerial.println(F("OK:STOP"));
        afficherEtat("URGENCE", "Arret logiciel");
        bipAlerte(300);
    }
    else if (cmd == "go" || cmd == "resume" || cmd == "reprend") {
        arretUrgence = false;
        obstacleDetecte = false;
        btSerial.println(F("OK:GO"));
        afficherEtat("Robot", "Reprise");
        digitalWrite(PIN_LED_URGENCE, LOW);
    }

    /* --- Commandes de deplacement (mode bluetooth) --- */

    else if (cmd == "avance" || cmd == "forward") {
        if (!arretUrgence) {
            moteurs.drive(VITESSE_MAX / 2, true, VITESSE_MAX / 2, true);
            vitesseGauche = VITESSE_MAX / 2;
            vitesseDroite = VITESSE_MAX / 2;
            btSerial.println(F("OK:FORWARD"));
        }
    }
    else if (cmd == "recule" || cmd == "backward") {
        if (!arretUrgence) {
            moteurs.drive(VITESSE_MAX / 2, false, VITESSE_MAX / 2, false);
            vitesseGauche = VITESSE_MAX / 2;
            vitesseDroite = VITESSE_MAX / 2;
            btSerial.println(F("OK:BACKWARD"));
        }
    }
    else if (cmd == "gauche" || cmd == "left") {
        if (!arretUrgence) {
            moteurs.drive(VITESSE_MAX / 3, false, VITESSE_MAX / 3, true);
            btSerial.println(F("OK:LEFT"));
        }
    }
    else if (cmd == "droite" || cmd == "right") {
        if (!arretUrgence) {
            moteurs.drive(VITESSE_MAX / 3, true, VITESSE_MAX / 3, false);
            btSerial.println(F("OK:RIGHT"));
        }
    }
    else if (cmd == "freine" || cmd == "brake") {
        moteurs.brake();
        vitesseGauche = 0;
        vitesseDroite = 0;
        btSerial.println(F("OK:BRAKE"));
    }

    /* --- Commandes articulaires directes (angle en degres) --- */

    else if (cmd.startsWith("base:")) {
        int angle = cmd.substring(5).toInt();
        bras.setBase(angle);
        btSerial.println("OK:BASE:" + String(angle));
    }
    else if (cmd.startsWith("epaule:")) {
        int angle = cmd.substring(7).toInt();
        bras.setShoulder(angle);
        btSerial.println("OK:EPAULE:" + String(angle));
    }
    else if (cmd.startsWith("coude:")) {
        int angle = cmd.substring(6).toInt();
        bras.setElbow(angle);
        btSerial.println("OK:COUDE:" + String(angle));
    }
    else if (cmd.startsWith("pince:")) {
        int angle = cmd.substring(6).toInt();
        bras.setGripper(angle);
        btSerial.println("OK:PINCE:" + String(angle));
    }
    else if (cmd.startsWith("vitesse:") || cmd.startsWith("speed:")) {
        int idx = cmd.indexOf(':');
        int v = cmd.substring(idx + 1).toInt();
        bras.setSpeed(v);
        btSerial.println("OK:SPEED:" + String(v));
    }
    else if (cmd.startsWith("pwm:")) {
        // Commande directe de vitesse moteur (0-255)
        int v = constrain(cmd.substring(4).toInt(), 0, 255);
        moteurs.drive(v, true, v, true);
        vitesseGauche = v;
        vitesseDroite = v;
        btSerial.println("OK:PWM:" + String(v));
    }

    /* --- Changement de mode de controle --- */

    else if (cmd == "mode:joystick" || cmd == "mode:joy") {
        modeActuel = 0;
        btSerial.println(F("OK:MODE:JOYSTICK"));
        afficherEtat("Mode", "Joystick");
    }
    else if (cmd == "mode:bt" || cmd == "mode:bluetooth") {
        modeActuel = 1;
        moteurs.stop();
        vitesseGauche = 0;
        vitesseDroite = 0;
        btSerial.println(F("OK:MODE:BT"));
        afficherEtat("Mode", "Bluetooth");
    }
    else if (cmd == "mode:vocal" || cmd == "mode:voice") {
        modeActuel = 2;
        btSerial.println(F("OK:MODE:VOCAL"));
        afficherEtat("Mode", "Commande vocale");
    }

    /* --- Requetes d'information --- */

    else if (cmd == "status" || cmd == "etat") {
        envoyerStatutComplet();
    }
    else if (cmd == "distances" || cmd == "dist") {
        envoyerDistances();
    }
    else if (cmd == "angles") {
        envoyerAngles();
    }
    else if (cmd == "help" || cmd == "aide") {
        envoyerAide();
    }
    else if (cmd == "version") {
        btSerial.println(F("Robot PMR v2.0 -- ENSMM Besancon"));
        btSerial.println(F("Arduino Mega 2560"));
    }
    else if (cmd == "reset") {
        // Reset logiciel (pas de reset materiel possible en code)
        arretUrgence = false;
        obstacleDetecte = false;
        modeActuel = 0;
        moteurs.stop();
        bras.goHome();
        vitesseGauche = 0;
        vitesseDroite = 0;
        nbCommandes = 0;
        btSerial.println(F("OK:RESET"));
        afficherEtat("Robot PMR", "Reset OK");
    }

    /* --- Commande non reconnue --- */

    else {
        btSerial.println("ERR:INCONNU:" + cmd);
        Serial.print(F("  -> commande inconnue : "));
        Serial.println(cmd);
    }
}

/* --------------------------------------------------------------------------
 * Detection et filtrage d'obstacles
 * -------------------------------------------------------------------------- */

void verifierObstacles() {
    // Lecture cyclique : un capteur par cycle (evite les interferences entre capteurs)
    static int capteurActuel = 0;

    switch (capteurActuel) {
        case 0: distFront = obstacles.getDistanceFiltered(FRONT); break;
        case 1: distBack  = obstacles.getDistanceFiltered(BACK);  break;
        case 2: distLeft  = obstacles.getDistanceFiltered(LEFT);  break;
        case 3: distRight = obstacles.getDistanceFiltered(RIGHT); break;
    }
    capteurActuel = (capteurActuel + 1) % 4;

    // --- Obstacle frontal : le plus critique ---
    if (distFront > 0 && distFront < DISTANCE_SECURITE) {
        if (!obstacleDetecte) {
            obstacleDetecte = true;
            moteurs.stop();
            vitesseGauche = 0;
            vitesseDroite = 0;

            Serial.print(F("OBSTACLE FRONTAL : "));
            Serial.print(distFront, 1);
            Serial.println(F(" cm"));

            btSerial.println("OBSTACLE:FRONT:" + String(distFront, 0));
            afficherEtat("Obstacle!", String(distFront, 0) + " cm devant");
        }
    }

    // --- Obstacle arriere : alerte Bluetooth ---
    if (distBack > 0 && distBack < DISTANCE_SECURITE) {
        btSerial.println("ALERTE:BACK:" + String(distBack, 0));
    }

    // --- Obstacles lateraux : alerte seulement ---
    if (distLeft > 0 && distLeft < DISTANCE_LATERALE) {
        btSerial.println("ALERTE:LEFT:" + String(distLeft, 0));
    }
    if (distRight > 0 && distRight < DISTANCE_LATERALE) {
        btSerial.println("ALERTE:RIGHT:" + String(distRight, 0));
    }

    // --- L'obstacle frontal s'est eloigne ---
    if (obstacleDetecte && (distFront < 0 || distFront > DISTANCE_ALERTE)) {
        obstacleDetecte = false;
        btSerial.println(F("OBSTACLE:DEGAGE"));
        Serial.println(F("Obstacle degage"));
    }
}

/* --------------------------------------------------------------------------
 * Affichage OLED
 * -------------------------------------------------------------------------- */

void afficherDemarrage() {
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(10, 2);
    oled.println(F("Robot PMR"));
    oled.setTextSize(1);
    oled.setCursor(8, 25);
    oled.println(F("Initialisation..."));
    oled.drawLine(0, 40, 128, 40, WHITE);
    oled.setCursor(12, 48);
    oled.println(F("ENSMM Besancon"));
    oled.setCursor(16, 56);
    oled.println(F("Hind Jabrane"));
    oled.display();
}

void afficherEtat(String ligne1, String ligne2) {
    oled.clearDisplay();

    // Barre de titre
    oled.fillRect(0, 0, 128, 16, WHITE);
    oled.setTextSize(1);
    oled.setTextColor(BLACK);
    oled.setCursor(2, 4);
    oled.print(ligne1);

    // Indicateur de mode en haut a droite
    oled.setCursor(100, 4);
    switch (modeActuel) {
        case 0: oled.print(F("JOY")); break;
        case 1: oled.print(F("BT"));  break;
        case 2: oled.print(F("VOC")); break;
    }

    // Ligne 2 (message principal)
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 22);
    oled.println(ligne2);

    // Barre de separation
    oled.drawLine(0, 34, 128, 34, WHITE);

    // Distances (bas de l'ecran)
    oled.setCursor(0, 38);
    oled.print(F("F:"));
    oled.print(distFront > 0 ? String(distFront, 0) : "--");
    oled.print(F("  B:"));
    oled.print(distBack > 0 ? String(distBack, 0) : "--");

    oled.setCursor(0, 48);
    oled.print(F("L:"));
    oled.print(distLeft > 0 ? String(distLeft, 0) : "--");
    oled.print(F("  R:"));
    oled.print(distRight > 0 ? String(distRight, 0) : "--");

    // Vitesses moteur
    oled.setCursor(0, 58);
    oled.print(F("VG:"));
    oled.print(vitesseGauche);
    oled.print(F(" VD:"));
    oled.print(vitesseDroite);

    // Indicateur d'urgence
    if (arretUrgence) {
        oled.fillRect(90, 55, 38, 9, WHITE);
        oled.setTextColor(BLACK);
        oled.setCursor(92, 56);
        oled.print(F("STOP"));
        oled.setTextColor(WHITE);
    }

    oled.display();
}

void mettreAJourOLED() {
    if (arretUrgence) {
        afficherEtat("URGENCE", "Envoyer 'go'");
    } else if (obstacleDetecte) {
        afficherEtat("Obstacle!", String(distFront, 0) + " cm devant");
    } else {
        String mode;
        switch (modeActuel) {
            case 0: mode = "Joystick actif"; break;
            case 1: mode = "Bluetooth actif"; break;
            case 2: mode = "Vocal actif"; break;
        }
        afficherEtat("Robot PMR", mode);
    }
}

/* --------------------------------------------------------------------------
 * Communication Bluetooth
 * -------------------------------------------------------------------------- */

void envoyerStatutBT() {
    // Format compact pour l'application de supervision
    // ST:etat,M:mode,VG:xxx,VD:xxx,DF:xxx,DB:xxx,P:0/1
    String s = "ST:";
    s += arretUrgence ? "URG" : (obstacleDetecte ? "OBS" : "OK");
    s += ",M:" + String(modeActuel);
    s += ",VG:" + String(vitesseGauche);
    s += ",VD:" + String(vitesseDroite);
    s += ",DF:" + String(distFront, 0);
    s += ",DB:" + String(distBack, 0);
    s += ",P:" + String(bras.isGripperClosed() ? 1 : 0);
    btSerial.println(s);
}

void envoyerStatutComplet() {
    btSerial.println(F("========== STATUT ROBOT PMR =========="));
    btSerial.print(F("Etat global   : "));
    if (arretUrgence) btSerial.println(F("ARRET URGENCE"));
    else if (obstacleDetecte) btSerial.println(F("OBSTACLE DETECTE"));
    else btSerial.println(F("OPERATIONNEL"));

    btSerial.print(F("Mode controle : "));
    switch (modeActuel) {
        case 0: btSerial.println(F("Joystick")); break;
        case 1: btSerial.println(F("Bluetooth")); break;
        case 2: btSerial.println(F("Vocal")); break;
    }
    btSerial.print(F("Vitesse G/D   : "));
    btSerial.print(vitesseGauche);
    btSerial.print(F(" / "));
    btSerial.println(vitesseDroite);
    btSerial.print(F("Pince         : "));
    btSerial.println(bras.isGripperClosed() ? F("FERMEE") : F("OUVERTE"));
    envoyerDistances();
    envoyerAngles();
    btSerial.print(F("Commandes     : "));
    btSerial.println(nbCommandes);
    btSerial.print(F("Uptime        : "));
    btSerial.print(millis() / 1000);
    btSerial.println(F(" s"));
    btSerial.println(F("======================================="));
}

void envoyerDistances() {
    btSerial.print(F("DIST avant="));
    btSerial.print(distFront, 1);
    btSerial.print(F(" arriere="));
    btSerial.print(distBack, 1);
    btSerial.print(F(" gauche="));
    btSerial.print(distLeft, 1);
    btSerial.print(F(" droite="));
    btSerial.println(distRight, 1);
}

void envoyerAngles() {
    btSerial.println("ANGLES:" + bras.getAnglesString());
}

void envoyerAide() {
    btSerial.println(F("=== COMMANDES ROBOT PMR ==="));
    btSerial.println(F("[Bras]"));
    btSerial.println(F("  attrape, lache, monte, descend"));
    btSerial.println(F("  home, range, tend, table, sol"));
    btSerial.println(F("[Deplacement]"));
    btSerial.println(F("  avance, recule, gauche, droite"));
    btSerial.println(F("  stop, go, freine"));
    btSerial.println(F("[Articulations]"));
    btSerial.println(F("  base:X epaule:X coude:X pince:X"));
    btSerial.println(F("  vitesse:X (ms/deg, 5-30)"));
    btSerial.println(F("[Mode]"));
    btSerial.println(F("  mode:joystick, mode:bt, mode:vocal"));
    btSerial.println(F("[Info]"));
    btSerial.println(F("  status, distances, angles"));
    btSerial.println(F("  version, reset, help"));
}

/* --------------------------------------------------------------------------
 * Utilitaires
 * -------------------------------------------------------------------------- */

void bipAlerte(int duree) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(duree);
    digitalWrite(PIN_BUZZER, LOW);
}
