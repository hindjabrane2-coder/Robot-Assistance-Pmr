#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

//  Pins par defaut (modifiables via constructeur) 
#define ARM_DEFAULT_PIN_BASE      9
#define ARM_DEFAULT_PIN_SHOULDER  10
#define ARM_DEFAULT_PIN_ELBOW     11
#define ARM_DEFAULT_PIN_GRIPPER   12
#define ARM_GRIPPER_FORCE_PIN     A0   // capteur de force dans la pince

//  Limites angulaires par articulation (degres) 
#define BASE_MIN       0
#define BASE_MAX       180
#define SHOULDER_MIN   25
#define SHOULDER_MAX   155
#define ELBOW_MIN      20
#define ELBOW_MAX      160
#define GRIPPER_MIN    15    // ouvert a fond
#define GRIPPER_MAX    140   // ferme a fond

// Vitesses de deplacement (ms par degre) 
#define ARM_SPEED_SLOW    30
#define ARM_SPEED_NORMAL  15
#define ARM_SPEED_FAST    6

// Pince 
#define GRIPPER_OPEN_ANGLE    20
#define GRIPPER_CLOSED_ANGLE  130
#define GRIPPER_FORCE_THRESHOLD  400  // seuil analogique = objet detecte
#define GRIPPER_FORCE_MAX        800  // au-dela on arrete pour pas casser

// Securite 
#define ARM_MOVE_TIMEOUT_MS  5000  // si un mouvement prend plus de 5s, on arrete

// Position d'une articulation : 4 angles + etat pince
struct ArmPosition {
    int base;
    int shoulder;
    int elbow;
    int gripper;
};

// Positions predefinies
// Chacune a ete mesuree sur le robot reel
static const ArmPosition POS_HOME       = {  90,  90,  90,  GRIPPER_OPEN_ANGLE };
static const ArmPosition POS_REST       = {  90, 140, 150,  GRIPPER_OPEN_ANGLE };
static const ArmPosition POS_PICK_HIGH  = {  90,  50,  45,  GRIPPER_OPEN_ANGLE };
static const ArmPosition POS_PICK_LOW   = {  90, 130, 110,  GRIPPER_OPEN_ANGLE };
static const ArmPosition POS_HANDOVER   = {  45,  60,  70,  GRIPPER_OPEN_ANGLE };
static const ArmPosition POS_TRANSPORT  = {  90,  70,  60,  GRIPPER_CLOSED_ANGLE };
static const ArmPosition POS_SCAN_LEFT  = {  30,  80,  80,  GRIPPER_OPEN_ANGLE };
static const ArmPosition POS_SCAN_RIGHT = { 150,  80,  80,  GRIPPER_OPEN_ANGLE };
static const ArmPosition POS_COMPACT    = {  90, 150, 155,  GRIPPER_CLOSED_ANGLE }; // rangement max

enum ArmState {
    ARM_IDLE,
    ARM_MOVING,
    ARM_HOLDING,       // pince fermee sur un objet
    ARM_ERROR,
    ARM_EMERGENCY_STOP
};


class ArmController {
private:
    Servo servoBase;
    Servo servoShoulder;
    Servo servoElbow;
    Servo servoGripper;

    int pinBase, pinShoulder, pinElbow, pinGripper;
    int pinForce;

    // Angles actuels
    int curBase;
    int curShoulder;
    int curElbow;
    int curGripper;

    int moveSpeed;      // ms par degre
    ArmState state;
    bool gripperHolding; // objet dans la pince
    bool stopped;        // arret d'urgence actif

    // Contraint un angle dans les bornes d'une articulation
    int clampBase(int a)     { return constrain(a, BASE_MIN, BASE_MAX); }
    int clampShoulder(int a) { return constrain(a, SHOULDER_MIN, SHOULDER_MAX); }
    int clampElbow(int a)    { return constrain(a, ELBOW_MIN, ELBOW_MAX); }
    int clampGripper(int a)  { return constrain(a, GRIPPER_MIN, GRIPPER_MAX); }

    // Mouvement lisse d'un seul servo
    // Retourne false si arret d'urgence pendant le mouvement
    bool smoothMove(Servo &srv, int &current, int target, int speedMs) {
        if (stopped) return false;

        int direction = (target > current) ? 1 : -1;
        unsigned long startTime = millis();

        while (current != target) {
            if (stopped) return false;

            // timeout de securite
            if (millis() - startTime > ARM_MOVE_TIMEOUT_MS) {
                state = ARM_ERROR;
                return false;
            }

            current += direction;
            srv.write(current);
            delay(speedMs);
        }
        return true;
    }

    // Lecture du capteur de force (pince)
    int readGripperForce() {
        return analogRead(pinForce);
    }

public:
    ArmController(int bPin, int sPin, int ePin, int gPin, int fPin = ARM_GRIPPER_FORCE_PIN)
        : pinBase(bPin), pinShoulder(sPin), pinElbow(ePin), pinGripper(gPin), pinForce(fPin)
    {
        curBase = 90;
        curShoulder = 90;
        curElbow = 90;
        curGripper = GRIPPER_OPEN_ANGLE;
        moveSpeed = ARM_SPEED_NORMAL;
        state = ARM_IDLE;
        gripperHolding = false;
        stopped = false;
    }

    void begin() {
        servoBase.attach(pinBase);
        servoShoulder.attach(pinShoulder);
        servoElbow.attach(pinElbow);
        servoGripper.attach(pinGripper);
        pinMode(pinForce, INPUT);

        // Position initiale sans interpolation
        servoBase.write(curBase);
        servoShoulder.write(curShoulder);
        servoElbow.write(curElbow);
        servoGripper.write(curGripper);
        delay(500);
        state = ARM_IDLE;
    }

    // --- Configuration ---
    void setSpeed(int msPerDegree) { moveSpeed = constrain(msPerDegree, 2, 50); }
    void setSpeedSlow()   { moveSpeed = ARM_SPEED_SLOW; }
    void setSpeedNormal() { moveSpeed = ARM_SPEED_NORMAL; }
    void setSpeedFast()   { moveSpeed = ARM_SPEED_FAST; }

    // Arret d'urgence
    void emergencyStop() {
        stopped = true;
        state = ARM_EMERGENCY_STOP;
        // on ne detache pas les servos, juste on arrete tout mouvement
    }

    void resetEmergency() {
        stopped = false;
        state = ARM_IDLE;
    }

    bool isStopped() { return stopped; }

    // Mouvement individuel par articulation
    bool moveBase(int angle) {
        if (stopped) return false;
        state = ARM_MOVING;
        angle = clampBase(angle);
        bool ok = smoothMove(servoBase, curBase, angle, moveSpeed);
        if (ok) state = gripperHolding ? ARM_HOLDING : ARM_IDLE;
        return ok;
    }

    bool moveShoulder(int angle) {
        if (stopped) return false;
        state = ARM_MOVING;
        angle = clampShoulder(angle);
        bool ok = smoothMove(servoShoulder, curShoulder, angle, moveSpeed);
        if (ok) state = gripperHolding ? ARM_HOLDING : ARM_IDLE;
        return ok;
    }

    bool moveElbow(int angle) {
        if (stopped) return false;
        state = ARM_MOVING;
        angle = clampElbow(angle);
        bool ok = smoothMove(servoElbow, curElbow, angle, moveSpeed);
        if (ok) state = gripperHolding ? ARM_HOLDING : ARM_IDLE;
        return ok;
    }

    // Mouvement interpole vers une position cible 
    // Deplace toutes les articulations en parallele (pas de vrai parallele
    // sur Arduino, mais on alterne les pas pour un mouvement coordonne)
    bool moveTo(const ArmPosition &target) {
        if (stopped) return false;
        state = ARM_MOVING;

        int tBase     = clampBase(target.base);
        int tShoulder = clampShoulder(target.shoulder);
        int tElbow    = clampElbow(target.elbow);
        int tGripper  = clampGripper(target.gripper);

        // Calculer le nombre de pas = plus grand ecart
        int dB = abs(tBase - curBase);
        int dS = abs(tShoulder - curShoulder);
        int dE = abs(tElbow - curElbow);
        int dG = abs(tGripper - curGripper);
        int maxSteps = max(max(dB, dS), max(dE, dG));

        if (maxSteps == 0) {
            state = gripperHolding ? ARM_HOLDING : ARM_IDLE;
            return true;
        }

        // Direction de chaque axe
        int dirB = (tBase > curBase) ? 1 : (tBase < curBase) ? -1 : 0;
        int dirS = (tShoulder > curShoulder) ? 1 : (tShoulder < curShoulder) ? -1 : 0;
        int dirE = (tElbow > curElbow) ? 1 : (tElbow < curElbow) ? -1 : 0;
        int dirG = (tGripper > curGripper) ? 1 : (tGripper < curGripper) ? -1 : 0;

        // Accumulateurs pour interpolation lineaire (Bresenham-like)
        int accB = 0, accS = 0, accE = 0, accG = 0;
        unsigned long startTime = millis();

        for (int step = 0; step < maxSteps; step++) {
            if (stopped) return false;
            if (millis() - startTime > ARM_MOVE_TIMEOUT_MS) {
                state = ARM_ERROR;
                return false;
            }

            accB += dB;
            if (accB >= maxSteps) { accB -= maxSteps; curBase += dirB; servoBase.write(curBase); }

            accS += dS;
            if (accS >= maxSteps) { accS -= maxSteps; curShoulder += dirS; servoShoulder.write(curShoulder); }

            accE += dE;
            if (accE >= maxSteps) { accE -= maxSteps; curElbow += dirE; servoElbow.write(curElbow); }

            accG += dG;
            if (accG >= maxSteps) { accG -= maxSteps; curGripper += dirG; servoGripper.write(curGripper); }

            delay(moveSpeed);
        }

        // S'assurer qu'on est bien a la cible
        curBase = tBase;       servoBase.write(curBase);
        curShoulder = tShoulder; servoShoulder.write(curShoulder);
        curElbow = tElbow;     servoElbow.write(curElbow);
        curGripper = tGripper; servoGripper.write(curGripper);

        state = gripperHolding ? ARM_HOLDING : ARM_IDLE;
        return true;
    }

    // Positions nommees 
    bool goHome()      { gripperHolding = false; return moveTo(POS_HOME); }
    bool goRest()      { gripperHolding = false; return moveTo(POS_REST); }
    bool goCompact()   { return moveTo(POS_COMPACT); }
    bool goTransport() { return moveTo(POS_TRANSPORT); }

    // Pince 
    // Ferme progressivement jusqu'a detecter un objet ou atteindre la limite
    bool gripperClose() {
        if (stopped) return false;
        state = ARM_MOVING;

        int target = GRIPPER_CLOSED_ANGLE;
        int direction = (target > curGripper) ? 1 : -1;

        while (curGripper != target) {
            if (stopped) return false;

            int force = readGripperForce();
            if (force > GRIPPER_FORCE_MAX) {
                // trop de force, on recule un peu
                curGripper -= direction * 3;
                servoGripper.write(curGripper);
                break;
            }
            if (force > GRIPPER_FORCE_THRESHOLD) {
                // on tient quelque chose
                gripperHolding = true;
                state = ARM_HOLDING;
                return true;
            }

            curGripper += direction;
            servoGripper.write(curGripper);
            delay(moveSpeed + 5); // un peu plus lent pour la pince
        }

        // Si on arrive la c'est qu'on a ferme sans rien detecter
        gripperHolding = false;
        state = ARM_IDLE;
        return true;
    }

    bool gripperOpen() {
        if (stopped) return false;
        gripperHolding = false;
        state = ARM_MOVING;
        bool ok = smoothMove(servoGripper, curGripper, GRIPPER_OPEN_ANGLE, moveSpeed);
        if (ok) state = ARM_IDLE;
        return ok;
    }

    bool isHoldingObject() { return gripperHolding; }

    // Sequences completes 

    // Ramasser un objet au sol devant le robot
    bool pickObject() {
        if (stopped) return false;
        if (!gripperOpen()) return false;
        if (!moveTo(POS_PICK_LOW)) return false;
        delay(200);
        if (!gripperClose()) return false;
        if (!gripperHolding) return false; // rien attrape
        if (!moveTo(POS_TRANSPORT)) return false;
        return true;
    }

    // Ramasser un objet en hauteur (table, etagere basse)
    bool pickObjectHigh() {
        if (stopped) return false;
        if (!gripperOpen()) return false;
        if (!moveTo(POS_PICK_HIGH)) return false;
        delay(200);
        if (!gripperClose()) return false;
        if (!gripperHolding) return false;
        if (!moveTo(POS_TRANSPORT)) return false;
        return true;
    }

    // Donner l'objet a l'utilisateur
    bool handoverToUser() {
        if (stopped) return false;
        if (!gripperHolding) return false;
        if (!moveTo(POS_HANDOVER)) return false;
        delay(1500);  // laisser le temps a l'utilisateur de prendre
        gripperOpen();
        delay(500);
        goHome();
        return true;
    }

    // Poser l'objet au sol
    bool placeObject() {
        if (stopped) return false;
        if (!gripperHolding) return false;
        if (!moveTo(POS_PICK_LOW)) return false;
        delay(200);
        gripperOpen();
        delay(300);
        goHome();
        return true;
    }

    // Calibration 
    // Parcourt lentement les limites de chaque axe
    void calibrate() {
        int savedSpeed = moveSpeed;
        moveSpeed = ARM_SPEED_SLOW;

        smoothMove(servoBase, curBase, BASE_MIN, moveSpeed);
        delay(300);
        smoothMove(servoBase, curBase, BASE_MAX, moveSpeed);
        delay(300);
        smoothMove(servoBase, curBase, 90, moveSpeed);

        smoothMove(servoShoulder, curShoulder, SHOULDER_MIN, moveSpeed);
        delay(300);
        smoothMove(servoShoulder, curShoulder, SHOULDER_MAX, moveSpeed);
        delay(300);
        smoothMove(servoShoulder, curShoulder, 90, moveSpeed);

        smoothMove(servoElbow, curElbow, ELBOW_MIN, moveSpeed);
        delay(300);
        smoothMove(servoElbow, curElbow, ELBOW_MAX, moveSpeed);
        delay(300);
        smoothMove(servoElbow, curElbow, 90, moveSpeed);

        moveSpeed = savedSpeed;
        goHome();
    }

    // Etat courant 
    ArmPosition getCurrentPosition() {
        ArmPosition pos;
        pos.base = curBase;
        pos.shoulder = curShoulder;
        pos.elbow = curElbow;
        pos.gripper = curGripper;
        return pos;
    }

    ArmState getState() { return state; }

    int getGripperForce() { return readGripperForce(); }

    // Renvoie une chaine lisible de l'etat du bras
    String getStatusString() {
        String s = "Bras: ";
        switch (state) {
            case ARM_IDLE:           s += "repos"; break;
            case ARM_MOVING:         s += "en mouvement"; break;
            case ARM_HOLDING:        s += "objet tenu"; break;
            case ARM_ERROR:          s += "ERREUR"; break;
            case ARM_EMERGENCY_STOP: s += "ARRET URGENCE"; break;
        }
        s += " | B:" + String(curBase);
        s += " S:" + String(curShoulder);
        s += " E:" + String(curElbow);
        s += " G:" + String(curGripper);
        s += " F:" + String(readGripperForce());
        return s;
    }
};

#endif
