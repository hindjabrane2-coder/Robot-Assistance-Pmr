#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// --- Pins L298N (par defaut, modifiables via constructeur) ---
#define MOTOR_LEFT_EN     5    // PWM enable gauche
#define MOTOR_LEFT_IN1    4
#define MOTOR_LEFT_IN2    3
#define MOTOR_RIGHT_EN    6    // PWM enable droite
#define MOTOR_RIGHT_IN1   7
#define MOTOR_RIGHT_IN2   8

// --- Parametres moteur ---
#define PWM_MIN_EFFECTIVE   45   // en dessous de ca les moteurs ne tournent pas
#define PWM_MAX             255
#define RAMP_STEP           5    // increment PWM par tick de rampe
#define RAMP_INTERVAL_MS    20   // temps entre deux increments
#define MOTOR_TIMEOUT_MS    3000 // arret auto si aucune commande pendant 3s

// --- Odometrie basique ---
// Vitesse lineaire approximative a PWM max (cm/s), mesure experimentale
#define MAX_SPEED_CM_S      30.0
// Entraxe des roues (cm), pour estimation de rotation
#define WHEEL_BASE_CM       28.0


class MotorDriver {
private:
    int pinEnL, pinIn1L, pinIn2L;
    int pinEnR, pinIn1R, pinIn2R;

    // Vitesses actuelles et cibles (PWM 0-255)
    int currentPwmL, currentPwmR;
    int targetPwmL, targetPwmR;

    // Direction actuelle par moteur (true = avant)
    bool dirForwardL, dirForwardR;

    // Facteur de mise a l'echelle (0.0 a 1.0) pour limiter la vitesse max
    float speedScale;

    // Rampe
    unsigned long lastRampTime;

    // Timeout securite
    unsigned long lastCommandTime;
    bool timeoutEnabled;
    bool timedOut;

    // Odometrie
    float odometryX;      // cm, position estimee
    float odometryY;
    float odometryTheta;  // radians
    unsigned long lastOdoTime;

    // Appliquer la direction sur les pins IN1/IN2
    void applyDirection(int in1, int in2, bool forward) {
        digitalWrite(in1, forward ? HIGH : LOW);
        digitalWrite(in2, forward ? LOW : HIGH);
    }

    // Ecrire les PWM sur les pins enable
    void writePwm() {
        // Gerer la zone morte : si la cible est > 0 mais < seuil, on met le seuil
        int pwmL = currentPwmL;
        int pwmR = currentPwmR;

        if (pwmL > 0 && pwmL < PWM_MIN_EFFECTIVE) pwmL = PWM_MIN_EFFECTIVE;
        if (pwmR > 0 && pwmR < PWM_MIN_EFFECTIVE) pwmR = PWM_MIN_EFFECTIVE;

        analogWrite(pinEnL, pwmL);
        analogWrite(pinEnR, pwmR);
    }

    // Convertir une vitesse -255..255 en PWM + direction pour un moteur
    void decomposeSpeed(int speed, int &pwm, bool &forward) {
        if (speed >= 0) {
            forward = true;
            pwm = min(speed, (int)PWM_MAX);
        } else {
            forward = false;
            pwm = min(-speed, (int)PWM_MAX);
        }
        // Appliquer le facteur d'echelle
        pwm = (int)(pwm * speedScale);
    }

public:
    MotorDriver(int enL, int in1L, int in2L, int enR, int in1R, int in2R)
        : pinEnL(enL), pinIn1L(in1L), pinIn2L(in2L),
          pinEnR(enR), pinIn1R(in1R), pinIn2R(in2R)
    {
        currentPwmL = 0; currentPwmR = 0;
        targetPwmL = 0;  targetPwmR = 0;
        dirForwardL = true; dirForwardR = true;
        speedScale = 1.0;
        lastRampTime = 0;
        lastCommandTime = 0;
        timeoutEnabled = true;
        timedOut = false;
        odometryX = 0; odometryY = 0; odometryTheta = 0;
        lastOdoTime = 0;
    }

    void begin() {
        pinMode(pinEnL, OUTPUT);
        pinMode(pinIn1L, OUTPUT);
        pinMode(pinIn2L, OUTPUT);
        pinMode(pinEnR, OUTPUT);
        pinMode(pinIn1R, OUTPUT);
        pinMode(pinIn2R, OUTPUT);
        stop();
        lastCommandTime = millis();
        lastOdoTime = millis();
    }

    // --- Configuration ---
    void setSpeedScale(float scale) { speedScale = constrain(scale, 0.0, 1.0); }
    float getSpeedScale() { return speedScale; }
    void enableTimeout(bool en) { timeoutEnabled = en; }

    // --- Commandes bas niveau ---

    // Definir la cible pour chaque moteur (valeurs signees, -255 a 255)
    void setMotors(int leftSpeed, int rightSpeed) {
        lastCommandTime = millis();
        timedOut = false;

        int pwmL, pwmR;
        bool fwdL, fwdR;
        decomposeSpeed(leftSpeed, pwmL, fwdL);
        decomposeSpeed(rightSpeed, pwmR, fwdR);

        targetPwmL = pwmL; dirForwardL = fwdL;
        targetPwmR = pwmR; dirForwardR = fwdR;

        applyDirection(pinIn1L, pinIn2L, dirForwardL);
        applyDirection(pinIn1R, pinIn2R, dirForwardR);
    }

    // Arret immediat sans rampe
    void stop() {
        targetPwmL = 0; targetPwmR = 0;
        currentPwmL = 0; currentPwmR = 0;
        analogWrite(pinEnL, 0);
        analogWrite(pinEnR, 0);
    }

    // Freinage d'urgence : court-circuit des moteurs via L298N
    void emergencyBrake() {
        targetPwmL = 0; targetPwmR = 0;
        currentPwmL = 0; currentPwmR = 0;

        // Les deux IN a HIGH = freinage par court-circuit
        digitalWrite(pinIn1L, HIGH); digitalWrite(pinIn2L, HIGH);
        digitalWrite(pinIn1R, HIGH); digitalWrite(pinIn2R, HIGH);
        analogWrite(pinEnL, PWM_MAX);
        analogWrite(pinEnR, PWM_MAX);
        delay(100);
        // Puis couper
        analogWrite(pinEnL, 0);
        analogWrite(pinEnR, 0);
        digitalWrite(pinIn1L, LOW); digitalWrite(pinIn2L, LOW);
        digitalWrite(pinIn1R, LOW); digitalWrite(pinIn2R, LOW);
    }

    // --- Commandes de deplacement ---

    void forward(int speed) {
        speed = abs(speed);
        setMotors(speed, speed);
    }

    void backward(int speed) {
        speed = abs(speed);
        setMotors(-speed, -speed);
    }

    void turnLeft(int speed) {
        speed = abs(speed);
        setMotors(speed / 3, speed);
    }

    void turnRight(int speed) {
        speed = abs(speed);
        setMotors(speed, speed / 3);
    }

    // Rotation sur place
    void spinLeft(int speed) {
        speed = abs(speed);
        setMotors(-speed, speed);
    }

    void spinRight(int speed) {
        speed = abs(speed);
        setMotors(speed, -speed);
    }

    // Virage en arc : rayon en cm (positif = droite, negatif = gauche)
    void arc(int speed, float radiusCm) {
        if (abs(radiusCm) < 1.0) {
            // Rayon trop petit, rotation sur place
            if (radiusCm >= 0) spinRight(speed);
            else spinLeft(speed);
            return;
        }

        float halfBase = WHEEL_BASE_CM / 2.0;
        float ratioInner = (abs(radiusCm) - halfBase) / (abs(radiusCm) + halfBase);
        ratioInner = constrain(ratioInner, -1.0, 1.0);

        int outerPwm = abs(speed);
        int innerPwm = (int)(outerPwm * ratioInner);

        if (radiusCm > 0) {
            setMotors(outerPwm, innerPwm);  // droite = interieur
        } else {
            setMotors(innerPwm, outerPwm);  // gauche = interieur
        }
    }

    // Commande en vitesse lineaire (cm/s) + angulaire (rad/s)
    // Modele differentiel classique
    void driveVelocity(float linearCmS, float angularRadS) {
        lastCommandTime = millis();
        timedOut = false;

        // v_left  = linear - angular * L/2
        // v_right = linear + angular * L/2
        float vL = linearCmS - angularRadS * (WHEEL_BASE_CM / 2.0);
        float vR = linearCmS + angularRadS * (WHEEL_BASE_CM / 2.0);

        // Convertir cm/s en PWM (proportionnel)
        int pwmL = (int)(vL / MAX_SPEED_CM_S * PWM_MAX);
        int pwmR = (int)(vR / MAX_SPEED_CM_S * PWM_MAX);

        pwmL = constrain(pwmL, -PWM_MAX, PWM_MAX);
        pwmR = constrain(pwmR, -PWM_MAX, PWM_MAX);

        setMotors(pwmL, pwmR);
    }

    // --- Mise a jour (appeler dans loop) ---
    // Gere la rampe d'acceleration et le timeout
    void update() {
        unsigned long now = millis();

        // Timeout de securite
        if (timeoutEnabled && (now - lastCommandTime > MOTOR_TIMEOUT_MS)) {
            if (!timedOut) {
                stop();
                timedOut = true;
            }
            return;
        }

        // Rampe d'acceleration/deceleration
        if (now - lastRampTime >= RAMP_INTERVAL_MS) {
            lastRampTime = now;

            if (currentPwmL < targetPwmL) {
                currentPwmL = min(currentPwmL + RAMP_STEP, targetPwmL);
            } else if (currentPwmL > targetPwmL) {
                currentPwmL = max(currentPwmL - RAMP_STEP, targetPwmL);
            }

            if (currentPwmR < targetPwmR) {
                currentPwmR = min(currentPwmR + RAMP_STEP, targetPwmR);
            } else if (currentPwmR > targetPwmR) {
                currentPwmR = max(currentPwmR - RAMP_STEP, targetPwmR);
            }

            writePwm();
        }

        // Mise a jour odometrie
        updateOdometry(now);
    }

    // --- Odometrie ---
private:
    void updateOdometry(unsigned long now) {
        float dt = (now - lastOdoTime) / 1000.0; // secondes
        lastOdoTime = now;
        if (dt <= 0 || dt > 0.5) return; // dt aberrant, on saute

        // Estimer la vitesse de chaque roue en cm/s a partir du PWM actuel
        float vL = (currentPwmL / (float)PWM_MAX) * MAX_SPEED_CM_S;
        float vR = (currentPwmR / (float)PWM_MAX) * MAX_SPEED_CM_S;
        if (!dirForwardL) vL = -vL;
        if (!dirForwardR) vR = -vR;

        float linear  = (vL + vR) / 2.0;
        float angular = (vR - vL) / WHEEL_BASE_CM;

        odometryTheta += angular * dt;
        odometryX += linear * dt * cos(odometryTheta);
        odometryY += linear * dt * sin(odometryTheta);
    }

public:
    void resetOdometry() { odometryX = 0; odometryY = 0; odometryTheta = 0; }
    float getOdoX()     { return odometryX; }
    float getOdoY()     { return odometryY; }
    float getOdoTheta() { return odometryTheta; }

    // Distance totale parcourue depuis le dernier reset
    float getOdoDistance() {
        return sqrt(odometryX * odometryX + odometryY * odometryY);
    }

    // --- Etat ---
    bool isTimedOut() { return timedOut; }
    bool isMoving()   { return (currentPwmL > 0 || currentPwmR > 0); }
    int getCurrentPwmLeft()  { return currentPwmL; }
    int getCurrentPwmRight() { return currentPwmR; }

    String getStatusString() {
        String s = "Moteurs: ";
        s += "G:" + String(dirForwardL ? "+" : "-") + String(currentPwmL);
        s += "/" + String(targetPwmL);
        s += " D:" + String(dirForwardR ? "+" : "-") + String(currentPwmR);
        s += "/" + String(targetPwmR);
        s += " | Odo: " + String(odometryX, 1) + "," + String(odometryY, 1);
        s += " a=" + String(odometryTheta * 180.0 / PI, 0) + "deg";
        if (timedOut) s += " [TIMEOUT]";
        return s;
    }
};

#endif
