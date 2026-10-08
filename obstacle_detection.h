#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <Arduino.h>

// --- Pins capteurs ultrason HC-SR04 ---
// Valeurs par defaut, modifiables via begin()
#define US_FRONT_TRIG  22
#define US_FRONT_ECHO  23
#define US_BACK_TRIG   24
#define US_BACK_ECHO   25
#define US_LEFT_TRIG   26
#define US_LEFT_ECHO   27
#define US_RIGHT_TRIG  28
#define US_RIGHT_ECHO  29

// --- Parametres de mesure ---
#define US_TIMEOUT_US          25000   // timeout pulseIn en microsecondes (~4.25m max)
#define US_MIN_DISTANCE_CM     2.0     // en dessous = erreur capteur
#define US_MAX_DISTANCE_CM     400.0   // au-dela = hors portee
#define US_MEDIAN_SAMPLES      3       // nb de lectures pour le filtre median
#define US_DELAY_BETWEEN_MS    5       // delai entre deux pings sur le meme capteur

// --- Filtrage moyenne mobile ---
#define US_HISTORY_SIZE        5       // taille du buffer circulaire par capteur

// --- Seuils de detection (cm) ---
#define DIST_EMERGENCY         10.0    // arret immediat
#define DIST_DANGER            25.0    // vitesse tres reduite
#define DIST_WARNING           50.0    // ralentissement
#define DIST_CAUTION           80.0    // vigilance accrue
#define DIST_CLEAR             120.0   // voie libre

// Seuils directionnels : l'avant est plus sensible que les cotes
#define FRONT_MULTIPLIER       1.0
#define BACK_MULTIPLIER        0.8
#define SIDE_MULTIPLIER        0.6

// --- Detection de changement brusque ---
#define SUDDEN_CHANGE_CM       30.0    // ecart entre 2 mesures = objet apparu vite
#define EMERGENCY_APPEAR_CM    40.0    // objet passe de loin a < DIST_DANGER

// --- Compensation temperature ---
#define US_DEFAULT_TEMP_C      20.0    // temperature ambiante par defaut

// Directions
enum Direction { FRONT = 0, BACK = 1, LEFT = 2, RIGHT = 3 };

// Niveau de danger par direction
enum ThreatLevel {
    THREAT_NONE,
    THREAT_CAUTION,
    THREAT_WARNING,
    THREAT_DANGER,
    THREAT_EMERGENCY
};

// Resultat d'un scan complet
struct ObstacleMap {
    float distance[4];          // distance filtree par direction
    ThreatLevel threat[4];      // niveau de menace par direction
    bool sensorOk[4];           // capteur fonctionnel ?
    bool suddenChange[4];       // changement brusque detecte ?
};


class ObstacleDetector {
private:
    int trigPin[4];
    int echoPin[4];

    // Historique pour moyenne mobile
    float history[4][US_HISTORY_SIZE];
    int histIndex[4];
    bool histFull[4];

    // Derniere valeur filtree par direction
    float lastFiltered[4];

    // Temperature pour compensation vitesse du son
    float temperature;

    // Seuils par direction (appliques avec multiplicateur)
    float thresholdMultiplier[4];

    // Nombre total de mesures echouees par capteur (pour diagnostic)
    unsigned int failCount[4];

    // Mesure brute d'un capteur
    // Retourne la distance en cm, ou -1 si timeout/erreur
    float measureRaw(int idx) {
        int trig = trigPin[idx];
        int echo = echoPin[idx];

        // Sequence standard HC-SR04
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        long duration = pulseIn(echo, HIGH, US_TIMEOUT_US);
        if (duration == 0) {
            failCount[idx]++;
            return -1.0;
        }

        // Vitesse du son compensee en temperature
        // v = 331.3 + 0.606 * T  (m/s)
        float speedOfSound = 331.3 + 0.606 * temperature; // m/s
        float speedCmPerUs = speedOfSound / 10000.0;       // cm/us
        float dist = duration * speedCmPerUs / 2.0;

        if (dist < US_MIN_DISTANCE_CM || dist > US_MAX_DISTANCE_CM) {
            return -1.0;
        }
        return dist;
    }

    // Filtre median sur 3 mesures
    float measureMedian(int idx) {
        float readings[US_MEDIAN_SAMPLES];
        int valid = 0;

        for (int i = 0; i < US_MEDIAN_SAMPLES; i++) {
            float d = measureRaw(idx);
            if (d > 0) {
                readings[valid++] = d;
            }
            if (i < US_MEDIAN_SAMPLES - 1) {
                delay(US_DELAY_BETWEEN_MS);
            }
        }

        if (valid == 0) return -1.0;
        if (valid == 1) return readings[0];
        if (valid == 2) return (readings[0] + readings[1]) / 2.0;

        // Tri pour 3 valeurs (tri a bulles, c'est que 3 elements)
        for (int i = 0; i < valid - 1; i++) {
            for (int j = i + 1; j < valid; j++) {
                if (readings[j] < readings[i]) {
                    float tmp = readings[i];
                    readings[i] = readings[j];
                    readings[j] = tmp;
                }
            }
        }
        return readings[valid / 2]; // median
    }

    // Ajouter une valeur au buffer circulaire et retourner la moyenne
    float pushAndAverage(int idx, float value) {
        history[idx][histIndex[idx]] = value;
        histIndex[idx] = (histIndex[idx] + 1) % US_HISTORY_SIZE;
        if (histIndex[idx] == 0) histFull[idx] = true;

        int count = histFull[idx] ? US_HISTORY_SIZE : histIndex[idx];
        float sum = 0;
        for (int i = 0; i < count; i++) {
            sum += history[idx][i];
        }
        return sum / count;
    }

    // Determiner le niveau de menace pour une direction
    ThreatLevel evaluateThreat(Direction dir, float dist) {
        if (dist < 0) return THREAT_NONE; // pas de mesure = on ne sait pas

        float mult = thresholdMultiplier[dir];
        if (dist < DIST_EMERGENCY * mult) return THREAT_EMERGENCY;
        if (dist < DIST_DANGER * mult)    return THREAT_DANGER;
        if (dist < DIST_WARNING * mult)   return THREAT_WARNING;
        if (dist < DIST_CAUTION * mult)   return THREAT_CAUTION;
        return THREAT_NONE;
    }

public:
    ObstacleDetector() {
        temperature = US_DEFAULT_TEMP_C;
        for (int i = 0; i < 4; i++) {
            histIndex[i] = 0;
            histFull[i] = false;
            lastFiltered[i] = -1.0;
            failCount[i] = 0;
            for (int j = 0; j < US_HISTORY_SIZE; j++) {
                history[i][j] = 0;
            }
        }
        thresholdMultiplier[FRONT] = FRONT_MULTIPLIER;
        thresholdMultiplier[BACK]  = BACK_MULTIPLIER;
        thresholdMultiplier[LEFT]  = SIDE_MULTIPLIER;
        thresholdMultiplier[RIGHT] = SIDE_MULTIPLIER;
    }

    void begin(int ft, int fe, int bt, int be, int lt, int le, int rt, int re) {
        trigPin[FRONT] = ft; echoPin[FRONT] = fe;
        trigPin[BACK]  = bt; echoPin[BACK]  = be;
        trigPin[LEFT]  = lt; echoPin[LEFT]  = le;
        trigPin[RIGHT] = rt; echoPin[RIGHT] = re;

        for (int i = 0; i < 4; i++) {
            pinMode(trigPin[i], OUTPUT);
            pinMode(echoPin[i], INPUT);
            digitalWrite(trigPin[i], LOW);
        }
    }

    // Mettre a jour la temperature (si on a un capteur DHT par exemple)
    void setTemperature(float tempC) {
        temperature = constrain(tempC, -20.0, 50.0);
    }

    // Configurer le multiplicateur de seuil pour une direction
    void setThresholdMultiplier(Direction dir, float mult) {
        thresholdMultiplier[dir] = constrain(mult, 0.2, 3.0);
    }

    // --- Mesures ---

    // Mesure filtree d'une seule direction
    float getFilteredDistance(Direction dir) {
        float raw = measureMedian((int)dir);
        if (raw < 0) return lastFiltered[dir]; // garder la derniere bonne valeur
        float filtered = pushAndAverage((int)dir, raw);
        lastFiltered[dir] = filtered;
        return filtered;
    }

    // Scanner les 4 directions et remplir la carte d'obstacles
    ObstacleMap measureAll() {
        ObstacleMap map;

        for (int i = 0; i < 4; i++) {
            Direction dir = (Direction)i;
            float raw = measureMedian(i);

            if (raw < 0) {
                // Capteur en erreur, utiliser la derniere valeur connue
                map.distance[i] = lastFiltered[i];
                map.sensorOk[i] = false;
            } else {
                float prev = lastFiltered[i];
                float filtered = pushAndAverage(i, raw);
                lastFiltered[i] = filtered;
                map.distance[i] = filtered;
                map.sensorOk[i] = true;

                // Detection de changement brusque
                if (prev > 0 && (prev - filtered) > SUDDEN_CHANGE_CM) {
                    map.suddenChange[i] = true;
                } else {
                    map.suddenChange[i] = false;
                }
            }

            map.threat[i] = evaluateThreat(dir, map.distance[i]);
        }

        return map;
    }

    // La voie est-elle libre dans une direction ?
    bool isPathClear(Direction dir, float minDistance) {
        float d = getFilteredDistance(dir);
        return (d < 0) || (d > minDistance); // -1 = pas de mesure, on considere libre
    }

    // --- Recommandation de vitesse ---
    // Retourne un facteur 0.0 (arret) a 1.0 (pleine vitesse)
    // base sur l'obstacle le plus proche devant
    float getSpeedFactor() {
        float front = lastFiltered[FRONT];
        if (front < 0) return 0.5; // pas de mesure, prudence

        if (front < DIST_EMERGENCY) return 0.0;
        if (front < DIST_DANGER)    return 0.15;
        if (front < DIST_WARNING)   return 0.4;
        if (front < DIST_CAUTION)   return 0.7;
        return 1.0;
    }

    // Recommandation de vitesse en prenant en compte toutes les directions
    // Utile pour adapter la vitesse en virage
    float getSpeedFactorAll() {
        float minFactor = 1.0;
        for (int i = 0; i < 4; i++) {
            float d = lastFiltered[i];
            if (d < 0) continue;

            float mult = thresholdMultiplier[i];
            float factor;
            if (d < DIST_EMERGENCY * mult) factor = 0.0;
            else if (d < DIST_DANGER * mult) factor = 0.15;
            else if (d < DIST_WARNING * mult) factor = 0.4;
            else if (d < DIST_CAUTION * mult) factor = 0.7;
            else factor = 1.0;

            if (factor < minFactor) minFactor = factor;
        }
        return minFactor;
    }

    // --- Diagnostics ---

    // Verifier qu'un capteur repond (envoie un ping et attend un echo)
    bool isSensorAlive(Direction dir) {
        float d = measureRaw((int)dir);
        // un capteur debranche renvoie systematiquement 0 (timeout)
        // on fait 3 tentatives
        if (d > 0) return true;
        delay(10);
        d = measureRaw((int)dir);
        if (d > 0) return true;
        delay(10);
        d = measureRaw((int)dir);
        return (d > 0);
    }

    // Nombre de mesures echouees depuis le demarrage
    unsigned int getFailCount(Direction dir) { return failCount[dir]; }

    // Derniere distance filtree sans relancer de mesure
    float getLastDistance(Direction dir) { return lastFiltered[dir]; }

    // Detection d'urgence : objet apparu tres pres tres vite
    bool isEmergency() {
        for (int i = 0; i < 4; i++) {
            float d = lastFiltered[i];
            float mult = thresholdMultiplier[i];
            if (d > 0 && d < DIST_EMERGENCY * mult) {
                return true;
            }
        }
        return false;
    }

    // Chaine de statut pour le debug serie
    String getStatusString() {
        String s = "US: ";
        const char* labels[] = {"AV", "AR", "GA", "DR"};
        for (int i = 0; i < 4; i++) {
            s += labels[i];
            s += ":";
            if (lastFiltered[i] < 0) {
                s += "---";
            } else {
                s += String(lastFiltered[i], 1);
            }
            s += "cm ";
        }

        ThreatLevel maxThreat = THREAT_NONE;
        for (int i = 0; i < 4; i++) {
            ThreatLevel t = evaluateThreat((Direction)i, lastFiltered[i]);
            if (t > maxThreat) maxThreat = t;
        }

        switch (maxThreat) {
            case THREAT_NONE:      s += "[OK]"; break;
            case THREAT_CAUTION:   s += "[ATTENTION]"; break;
            case THREAT_WARNING:   s += "[RALENTIR]"; break;
            case THREAT_DANGER:    s += "[DANGER]"; break;
            case THREAT_EMERGENCY: s += "[URGENCE]"; break;
        }

        return s;
    }
};

#endif
