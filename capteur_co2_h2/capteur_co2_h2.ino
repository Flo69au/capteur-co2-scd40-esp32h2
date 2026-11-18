/*
 * Capteur Zigbee CO2 / Température / Humidité
 * Waveshare ESP32-H2-Zero (ESP32-H2FH4S) + Sensirion SCD40 (I2C)
 *
 * Bibliothèque à installer (Gestionnaire de bibliothèques Arduino) :
 *   - "Sensirion I2C SCD4x" (auteur: Sensirion AG)
 *   La bibliothèque Zigbee est fournie nativement par le core ESP32
 *   (Boards Manager -> "esp32" by Espressif Systems, version 3.3.x ou plus récente).
 *
 * Réglages Outils (Arduino IDE) AVANT de flasher :
 *   - Carte                                   : ESP32H2 Dev Module
 *   - USB CDC On Boot                         : Enabled (moniteur série par l'USB-C)
 *   - Zigbee mode                             : Zigbee ZCZR (coordinator/router)
 *   - Partition Scheme                        : Zigbee ZCZR 4MB with spiffs
 *   - Erase All Flash Before Sketch Upload    : Enabled au 1er flash, Disabled ensuite
 *                                                (pour garder l'appairage)
 *
 * Mode routeur : le capteur est alimenté sur secteur, il relaie donc aussi les
 * messages des autres appareils Zigbee (maillage plus solide). Changer de rôle
 * (routeur <-> end device) change la table des partitions : flash USB avec
 * effacement complet, jamais par OTA.
 *
 * Câblage SCD40 -> ESP32-H2-Zero :
 *   VDD -> 5V   (seulement si SDA/SCL restent à ~3,3 V une fois le module
 *                alimenté : mesurer avant de brancher, voir README ; sinon 3V3)
 *   GND -> GND
 *   SDA -> GPIO12
 *   SCL -> GPIO22
 *   (broches I2C par défaut de l'ESP32-H2 dans Arduino ; vérifier la sérigraphie.
 *    À éviter : GPIO 2, 3, 8, 9, 25 = broches de démarrage/LED, 26/27 = USB)
 *
 * Bouton d'appairage (optionnel, en plus du bouton BOOT) :
 *   Un simple bouton-poussoir entre GPIO1 et GND. Utilise la pull-up
 *   interne, donc pas besoin de résistance externe. Un appui déclenche un
 *   factory reset Zigbee (le device redevient "non appairé" et rejoint le
 *   1er réseau ouvert détecté). Change PAIR_BUTTON ci-dessous si tu préfères
 *   une autre broche libre (GPIO 0, 4, 5, 10, 11...).
 *   Le bouton BOOT de la carte (GPIO9) fonctionne aussi : appui long > 5 s.
 *
 * LED de statut (LED RGB WS2812 intégrée, RGB_BUILTIN = GPIO8) :
 *   - connecté, fonctionnement normal : éteinte
 *   - non connecté au réseau Zigbee (appairage, perte du réseau) : rouge clignotant
 *   - envoi des données à Zigbee2MQTT : flash vert
 *   - échec d'envoi : flash rouge
 *   - bouton maintenu : orange, puis violet quand le reset Zigbee est lancé
 *   - mise à jour OTA en cours : bleu rapide
 *   Couleurs dans les fonctions led...() plus bas.
 *
 * Côté Zigbee2MQTT : ce firmware n'étant pas un device "connu", il faut lui
 * associer le convertisseur externe du repo (h2_scd40_co2.js).
 *
 * Mise à jour OTA (par le réseau Zigbee, pas de WiFi nécessaire) :
 *   Le device embarque un client OTA Zigbee natif.
 *
 *   Méthode recommandée : GitHub (automatique)
 *     Publier un tag "v1.0", "v1.1"... sur le repo GitHub : le workflow
 *     .github/workflows/ota-release.yml compile ce sketch avec
 *     OTA_FW_VERSION = version du tag, fabrique l'image .zigbee, la publie
 *     dans une Release et met à jour ota_index.json. L'index commun
 *     Flo69au/zigbee-ota le reprend dans l'heure ; c'est lui que lit Zigbee2MQTT.
 *
 *   Méthode manuelle (sans GitHub) :
 *     1. Sketch -> "Export compiled binary" dans l'Arduino IDE (.bin)
 *     2. python image_builder_tool.py --manuf-id 0x1337 --image-type 0x0003 \
 *          --file-version 0x01080000 --tag 0x0000 capteur_co2_h2.ino.bin
 *        (outil Espressif : esp-zigbee-sdk/tools/image_builder_tool/)
 *        -> produit 1337-0003-01080000-ota-file.zigbee
 *     3. Le déclarer dans un index JSON lu par Zigbee2MQTT (voir le guide).
 *
 *   Ne JAMAIS mettre "force": true dans l'index : Z2M proposerait alors la
 *   mise à jour en permanence, même si la version est déjà installée.
 *   Le device vérifie automatiquement les mises à jour ~1 min après connexion
 *   puis toutes les heures. Pendant une mise à jour, la LED clignote vite en bleu et
 *   les boutons de reset sont désactivés pour éviter de couper l'update.
 */

#include <Arduino.h>

#ifndef ZIGBEE_MODE_ZCZR
#error "Sélectionne 'Zigbee ZCZR (coordinator/router)' dans Outils -> Zigbee mode (et Partition Scheme 'Zigbee ZCZR 4MB with spiffs')"
#endif

#include "Zigbee.h"
#include <Wire.h>
#include <SensirionI2cScd4x.h>

// ---- Configuration ----
#define TEMP_HUM_ENDPOINT 10
#define CO2_ENDPOINT      11
#define I2C_SDA           12  // GPIO12
#define I2C_SCL           22  // GPIO22
#define PAIR_BUTTON       1   // GPIO1 : bouton dédié à l'appairage (vers GND)
#define STATUS_LED        RGB_BUILTIN  // LED RGB WS2812 intégrée (GPIO8)

// Doivent correspondre au "zigbeeModel"/"model" du convertisseur externe Z2M
#define ZB_MANUFACTURER "Florian-DIY"
#define ZB_MODEL        "H2-SCD40-CO2"

// ---- OTA ----
// Version de CE firmware, format 0xMMmmPP00 (majeur.mineur.correctif) :
//   v1.0 -> 0x01000000, v1.1 -> 0x01010000, v1.2.3 -> 0x01020300
// Pour une release GitHub, elle est imposée depuis le tag (v1.1, v1.2...) via
// -DOTA_FW_VERSION. La valeur par défaut ci-dessous sert au flash USB depuis
// l'Arduino IDE : la garder ÉGALE au dernier tag publié, pour que Zigbee2MQTT
// ne repropose pas une mise à jour déjà installée.
#ifndef OTA_FW_VERSION
#define OTA_FW_VERSION       0x01070000  // v1.7 (couleurs de LED corrigées : g et r inversés)
#endif
#define OTA_HW_VERSION       0x0101      // version matérielle (rarement à changer)
#define OTA_MANUFACTURER_ID  0x1337      // arbitraire, doit matcher l'image .zigbee généré
#define OTA_IMAGE_TYPE       0x0003      // 3 = capteur CO2 ESP32-H2 (2 = sonde radiateur, 1 = ancien capteur C6 abandonné)

// Offset de température (°C) -> corrige aussi l'humidité (voir setup()).
// Valeur usine par défaut = 4.0°C. À calibrer une fois la carte dans son boîtier
// (~30 min de stabilisation) : nouvel_offset = T_capteur - T_reference + offset_installé
// Chaque carte doit être calibrée séparément (boîtier, alimentation et placement changent l'écart).
// 2026-11-08 : capteur 25.8°C vs référence 21.2°C avec 4.0 -> 25.8 - 21.2 + 4.0 = 8.6
#define TEMP_OFFSET_C 8.6

// Correction d'humidité (en points de %RH), appliquée par le firmware avant
// l'envoi Zigbee. À régler seulement si un écart stable persiste une fois la
// température juste : HUM_OFFSET_PCT = RH_reference - RH_affichée + ancien offset
// 2026-11-08 (v1.2, offset T 8.6) : affiché 61.3% vs référence 51% -> 51 - 61.3 + 0.0 = -10.3
// 2026-11-10 : réglé à -6.1 par Florian (v1.4)
// 2026-11-15 (v1.4) : affiché 59.3% vs référence 51.7% -> 51.7 - 59.3 + (-6.1) = -13.7
#define HUM_OFFSET_PCT -13.7

// Envoi vers Zigbee2MQTT : chaque mesure n'est envoyée que si elle a varié d'au
// moins son seuil depuis son dernier envoi, ou si elle n'a rien envoyé depuis
// REPORT_HEARTBEAT_MS (envoi de sécurité). Plus d'envoi forcé périodique.
// C'est le firmware qui décide : le reporting automatique de la pile Zigbee est
// neutralisé (voir setup()), il ne garantit pas un envoi régulier.
#define TEMP_REPORT_DELTA_C   0.1f        // °C
#define HUM_REPORT_DELTA_PCT  0.1f        // %RH
#define CO2_REPORT_DELTA_PPM  1.0f        // ppm
#define REPORT_HEARTBEAT_MS   3600000UL   // envoi de sécurité : au moins 1 fois par heure

// Suivi du dernier envoi d'une mesure
struct ReportState {
  bool sent = false;        // false = rien envoyé depuis le démarrage
  float value = 0.0f;       // dernière valeur envoyée
  unsigned long ms = 0;     // moment du dernier envoi

  // true si la valeur doit partir : 1er envoi, variation >= seuil, ou 1 h sans envoi
  // (marge de 0,001 : en flottant 20,1 - 20,0 donne 0,0999...)
  bool due(float v, float delta) const {
    if (!sent) return true;
    if (fabsf(v - value) >= delta - 0.001f) return true;
    return millis() - ms >= REPORT_HEARTBEAT_MS;
  }
  void done(float v) {
    sent = true;
    value = v;
    ms = millis();
  }
};
ReportState repTemp, repHum, repCO2;

uint8_t button = BOOT_PIN;
volatile bool otaRunning = false;
volatile bool otaEnded = false;          // OTA terminée sans redémarrage auto (= échec)
volatile unsigned long otaStartMs = 0;

// Garde-fou OTA : la lib Zigbee ne prévient pas le sketch quand une mise à jour
// est annulée (ABORT), et garde alors un état interne incohérent qui fait
// échouer les tentatives suivantes. Si l'OTA dure plus que ça, ou se termine
// sans redémarrer, le firmware redémarre pour repartir sur un état propre.
#define OTA_MAX_DURATION_MS (2UL * 60UL * 60UL * 1000UL)  // 2 h

// LED RGB WS2812 (RGB_BUILTIN) : pilotée en couleur avec rgbLedWrite()
static bool ledState = false;
inline void ledColor(uint8_t r, uint8_t g, uint8_t b) {
  rgbLedWrite(STATUS_LED, g, r, b);  // g et r inversés : LED de l'ESP32-H2-Zero
  ledState = (r | g | b) != 0;
}
inline void ledOff() {  // connecté, fonctionnement normal
  ledColor(0, 0, 0);
}
inline void ledToggleDisconnected() {  // rouge clignotant : pas connecté au réseau
  if (ledState) ledOff(); else ledColor(40, 0, 0);
}
inline void ledButtonHeld() {  // orange : bouton maintenu
  ledColor(40, 15, 0);
}
inline void ledResetting() {  // violet : reset Zigbee lancé, redémarrage
  ledColor(30, 0, 40);
}

// Flash bref à chaque envoi de données (non bloquant : éteint dans loop())
#define LED_FLASH_MS 150
static unsigned long ledFlashUntil = 0;
inline void ledFlash(bool ok) {  // vert = envoyé, rouge = échec
  if (ok) ledColor(0, 40, 0); else ledColor(40, 0, 0);
  ledFlashUntil = millis() + LED_FLASH_MS;
}

void onOTAStateChange(bool active) {
  if (active) {
    otaStartMs = millis();
    otaEnded = false;
  } else if (otaRunning) {
    otaEnded = true;  // en cas de succès la lib redémarre d'elle-même juste après
  }
  otaRunning = active;
  Serial.println(active ? "OTA: mise à jour démarrée" : "OTA: terminée");
}

SensirionI2cScd4x scd40;
ZigbeeTempSensor zbTempHum = ZigbeeTempSensor(TEMP_HUM_ENDPOINT);
ZigbeeCarbonDioxideSensor zbCO2 = ZigbeeCarbonDioxideSensor(CO2_ENDPOINT);

static char errorMessage[64];

void setup() {
  Serial.begin(115200);
  delay(2000);  // laisse le temps d'ouvrir le moniteur série en debug
  Serial.printf("Firmware v%lu.%lu.%lu (0x%08lX)\r\n",
                (unsigned long)((OTA_FW_VERSION >> 24) & 0xFF),
                (unsigned long)((OTA_FW_VERSION >> 16) & 0xFF),
                (unsigned long)((OTA_FW_VERSION >> 8) & 0xFF),
                (unsigned long)OTA_FW_VERSION);

  pinMode(button, INPUT_PULLUP);
  pinMode(PAIR_BUTTON, INPUT_PULLUP);
  ledOff();

  // ---------------- I2C / SCD40 ----------------
  Wire.begin(I2C_SDA, I2C_SCL);
  scd40.begin(Wire, SCD40_I2C_ADDR_62);

  uint16_t error;
  scd40.wakeUp();
  scd40.stopPeriodicMeasurement();
  scd40.reinit();

  uint64_t serialNumber = 0;
  error = scd40.getSerialNumber(serialNumber);
  if (error) {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("Erreur getSerialNumber: %s\r\n", errorMessage);
  }

  // Calibrage automatique (ASC) : le SCD40 corrige tout seul sa dérive dans le
  // temps, en supposant que la valeur la plus basse vue sur une période (par
  // défaut ~6,5 jours après une 1ère période de rodage de ~44h) correspond à
  // de l'air frais (~400 ppm). Pour que ça fonctionne, la pièce doit être
  // ventilée/aérée de temps en temps. Réglage stocké en EEPROM sur le capteur
  // (persiste entre les redémarrages), mais on le force à chaque boot pour
  // être sûr qu'il est bien actif.
  error = scd40.setAutomaticSelfCalibrationEnabled(1);
  if (error) {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("Erreur setAutomaticSelfCalibrationEnabled: %s\r\n", errorMessage);
  }
  error = scd40.setAutomaticSelfCalibrationTarget(400);  // cible = air frais, 400 ppm
  if (error) {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("Erreur setAutomaticSelfCalibrationTarget: %s\r\n", errorMessage);
  }

  // Le SCD40 n'a pas de calibrage dédié pour l'humidité : la RH est calculée
  // à partir de la température mesurée en interne, qui chauffe légèrement à
  // cause de l'auto-échauffement du capteur/de l'électronique une fois dans
  // son boîtier. Corriger cet offset de température corrige donc aussi la RH
  // (et n'affecte pas la précision du CO2). Voir le guide du projet pour la
  // procédure de calibrage (comparaison avec un thermomètre de référence).
  error = scd40.setTemperatureOffset(TEMP_OFFSET_C);
  if (error) {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("Erreur setTemperatureOffset: %s\r\n", errorMessage);
  }

  error = scd40.startPeriodicMeasurement();
  if (error) {
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.printf("Erreur startPeriodicMeasurement: %s\r\n", errorMessage);
  }

  // ---------------- Endpoint Température + Humidité (EP 10) ----------------
  zbTempHum.setManufacturerAndModel(ZB_MANUFACTURER, ZB_MODEL);
  zbTempHum.setMinMaxValue(-10, 60);     // plage SCD40
  zbTempHum.setTolerance(0.2);
  zbTempHum.setDefaultValue(20.0);       // valeur affichée avant la 1ère mesure réelle
  zbTempHum.addHumiditySensor(0, 100, 1, 50.0);

  // Client OTA (une seule fois suffit, il met à jour tout le firmware du device)
  zbTempHum.addOTAClient(OTA_FW_VERSION, OTA_FW_VERSION, OTA_HW_VERSION, OTA_MANUFACTURER_ID, OTA_IMAGE_TYPE);
  zbTempHum.onOTAStateChange(onOTAStateChange);

  Zigbee.addEndpoint(&zbTempHum);

  // ---------------- Endpoint CO2 (EP 11) ----------------
  zbCO2.setManufacturerAndModel(ZB_MANUFACTURER, ZB_MODEL);
  zbCO2.setMinMaxValue(0, 5000);         // plage SCD40 (jusqu'à 40000 en mode non calibré)
  Zigbee.addEndpoint(&zbCO2);

  Serial.println("Démarrage Zigbee...");
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {  // routeur Zigbee (alimentation secteur)
    Serial.println("Échec du démarrage Zigbee, redémarrage...");
    ESP.restart();
  }

  Serial.print("Connexion au réseau Zigbee (appairage en cours)");
  while (!Zigbee.connected()) {
    Serial.print(".");
    ledToggleDisconnected();  // rouge clignotant tant que non connecté
    delay(200);
  }
  Serial.println();
  Serial.println("Connecté au réseau Zigbee !");
  ledOff();  // appairage terminé -> LED éteinte en fonctionnement normal

  // Reporting automatique de la pile Zigbee neutralisé -> DOIT être appelé après
  // Zigbee.begin() (réappliqué à chaque démarrage). Le firmware décide des envois
  // (seuils ou 1 h) : max = 65000 s (~18 h), seuils hors d'atteinte (600 °C, 600 %,
  // 65000 ppm). Doit rester aligné sur le convertisseur Z2M.
  zbTempHum.setReporting(0, 65000, 600);
  zbTempHum.setHumidityReporting(0, 65000, 600);
  zbCO2.setReporting(0, 65000, 65000);

  // 1ère vérification OTA ~1 min après connexion, puis toutes les heures automatiquement
  zbTempHum.requestOTAUpdate();
}

void loop() {
  static unsigned long lastRead = 0;

  // Pendant une mise à jour OTA : LED clignote vite, boutons de reset désactivés
  // (couper l'alimentation ou remettre à zéro le réseau en plein flash OTA = brique)
  if (otaEnded) {
    Serial.println("OTA interrompue -> redémarrage pour repartir sur un état propre");
    delay(2000);
    ESP.restart();
  }
  if (otaRunning) {
    if (millis() - otaStartMs > OTA_MAX_DURATION_MS) {
      Serial.println("OTA bloquée depuis trop longtemps -> redémarrage");
      delay(500);
      ESP.restart();
    }
    if ((millis() / 100) % 2) ledColor(0, 0, 40); else ledOff();  // bleu rapide pendant l'OTA
    delay(10);
    return;
  }

  // Appui long (>5s) sur le bouton BOOT = factory reset Zigbee (re-appairage)
  // LED orange tant que le bouton est maintenu, violette quand le reset est lancé
  if (digitalRead(button) == LOW) {
    delay(100);
    unsigned long start = millis();
    if (digitalRead(button) == LOW) ledButtonHeld();
    while (digitalRead(button) == LOW && !otaRunning) {
      delay(50);
      if (millis() - start > 5000) {
        Serial.println("Factory reset Zigbee...");
        ledResetting();
        delay(500);
        Zigbee.factoryReset();
      }
    }
    ledOff();  // relâché avant 5 s : rien ne se passe
  }

  // Bouton dédié à l'appairage : simple appui court -> factory reset Zigbee
  if (digitalRead(PAIR_BUTTON) == LOW) {
    delay(50);  // anti-rebond
    if (digitalRead(PAIR_BUTTON) == LOW) {
      Serial.println("Bouton d'appairage pressé -> factory reset Zigbee...");
      ledButtonHeld();  // orange : bouton pressé
      delay(300);
      ledResetting();   // puis violet : reset lancé
      delay(500);
      Zigbee.factoryReset();
      while (digitalRead(PAIR_BUTTON) == LOW) {
        delay(10);  // attend le relâchement du bouton
      }
    }
  }

  // Fin du flash LED d'envoi
  if (ledFlashUntil && (long)(millis() - ledFlashUntil) >= 0) {
    ledFlashUntil = 0;
    ledOff();
  }

  // Réseau Zigbee perdu : rouge clignotant jusqu'au retour de la connexion
  static unsigned long lastBlink = 0;
  if (!Zigbee.connected()) {
    if (millis() - lastBlink >= 200) {
      lastBlink = millis();
      ledToggleDisconnected();
    }
  } else if (!ledFlashUntil && ledState) {
    ledOff();  // reconnecté : LED éteinte
  }

  // Le SCD40 produit une nouvelle mesure toutes les ~5s en mode périodique
  if (millis() - lastRead > 1000) {
    lastRead = millis();
    bool dataReady = false;
    uint16_t error = scd40.getDataReadyStatus(dataReady);
    if (!error && dataReady) {
      uint16_t co2 = 0;
      float temperature = 0.0f;
      float humidity = 0.0f;
      error = scd40.readMeasurement(co2, temperature, humidity);
      if (!error) {
        float rawHumidity = humidity;
        humidity = constrain(humidity + HUM_OFFSET_PCT, 0.0f, 100.0f);
        Serial.printf("CO2: %u ppm | T: %.1f°C | RH: %.1f%% (brut %.1f%%)\r\n", co2, temperature, humidity, rawHumidity);
        zbTempHum.setTemperature(temperature);
        zbTempHum.setHumidity(humidity);
        zbCO2.setCarbonDioxide((float)co2);

        // N'envoie que les mesures qui ont assez varié (ou 1 h sans envoi).
        // En cas d'échec, la valeur n'est pas notée comme envoyée : nouvel essai
        // à la mesure suivante (~5 s).
        if (Zigbee.connected()) {
          bool sentT = false, sentH = false, sentC = false, ok = true;
          if (repTemp.due(temperature, TEMP_REPORT_DELTA_C)) {
            sentT = true;
            if (zbTempHum.reportTemperature()) repTemp.done(temperature); else ok = false;
          }
          if (repHum.due(humidity, HUM_REPORT_DELTA_PCT)) {
            sentH = true;
            if (zbTempHum.reportHumidity()) repHum.done(humidity); else ok = false;
          }
          if (repCO2.due((float)co2, CO2_REPORT_DELTA_PPM)) {
            sentC = true;
            if (zbCO2.report()) repCO2.done((float)co2); else ok = false;
          }
          if (sentT || sentH || sentC) {
            Serial.printf("Rapport Zigbee%s%s%s : %s\r\n", sentT ? " T" : "", sentH ? " RH" : "",
                          sentC ? " CO2" : "", ok ? "envoyé" : "ÉCHEC");
            ledFlash(ok);  // flash vert (ou rouge si échec)
          }
        }
      } else {
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.printf("Erreur lecture SCD40: %s\r\n", errorMessage);
      }
    }
  }
}
