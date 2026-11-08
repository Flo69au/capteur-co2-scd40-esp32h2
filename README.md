# Capteur CO2 / température / humidité Zigbee — ESP32-H2 + SCD40

Version **ESP32-H2** (Waveshare ESP32-H2-Zero) du capteur
[capteur-temp-rature-scd40_zigbee](https://github.com/Flo69au/capteur-temp-rature-scd40_zigbee)
(XIAO ESP32-C6). Firmware Arduino, **routeur Zigbee** (alimentation secteur), pour Zigbee2MQTT.

- Température + humidité (endpoint 10) et CO2 (endpoint 11), envoyés toutes les 30 s
- Calibrage automatique du CO2 (ASC, 400 ppm), offset de température (`TEMP_OFFSET_C`) et
  correction d'humidité (`HUM_OFFSET_PCT`) réglables
- Bouton d'appairage (GPIO1 → GND) + bouton BOOT (appui long 5 s), LED RGB de statut
- Mise à jour OTA par le réseau Zigbee, publiée automatiquement par GitHub Actions

## Câblage

| SCD40 | ESP32-H2-Zero |
|---|---|
| VDD | 3V3 |
| GND | GND |
| SDA | GPIO12 |
| SCL | GPIO22 |
| Bouton d'appairage | GPIO1 → GND |

Broches I2C par défaut de l'ESP32-H2 dans Arduino : vérifier la sérigraphie de la carte.
À éviter : GPIO 2, 3, 8, 9, 25 (démarrage / LED), 26/27 (USB). `i2c_scanner_h2/` permet de vérifier
que le SCD40 répond en `0x62`.

| LED RGB (GPIO8) | Signification |
|---|---|
| Orange clignotant | Appairage en cours (pas encore connecté au réseau) |
| Éteinte | Connecté, fonctionnement normal |
| Blanc fixe | Réinitialisation Zigbee demandée (bouton) |
| Bleu rapide | Mise à jour OTA en cours |

## Compilation (Arduino IDE)

- Core **esp32 by Espressif** ≥ 3.3, librairie **Sensirion I2C SCD4x**
- Outils → Carte : `ESP32H2 Dev Module`
- Outils → USB CDC On Boot : `Enabled`
- Outils → Zigbee mode : `Zigbee ZCZR (coordinator/router)`
- Outils → Partition Scheme : `Zigbee ZCZR 4MB with spiffs`
- Outils → Erase All Flash Before Sketch Upload : `Enabled` au premier flash, `Disabled` ensuite

Calibrage : partir de `TEMP_OFFSET_C 4.0` (l'ESP32-H2 chauffe moins que la C6), laisser la carte
30 min dans son boîtier puis `nouvel_offset = T_capteur − T_référence + offset_installé`.

## Zigbee2MQTT

1. Copier `z2m_converter_h2.js` dans `external_converters/` (à côté de `configuration.yaml`),
   il peut cohabiter avec le convertisseur de la version C6.
2. L'OTA passe par l'index commun [zigbee-ota](https://github.com/Flo69au/zigbee-ota), qui
   reprend toutes les heures le `ota_index.json` de ce dépôt :
   ```yaml
   ota:
     zigbee_ota_override_index_location: https://raw.githubusercontent.com/Flo69au/zigbee-ota/main/ota_index.json
     disable_automatic_update_check: false
     update_check_interval: 1440
     default_maximum_data_size: 50           # ne PAS mettre 100 : l'OTA échoue dès le début
   ```
3. Redémarrer Zigbee2MQTT, *Permit join*, puis brancher la carte : elle apparaît comme
   `Florian-DIY H2-SCD40-CO2` (routeur).

## Publier une mise à jour OTA

Identifiants OTA : fabricant `0x1337`, type d'image **`0x0003`** (`0x0001` = version C6,
`0x0002` = sonde radiateur), modèle `H2-SCD40-CO2`. Une image H2 ne peut donc jamais être
proposée à la version C6, ni l'inverse.

1. Commit + push des modifications de `capteur_co2_h2/capteur_co2_h2.ino`
2. Créer un tag **`v<majeur>.<mineur>`** plus grand que le précédent (`v1.1`, `v1.2`…, ou `v1.2.1`),
   depuis GitHub : *Releases → Draft a new release → nouveau tag → Publish release*. Mettre la même
   valeur dans `OTA_FW_VERSION` du sketch (format `0xMMmmPP00` : v1.1 → `0x01010000`).
3. Le workflow `.github/workflows/ota-release.yml` compile pour l'ESP32-H2, crée l'image `.zigbee`,
   la publie dans la Release et met à jour `ota_index.json`
4. `zigbee-ota` reprend l'index dans l'heure (ou tout de suite : *Actions → Fusion des index OTA →
   Run workflow*). Dans Z2M : *Check for updates* → *Update* (~1 h, ne pas redémarrer Z2M pendant).

Pas de `"force": true` dans l'index : la mise à jour serait proposée en boucle.
