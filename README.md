# HM485 Gateway für WT32-ETH01

Ein ESP32/WT32-ETH01 als Ethernet-/MQTT-Gateway für **Homematic Wired / HM485**.

Der aktuelle Entwicklungsstand ist **v0.9.3g1**. Das Gateway arbeitet produktiv mit nativer HM485-Discovery für offizielle HMW-Geräte und zusätzlicher passiver Erkennung ausgewählter HBW/Homebrew-Geräte. Zustände werden gelesen, über MQTT/Home Assistant bereitgestellt und bei ausdrücklich freigegebenen, real getesteten Aktoren auch geschrieben.

Schreibzugriffe sind weiterhin **geräte- und protokollspezifisch freigegeben**. Es gibt keinen generischen Schreibmodus für unbekannte Geräte.

## Aktueller Funktionsumfang

- native HM485-Discovery über den vollständigen 32-Bit-Prefix-Baum
- passive Erkennung von HBW/Homebrew-Geräten über gültige Busframes und `0x41`-Identity-Broadcasts
- keine fest im Code hinterlegten Geräteadressen
- automatische Abfrage von Gerätetyp, Seriennummer und Firmware
- Lesen von EEPROM-Daten und Kanalstatus für dafür freigegebene Geräte
- passive Auswertung von HM485-Statusereignissen für schnelle Zustandsupdates
- gezieltes Status-Polling für Geräte, deren aktiver Read-Pfad real verifiziert ist
- persistente Gerätedatenbank im NVS; bekannte passive Geräte werden nach einem ESP-Neustart wiederhergestellt
- gezielte Verifikation wiederhergestellter Geräte nach Native Discovery
- explizites „Gerät vergessen“ inklusive Löschen der retained Home-Assistant-Discovery-Einträge
- kontrollierte Schaltfunktion für real verifizierte Aktorprofile
- Ethernet als primärer Netzwerkzugang
- WLAN als Fallback
- MQTT mit retained States und Availability
- Home Assistant MQTT Discovery
- mDNS unter `http://<hostname>.local/`
- direkter Home-Assistant-Link zur Gateway-Weboberfläche (`configuration_url`)
- stabile MQTT-Pfade auf Basis der Seriennummer
- frei vergebbare Geräte- und Kanalnamen
- semantische Kanalprofile
- NO/NC-Invertierung pro Kanal
- persistente Konfiguration im ESP32-NVS
- Web-Oberfläche mit Deutsch/Englisch-Umschaltung
- Konfigurations-/Profil-Backup und Restore
- Web-OTA
- RAW-RX-Only-Diagnosemodus mit harter TX-Sperre
- passiver HM485-Adresskonfliktschutz
- erweiterte ESP32-Systemdiagnose mit Chip-, RAM-, Flash-, Sketch- und Reset-Informationen

## Standard-Zugangsdaten / Erstinstallation

Nach einem frischen Flash ohne vorhandene NVS-Konfiguration gelten folgende Standardwerte:

| Funktion | Standardwert |
|---|---|
| Web-Benutzername | `admin` |
| Web-Passwort | `hm485setup` |
| Setup-AP SSID | `HM485-Gateway-XXXXXX` |
| Setup-AP Passwort | `hm485setup` |
| MQTT-Port | `1883` |
| MQTT Base Topic | `hm485` |
| Home-Assistant Discovery Prefix | `homeassistant` |
| Hostname | `hm485-gateway` |
| HM485-Zentraladresse | `00000001` |
| Websprache | Deutsch |

`XXXXXX` wird aus der ESP32-Chip-ID gebildet und ist bei jedem Gateway unterschiedlich.

Wenn weder Ethernet noch ein konfiguriertes WLAN zur Verfügung steht, stellt das Gateway den Setup-AP bereit. Die Setup-Weboberfläche ist dann normalerweise über die AP-Adresse des ESP32 erreichbar.

**Wichtig:** Benutzername und Passwort sollten nach der Erstinstallation geändert werden. Für einen Parallelbetrieb mit einer vorhandenen HM485-/FHEM-Zentrale sollte außerdem die eigene HM485-Adresse vor produktiven Tests von `00000001` auf z. B. `00000002` geändert werden.

## Hardware

### Controller

- WT32-ETH01 / ESP32
- integriertes LAN8720 Ethernet

### RS485-Transceiver

Aktuell getestet mit einem SP485/SP3485-artigen 3,3-V-RS485-Transceiver.

Verdrahtung am WT32:

| Funktion | GPIO | RS485-Modul |
|---|---:|---|
| HM485 RX | GPIO35 | RO / RX |
| HM485 TX | GPIO17 | DI / TX |
| Richtung | GPIO33 | DE + /RE bzw. RTS |
| GND | GND | GND |

GPIO35 ist ein reiner Eingang und besitzt beim ESP32 **keinen internen Pull-up**.

### UART

HM485 wird mit folgenden Parametern betrieben:

- 19200 Baud
- 8 Datenbits
- Even Parity
- 1 Stopbit
- `SERIAL_8E1`

## Wichtig: A/B-Polarität

Die A/B-Bezeichnung von RS485-Modulen ist leider nicht herstellerübergreifend einheitlich.

Im Projekt zeigte sich, dass eine vertauschte A/B-Polarität zwar teilweise plausibel wirkende Signale erzeugen kann, aktive HM485-Kommunikation aber nicht korrekt funktioniert. Mit der richtigen Polarität funktionieren Discovery sowie TYPE-, SERIAL-, FW-, EEPROM- und STATUS-Abfragen bidirektional.

Wenn beim Aufbau nur `00`-Antworten, lange LOW-Pegel oder keine gültigen Antworten auftreten, sollte die A/B-Polarität als Erstes geprüft werden.

## HM485-Adresse des Gateways

Die reguläre Zentraladresse ist:

```text
00000001
```

Für Parallelbetrieb mit einer bestehenden FHEM-/HM485-Zentrale sollte eine andere Adresse verwendet werden, z. B.:

```text
00000002
```

Die Adresse kann in der Weboberfläche geändert werden.

### Adresskonfliktschutz

Nach jedem Boot lauscht das Gateway zunächst passiv auf dem Bus. Wird ein gültiges Telegramm mit der **eigenen konfigurierten Source-Adresse** erkannt, sperrt das Gateway HM485-TX.

Zusätzlich wird angezeigt, ob die reguläre Zentraladresse `00000001` passiv auf dem Bus gesehen wurde.

Wichtig: Eine Adresse, die nicht gesehen wurde, ist dadurch nicht garantiert frei. Eine vorhandene Zentrale könnte im Beobachtungszeitraum lediglich still gewesen sein.

## Discovery

Die native Discovery orientiert sich am Verhalten von `hm485d` / `HM485_Protocol.pm`.

Grundprinzip:

1. Start bei Adresse `00000000` mit einem gültigen Prefix-Bit.
2. `CTRL` wird aus der Prefix-Tiefe gebildet.
3. Ein erstes empfangenes Byte ungleich `00` bedeutet: In diesem Prefix-Zweig befindet sich mindestens ein Gerät.
4. Ein `00` bzw. Timeout gilt als negative Antwort.
5. Negative Prefixe werden bis zu dreimal geprüft.
6. Der Baum wird bis zur vollständigen 32-Bit-Adresse durchlaufen.

Für offizielle HMW-Geräte bleibt die native Discovery die primäre Quelle. HBW/Homebrew-Geräte können die native Prefix-Discovery jedoch absichtlich nicht beantworten und werden deshalb zusätzlich passiv gelernt.

Bekannte Geräte werden persistent im NVS registriert. Nach einem ESP-Neustart werden diese Einträge wieder in die Runtime-Gerätedatenbank übernommen. Das bedeutet **nicht automatisch „online“**: Geräte mit sicherem aktivem Read-Pfad werden nach Abschluss der nativen Discovery gezielt verifiziert; rein passive Geräte bleiben bekannt, bis wieder gültiger Busverkehr von ihnen gesehen wird.

## Geräte- und Kanalprofile

Nach der Discovery werden bekannte HM485-Gerätetypen einem internen Geräteprofil zugeordnet.

Aktuell enthalten sind unter anderem:

- `HMW-Sen-SC-12-DR`
- `HMW-IO-12-Sw14-DR`
- `HBW-1W-T10`
- `HBW-LC-Sw8`
- `HBW-Sen-EP` (derzeit in realer Protokollprüfung)

### Aktuell real getestete HBW-Geräte

| Typ | Device Type | Stand |
|---|---:|---|
| HBW-1W-T10 | `0x0081` | Identity und passive Temperaturwerte real getestet |
| HBW-LC-Sw8 | `0x0083` | Identity, 8 Kanäle, `53`-Statuspolling, `78`-Schalten und `69`-Rückmeldung real getestet |
| HBW-Sen-EP | `0x0084` | Identity und passive `69`-Telegramme real gesehen; FHEM-XML bestätigt 8 × 16-Bit-Counter sowie `LEVEL_GET`/`INFO_LEVEL`; reale Eingangs-/Konfigurationsprüfung läuft |

Beim HBW-LC-Sw8 ist der verifizierte Laufzeitpfad:

```text
LEVEL_GET:  53 <channel>
LEVEL_SET:  78 <channel> <00|C8>
INFO_LEVEL: 69 <channel> <00|C8> 00
```

Der HBW-LC-Sw8 wird nach einem eigenen `0x41`-Identity-Broadcast gezielt neu gepollt. Dadurch werden z. B. nach einem Modulneustart die echten Ausgangszustände wieder an MQTT/Home Assistant übertragen.

### HBW-Sen-EP (`0x0084`)

Die FHEM-XML beschreibt das Gerät als **„Homebrew Wired S0-Interface (8-fach)“** mit acht `COUNTER_INPUT`-Kanälen. Der über `INFO_LEVEL` übertragene `STATE` ist ein **16-Bit-Wert** (`0..65535`), nicht 24 Bit.

Der dokumentierte Laufzeitpfad ist:

```text
LEVEL_GET:  53 <bus-channel>
INFO_LEVEL: 69 <bus-channel> <counter_hi> <counter_lo>
```

`COUNTER` ist laut XML lesbar und als Event verfügbar. Pro Kanal sind außerdem diese EEPROM-Parameter beschrieben:

- `SEND_DELTA_COUNT`: `1..1000`, XML-Default `1`
- `SEND_MIN_INTERVAL`: `0..3600 s`, XML-Default `0`
- `SEND_MAX_INTERVAL`: `5..3600 s`, XML-Default `600`

Die aktuell vorliegenden Homebrew-Sourcen sind ausdrücklich als Entwicklungs-/Experimentierstand zu behandeln. Dort werden die acht Eingänge zyklisch alle 10 ms gelesen und der Zähler bei einer **LOW→HIGH-Flanke** erhöht. Die dortigen Defaultwerte (`SEND_MIN_INTERVAL=10 s`, `SEND_MAX_INTERVAL=150 s`) weichen von der FHEM-XML ab. Für Gateway-Unterstützung ist daher die XML die Referenz für das Geräteprofil; reale Busmitschnitte bleiben die Referenz für das tatsächlich geflashte Modul.

Pin-/Buskanal-Zuordnung des vorliegenden Source-Stands:

| Buskanal | Eingang | Arduino-Pin |
|---:|---|---|
| `00` | Sen1 | 14 / A0 |
| `01` | Sen2 | 15 / A1 |
| `02` | Sen3 | 16 / A2 |
| `03` | Sen4 | 17 / A3 |
| `04` | Sen5 | 18 / A4 |
| `05` | Sen6 | 19 / A5 |
| `06` | Sen7 | 6 |
| `07` | Sen8 | 7 |

Damit ist insbesondere wichtig: **`69 06 ...` gehört zu Sen7 / Arduino-Pin 6, nicht zu `#define Sen6 19`.**

Aktives Polling des HBW-Sen-EP wird erst im Gateway freigegeben, wenn `53` auf realer Hardware verifiziert wurde. Die passive Auswertung der `69`-Counter-Telegramme ist dagegen bereits protokollseitig klar.

Die Weboberfläche zeigt nur die regulär bekannte Homematic-Kanalnummer. Die interne BUS-Kanalnummer bleibt eine Implementierungsdetails des Codes.

Pro Kanal kann gewählt werden:

- Auto
- Fenster / Window
- Tür / Door
- Alarm
- Kontakt / Contact
- Binäreingang / Binary input
- Analogsensor / Analog sensor
- Frequenzsensor / Frequency sensor
- Ausgang / Output — je nach verifiziertem Geräteprofil read-only oder schaltbar
- Rollladen / Shutter — derzeit nur für bekannte/unterstützte Profile; generische Schreibfreigabe bleibt gesperrt
- Raw Sensor

### Invertierung NO/NC

Pro Kanal kann die logische Auswertung invertiert werden.

Der **HM485-Rohwert bleibt unverändert**. Nur der logische Zustand für MQTT/Home Assistant wird gedreht.

Das ist besonders für Kontakte sinnvoll, bei denen abhängig von NO/NC beispielsweise `0` entweder „geschlossen“ oder „offen“ bedeuten kann.

## MQTT

Die MQTT-Pfade bleiben unabhängig von frei vergebenen Anzeigenamen stabil.

Schema:

```text
<base-topic>/<seriennummer>/channel/<kanal>/state
```

Beispiel:

```text
hm485/LEQ0251870/channel/11/state
```

Ein späteres Umbenennen von `Kanal 11` in beispielsweise `Briefkasten` ändert den MQTT-Pfad **nicht**.

Auch die Home-Assistant-`unique_id` bleibt stabil. Der frei gewählte Name ist nur der Anzeigename.

## Home Assistant

Home Assistant Discovery wird automatisch aus Gerät, Seriennummer, Kanal und gewähltem semantischem Profil erzeugt.

Ab v0.7.51 enthält jedes per MQTT Discovery angelegte HM485-Gerät zusätzlich eine `configuration_url`. Home Assistant kann dadurch direkt auf die Weboberfläche des Gateways verlinken. Statt einer LAN- oder WLAN-IP wird der stabile mDNS-Name verwendet:

```text
http://hm485-gateway.local/
```

Bei geändertem Hostnamen entsprechend `http://<hostname>.local/`. Beim Wechsel zwischen Ethernet und WLAN bleibt die URL damit gleich. mDNS wird nach einem Wechsel des aktiven Netzwerkwegs erneut registriert. MQTT und die eigentliche Home-Assistant-Anbindung sind von einer funktionierenden `.local`-Namensauflösung unabhängig.

Beispiele für `device_class`:

| Profil | Home Assistant |
|---|---|
| Fenster | `window` |
| Tür | `door` |
| Alarm | `problem` |
| Kontakt | `opening` |

Nach einer Änderung von Name, Profil oder Invertierung wird Discovery erneut publiziert, ohne bewusst eine neue Entity-ID/`unique_id` zu erzeugen.

## Persistenz / NVS

Folgende Einstellungen bzw. Metadaten überleben Neustart und OTA:

- Netzwerk-/MQTT-Konfiguration
- Web-Zugang
- Gateway-HM485-Adresse
- Websprache
- Gerätenamen
- Kanalnamen
- semantische Kanalprofile
- Invertierung pro Kanal
- gecachte Geräte-Metadaten
- Registry der bekannten Geräteadressen

Beim Boot wird die bekannte Geräteliste wiederhergestellt. Geräte mit `activeReadSafe` werden nach der nativen Discovery gezielt abgefragt. Rein passive Geräte werden nicht blind angefragt, sondern bleiben bis zum nächsten gültigen Busframe als bekannt gespeichert.

Über **Gerät vergessen** kann ein Eintrag bewusst aus Runtime-Datenbank und NVS entfernt werden. Dabei werden auch die retained Home-Assistant-Discovery-Einträge und retained State-Topics dieses Geräts bereinigt.

## Backup und Restore

Ab v0.7.51 gibt es unter **Sicherung / Backup** einen Export und Import.

Der Export enthält:

- Gateway-Konfiguration
- WLAN-Konfiguration
- MQTT-Konfiguration
- Web-Zugang
- HM485-Adresse
- Sprache
- Namen/Profile/Invertierung der aktuell bekannten Geräte

Dateiformat:

```text
HM485GW_BACKUP_V1
```

### Sicherheitswarnung

Die Backup-Datei enthält WLAN-, MQTT- und Web-Passwörter in reversibler Form.

Sie sollte deshalb wie ein Passwort-Backup behandelt und nicht öffentlich abgelegt werden.

Nach einem Import startet das Gateway neu. Geräte werden anschließend weiterhin regulär per HM485-Discovery gefunden.

## Weboberfläche

Ab v0.7.51 gibt es eine gemeinsame Navigation für:

- Übersicht / Overview
- Konfiguration / Configuration
- Sicherung / Backup
- Diagnose / Diagnostics
- Firmware

Die Sprache kann unter Konfiguration zwischen **Deutsch** und **English** gewählt werden. Die Auswahl wird im NVS gespeichert.

Die Sprachauswahl beeinflusst ausschließlich die Weboberfläche. MQTT-Topics, Seriennummern und Home-Assistant-`unique_id` bleiben unverändert.

Der Button **Gateway neu starten / Restart gateway** befindet sich bewusst unter **Diagnose / Diagnostics** und nicht auf der Übersichtsseite.

Die Diagnoseseite zeigt zusätzlich Systeminformationen des ESP32, darunter Chipmodell und Revision, CPU-Takt/Kerne, SDK, freien und minimal freien Heap, größten freien Heap-Block, Flashgröße und -takt, Sketchgröße, freien OTA-Speicher, PSRAM (falls vorhanden), Reset-Grund und Uptime.

Der frühere passive **Discovery Analyzer** wurde ab v0.7.51 entfernt. Nachdem die native Discovery auf realer Hardware zuverlässig funktioniert, war dieser Forschungsmodus für den regulären Betrieb nicht mehr erforderlich. Die normale Native Discovery und der permanente RAW-RX-Only-Diagnosemodus bleiben erhalten.

## RAW RX Only

Der RAW-RX-Only-Modus ist ein permanentes Diagnose- und Sicherheitsfeature.

In diesem Modus:

- bleibt DIR hart auf Empfang
- HM485-TX ist vollständig blockiert
- Scan, Poll, ACK und Discovery sind deaktiviert
- der normale Parser wird umgangen
- UART-Rohdaten werden mit Timing protokolliert

Dieses Feature soll auch in zukünftigen Versionen erhalten bleiben.

## Netzwerk

### Ethernet

Ethernet ist der primäre Netzwerkpfad des WT32-ETH01.

LAN8720-Belegung:

| Funktion | GPIO |
|---|---:|
| PHY Clock Enable | GPIO16 |
| MDIO | GPIO18 |
| MDC | GPIO23 |
| REFCLK | GPIO0 |

### WLAN

Wenn Ethernet nicht verfügbar ist, kann WLAN als Fallback verwendet werden. Die Netzwerkparameter werden über die Weboberfläche gespeichert.

## Firmware-Update

Die Firmware kann über die Weboberfläche als Arduino/ESP32-`.bin` hochgeladen werden.

Der Webzugang ist durch HTTP Basic Auth geschützt.

Für Recovery sollte weiterhin die Möglichkeit zum seriellen Flashen erhalten bleiben.

## Sicherheitsphilosophie

Das Gateway ist nicht mehr grundsätzlich read-only, aber Schreibzugriffe bleiben **explizit auf bekannte und verifizierte Geräte-/Kanalprofile begrenzt**.

Aktuell gilt:

- kein generisches Schreiben auf unbekannte HM485/HBW-Geräte
- keine automatische Aktivierung experimenteller Aktorprotokolle
- EEPROM-Schreiben nur für ausdrücklich unterstützte und abgesicherte Konfigurationspfade
- RAW-RX-Only bleibt als harter Diagnosemodus mit vollständig gesperrtem HM485-TX erhalten
- bekannte passive HBW-Geräte werden nicht automatisch aktiv gepollt, solange der Read-Pfad nicht verifiziert ist

Die Entwicklung erfolgt weiterhin schrittweise anhand realer Busmitschnitte und Tests auf realer Hardware.

## Bekannte offene Punkte

- Langzeit-Test im produktiven Betrieb
- Geräte online/offline bzw. last-seen weiter ausbauen
- weitere HM485-/HBW-Geräteprofile real testen
- HBW-Sen-EP (`0x0084`): `53`-Polling auf realer Hardware verifizieren, reale Eingangszählung gegen die Pin-/Buskanal-Zuordnung testen und XML-/Source-Abweichungen bei den Sendeintervallen dokumentieren
- Backup/Restore bei zukünftigen NVS-Schemaänderungen weiter versionieren
- weitere Aktor-/EEPROM-Schreibpfade nur nach realer Protokollverifikation freigeben

## Versionsstand

Diese README beschreibt den Entwicklungsstand **v0.9.3g1**.

Für den HBW-Sen-EP wurden zusätzlich die FHEM-Dateien `hbw_sen_ep.xml` / `hbw_sen_ep.pm` sowie die vorliegenden Homebrew-Sourcen ausgewertet. Da die Sourcen experimentelle lokale Änderungen enthalten können, werden XML, Source und reale Busbeobachtung bewusst getrennt bewertet.

Die Software entstand iterativ aus Busmitschnitten, dem Verhalten von `hm485d`, FHEM-/HBW-Gerätebeschreibungen und Tests mit realer Homematic-Wired- und Homebrew-Hardware. Native Discovery, passive HBW-Erkennung, persistente Gerätedatenbank und RAW-RX-Only bleiben getrennte Bausteine mit unterschiedlichen Sicherheitsaufgaben.
