# Plan de prototype — Validation ESP-ADF
### Tester si ESP-ADF peut remplacer la stack squeezelite-esp32 + proxy HLS

**Matériel de prototype** : LilyGO TTGO T7 Mini32 V1.5 (ESP32-WROVER-B, 16MB flash / 8MB PSRAM)
**Objectif** : déterminer, par un prototype mesurable, si le framework ESP-ADF d'Espressif peut servir de base à Préset — en particulier valider que **HLS + décodage + A2DP source coexistent de façon stable**, sans qu'on ait à écrire le parseur HLS, le demuxer TS, ni la chaîne A2DP nous-mêmes.

**Ce n'est pas un produit. C'est une expérience pour trancher une décision d'architecture.**

---

## 1. La question à laquelle ce prototype répond

Une seule question, formulée pour être réfutable :

> **Sur un ESP32-WROVER, ESP-ADF peut-il lire un flux radio internet (HTTP/ICY ET HLS), le décoder, et le diffuser en A2DP vers un haut-parleur Bluetooth, de façon stable pendant 1h+, avec assez de marge mémoire et CPU pour ajouter ensuite captive portal, télémétrie, OTA et playlist?**

Si **oui** → on bascule l'architecture du projet vers ESP-ADF et on réécrit le plan principal.
Si **non** → on reste sur squeezelite-esp32 + proxy HLS server-side, comme précédemment planifié.

Tout le reste du prototype sert à répondre à cette question avec des preuves, pas des impressions.

---

## 2. Pourquoi ESP-ADF est un candidat sérieux

Les faits qui justifient l'investigation :

- **HLS est inclus officiellement.** L'exemple `pipeline_living_stream` démontre la lecture de radio internet en HLS temps réel. Le composant `http_stream` supporte HLS, y compris la rotation de clés AES-128.
- **A2DP source est inclus officiellement.** L'exemple `pipeline_a2dp_source_stream` a le pipeline : `sdcard → fatfs_stream → mp3_decoder → bt_stream → a2dp_source`. Notre besoin est ce pipeline avec la source SD remplacée par http/hls.
- **Codecs maintenus par Espressif** : MP3, AAC, FLAC, OGG, OPUS, AMR, TS. Pas de dépendance à RealNetworks/Helix non maintenu.
- **Apache 2.0** : compatible avec le modèle open source de Préset.
- **C'est le framework officiel du fabricant du chip** : argument de robustesse et de B2B fort.

**L'insight central** : notre produit est presque exactement `[source réseau] → décodeur → a2dp_source`. ESP-ADF a déjà les trois maillons. Le prototype vérifie surtout qu'ils tiennent ensemble avec WiFi + BT actifs simultanément.

---

## 3. Particularités du matériel de prototype

On prototype sur une carte qu'on a en main, avec ses contraintes. À comprendre avant de commencer.

### 3.1 La carte : TTGO T7 Mini32 V1.5

| Caractéristique | Valeur | Implication |
|---|---|---|
| Module | ESP32-WROVER-B | Même **famille** que le module cible (WROVER), mais chip plus ancien |
| PSRAM | 8MB | Suffisant — identique au module cible côté RAM |
| Flash | 16MB | Large — couvre même une partition A/B OTA si on veut la tester |
| Codec audio | **Aucun** | Pas de sortie I2S/jack utilisable — sans importance, voir plus bas |
| Lecteur SD | **Aucun** | Pas de source SD — on saute les étapes SD des exemples |

### 3.2 Trois conséquences à garder en tête

**a) WROVER-B ≠ WROVER-E (le module final).**
Le T7 utilise un WROVER-B, une révision de chip plus ancienne qui souffre d'un défaut de cache PSRAM. Il faut **compiler avec le correctif** :
```
-DBOARD_HAS_PSRAM
-mfix-esp32-psram-cache-issue
-mfix-esp32-psram-cache-strategy=memw
```
Conséquence pour le verdict : si on rencontre un bug PSRAM étrange sur le T7, vérifier s'il est spécifique au WROVER-B **avant** de blâmer ESP-ADF. Le WROVER-E final n'a pas ce défaut. Ne pas confondre un problème de chip ancien avec un problème de framework.

**b) Pas de codec audio = pas de sortie jack.**
Les exemples ESP-ADF comme `pipeline_living_stream` sortent par défaut sur le codec I2S du board LyraT. Le T7 n'en a pas. **Mais ça n'a presque aucune importance ici** : notre produit sort en **A2DP Bluetooth**, jamais en I2S. La sortie audio de validation, c'est le haut-parleur Bluetooth. On ne perd rien d'essentiel.

**c) Pas de SD = on saute les sources SD.**
Les exemples A2DP source lisent un MP3 depuis une carte SD. Le T7 n'a pas de lecteur. On saute cette étape et on attaque directement la source réseau. C'est en fait plus proche du produit final.

### 3.3 Module cible final — à commander en parallèle

**Commander un ESP32-WROVER-E-N8R8** (le module de production) si ce n'est pas déjà fait. Les mesures finales de mémoire et de stabilité (Phase C) devraient idéalement être refaites sur le module cible exact pour que les chiffres soient représentatifs. Le T7 V1.5 est un excellent proxy (même PSRAM 8MB, même famille WROVER), mais le WROVER-E a un chip plus récent — les chiffres définitifs doivent venir de lui.

**Statut du prototype** :
- Phases A à E : sur le T7 V1.5 (qu'on a en main, on démarre tout de suite)
- Re-validation Phase C : sur le WROVER-E quand il arrive

---

## 4. Ce qui pourrait faire échouer le prototype (hypothèses de risque)

Avant de coder, nommons ce qu'on cherche à casser. Ce sont les vraies inconnues.

| Risque | Pourquoi c'est incertain | Comment on le teste |
|---|---|---|
| **Coexistence WiFi + A2DP instable** | Le problème historique de l'ESP32. ADF ne le résout pas magiquement. | Phase B — écoute prolongée, mesure des coupures |
| **HLS + A2DP trop lourd ensemble** | HLS bufferise des segments (~400KB), A2DP demande du temps réel. | Phase C — mesure mémoire et CPU |
| **Latence HLS rédhibitoire** | HLS a 10-30s de latence intrinsèque. Acceptable pour radio? | Phase C — mesure du temps au premier son |
| **Mémoire insuffisante** | ADF a plus d'overhead que le code custom. | Toutes phases — heap_caps en continu |
| **Modèle pipeline trop rigide** | Si on doit insérer une opération non-ADF (télémétrie, hashing), c'est compliqué | Phase D — insertion d'un élément custom |
| **Doc ESP-ADF lacunaire sur points clés** | Réputation de doc incomplète | Tout du long — noter chaque recours au code source |
| **Branche v2.x vs master incompatibles** | ADF a divergé en 2024 | Phase A — choisir et figer la branche |
| **Défaut PSRAM du WROVER-B** | Chip ancien sur le T7 | Phase A — appliquer le cache fix, isoler de ADF |

Si l'un de ces risques se révèle bloquant et non contournable, le prototype a fait son travail : il nous a évité de bâtir le produit sur une fondation fragile.

---

## 5. Matériel et environnement

### 5.1 Matériel
- **1× TTGO T7 Mini32 V1.5** (en main) — carte de prototype
- **1× ESP32-WROVER-E-N8R8** (à commander) — module cible, pour re-valider Phase C
- **1× haut-parleur Bluetooth** simple (sink A2DP standard) — sortie audio de test
- **1× système audio de voiture réel** ou autoradio Bluetooth aftermarket — test réaliste (Phase E)
- **1× téléphone** configuré en hotspot — la connexion internet du produit
- **1× câble USB de données** (pas seulement charge) pour le T7 (puce CH9102 ou CP2104 selon la variante)
- Optionnel : **1× sonde de courant USB** pour mesurer la consommation

> Note : pas de board LyraT nécessaire. Comme la sortie est A2DP (Bluetooth) et non I2S (jack), le codec audio du LyraT ne servirait à rien pour notre cas. Le seul confort perdu est de pouvoir comparer avec le board de référence d'ESP-ADF en cas de bug obscur — on compensera par les logs.

### 5.2 Environnement logiciel
- **ESP-ADF** — figer une version. Recommandation : dernière release stable de la branche `release/v2.x` (compatible IDF v5.x). Documenter le commit exact.
- **ESP-IDF** — la version embarquée dans ESP-ADF (`$ADF_PATH/esp-idf`), pour éviter les incompatibilités.
- **Config board** : "ESP32 générique / custom" en menuconfig, AVEC le PSRAM cache fix (§3.2a). Ne pas sélectionner un board LyraT.
- **VS Code + extension Espressif IDF**, ou ligne de commande `idf.py`.
- Un dépôt git dédié au prototype, séparé du futur dépôt produit.

---

## 6. Stations de test (fixtures)

On teste contre des flux réels, choisis pour couvrir les cas qui comptent. Idéalement des stations québécoises/canadiennes pertinentes pour le produit.

| Type de flux | Exemple de source | Ce qu'on valide |
|---|---|---|
| MP3 Icecast classique | Une station communautaire qc en MP3 | Le cas le plus simple, baseline |
| AAC Icecast | Radio-Canada / ICI Musique (AAC) | Le codec dominant en radio moderne |
| HLS AAC | Une station qui diffuse en HLS (ex : certaines BBC, Radio France) | **Le cas critique de ce prototype** |
| HLS avec master playlist | BBC (master → child playlists) | L'indirection multi-bitrate |
| OPUS (si trouvable) | Radio France propose des flux Opus | Valider le codec Opus |
| Flux mono | Station parlée bas débit | Le cas mono → stéréo |
| Flux 48kHz | Beaucoup de flux AAC | Le resampling vers 44.1kHz |

On documentera les URLs exactes utilisées (`url_resolved` de radio-browser.info est la bonne source).

---

## 7. Phases du prototype

Chaque phase a un critère de réussite mesurable. On avance seulement si la phase précédente passe.

> **Note sur la validation audio sans codec** : le T7 n'ayant pas de jack, on ne peut pas "écouter" le décodage intermédiaire. On valide donc par deux moyens : (1) les logs ESP-ADF qui rapportent `sample_rate`, `codec type`, octets décodés, état du pipeline; (2) l'écoute finale via le haut-parleur Bluetooth, qui est de toute façon la vraie sortie du produit.

### Phase A — Mise en place et validation des radios isolées (2-3 jours)

**But** : faire tourner ESP-ADF sur le T7, et valider séparément WiFi et A2DP **avant** de les combiner. Isoler chaque radio évite de débattre plus tard si un problème de coexistence vient du WiFi, du BT, ou de leur interaction.

Étapes :

**A.1 — Toolchain et compilation**
1. Installer ESP-ADF, figer la version, documenter le commit
2. Configurer pour le T7 (ESP32 générique + PSRAM cache fix, §3.2a)
3. Compiler `pipeline_a2dp_source_stream` tel quel (même si on n'a pas de SD pour le faire jouer) — objectif : valider que le toolchain compile et flashe sur le T7

**Critère A.1** : ✅ Le projet compile et flashe sans erreur sur le T7. PSRAM détectée au boot (vérifier le log `Found ... external RAM`).

**A.2 — A2DP source seul**
1. Faire tourner l'exemple A2DP source (ou une version minimale)
2. Valider qu'il scanne et se connecte au haut-parleur Bluetooth
3. Confirmer la connexion dans les logs (`A2DP connection state = CONNECTED`)

**Critère A.2** : ✅ Le T7 découvre et se connecte à un haut-parleur Bluetooth en mode A2DP source. Connexion stable plusieurs minutes.

**A.3 — WiFi STA seul**
1. Un exemple WiFi basique (connexion STA au hotspot du téléphone)
2. Valider l'obtention d'une IP, la stabilité de connexion

**Critère A.3** : ✅ Le T7 se connecte au hotspot, obtient une IP, reste connecté plusieurs minutes.

**Si échec en A** : problème de toolchain, de config board, ou de PSRAM fix. Pas encore un verdict sur ESP-ADF. Débugger avant d'avancer. Si le PSRAM ne se détecte pas, vérifier les flags de compilation avant tout.

---

### Phase B — La fusion critique : réseau → A2DP (2-3 jours)

**But** : combiner WiFi et A2DP en un seul pipeline. Source réseau → décodeur → A2DP source. C'est LE test de coexistence — le cœur du verdict.

Pipeline cible :
```
http_stream/hls_stream → decoder (auto) → [resample] → a2dp_stream → a2dp_source
```

Étapes :
1. Partir de `pipeline_a2dp_source_stream`
2. Remplacer `fatfs_stream` (lecture SD) par `http_stream` (lecture réseau)
3. Connecter au WiFi (STA, hotspot) **avant** d'initialiser le pipeline audio
4. Tester d'abord avec un flux **MP3 Icecast** (le plus simple)
5. Puis un flux **AAC Icecast**
6. Puis un flux **HLS**

**Critères de réussite** :
- ✅ Un flux MP3 internet joue en A2DP, audible sur le haut-parleur, sans coupure pendant 10 min
- ✅ Un flux AAC internet joue en A2DP, idem
- ✅ **Un flux HLS joue en A2DP** — le test décisif
- ✅ WiFi STA + A2DP coexistent (pas de déconnexion en boucle, pas de reset)

**Si échec ici** : c'est le verdict central. Si HLS+A2DP ne coexistent pas sur ESP-ADF malgré le PSRAM et la config correcte, alors ESP-ADF ne résout pas notre problème mieux que le custom, et on revient au proxy server-side. **Documenter précisément le mode d'échec** (coupures? reset? déconnexion WiFi? underrun audio?).

> Rappel WROVER-B : si l'instabilité semble liée à la PSRAM, tester si le cache fix est bien appliqué et noter que le WROVER-E final pourrait se comporter différemment. Ne pas conclure no-go sur la seule base du T7 si le mode d'échec pointe vers le chip ancien.

---

### Phase C — Mesures de stabilité et de ressources (2-3 jours)

**But** : quantifier. Pas "ça marche" mais "ça marche avec X% de marge."

Étapes et mesures :
1. **Test d'endurance 1h+** sur flux AAC Icecast — compter les coupures audio
2. **Test d'endurance 1h+** sur flux HLS — compter les coupures, observer la stabilité du buffering de segments
3. **Mémoire continue** : logger `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` et `MALLOC_CAP_SPIRAM` toutes les 30s. Chercher les fuites (tendance à la baisse sur 1h).
4. **CPU** : `vTaskGetRunTimeStats` pour la charge par task et par core. Identifier la marge restante.
5. **Latence de démarrage** : temps entre "ouverture du flux" et "premier son" — pour HTTP et HLS séparément.
6. **Latence de changement de station** : simuler un changement de flux, mesurer le gap.

**Critères de réussite** :
- ✅ Moins de 2 coupures audibles par heure sur AAC Icecast
- ✅ Moins de 5 coupures audibles par heure sur HLS
- ✅ Mémoire stable (pas de fuite sur 1h)
- ✅ **Au moins 30% de RAM interne libre et 1.5MB+ de PSRAM libre** en régime — la marge pour ajouter le reste du produit
- ✅ **Au moins 20% de CPU libre** sur chaque core en régime
- ✅ Latence de démarrage HTTP < 5s, HLS < 15s (acceptable pour radio)

**Si la marge est insuffisante** (ex : 5% de RAM libre) : ESP-ADF fait le travail mais ne laisse pas de place pour captive portal + télémétrie + OTA. Verdict nuancé : possible mais serré, à peser.

**À refaire sur le WROVER-E** : ces mesures, une fois la carte cible reçue, pour des chiffres représentatifs du produit final.

---

### Phase D — Test d'extensibilité (2 jours)

**But** : vérifier qu'on peut greffer NOS fonctionnalités sur le modèle pipeline d'ESP-ADF. C'est le risque "modèle trop rigide."

Étapes :
1. **Insérer un élément custom** dans le pipeline (ex : un passe-through qui compte les octets, simulant un futur point de télémétrie ou de hashing). Valide qu'on peut intercepter le flux.
2. **Changer de flux à chaud** : implémenter un changement de station (arrêt pipeline, nouvelle URL, redémarrage) et mesurer si A2DP reste connecté pendant l'opération.
3. **Coexistence avec un serveur HTTP** : lancer un `esp_http_server` minimal (le futur captive portal / page de config) en parallèle du pipeline audio. Vérifier qu'ils cohabitent sans tuer l'audio.
4. **AVRCP** : vérifier que le `a2dp_stream` d'ESP-ADF expose les commandes AVRCP (play/pause/next/prev) dont on a besoin pour la navigation playlist.

**Critères de réussite** :
- ✅ Un élément custom peut être inséré dans le pipeline
- ✅ Le changement de station garde le lien A2DP vivant (ou le rétablit en < 3s)
- ✅ Un serveur HTTP tourne en parallèle sans tuer l'audio
- ✅ Les commandes AVRCP arrivent jusqu'à notre code

**Si échec ici** : ESP-ADF fait le cœur audio mais résiste à nos extensions. Verdict : utilisable pour l'audio, mais on devra peut-être mélanger ADF (audio) et IDF nu (le reste), ce qui ajoute de la complexité. À peser.

---

### Phase E — Test en conditions réelles (1 jour)

**But** : sortir du banc, aller dans une vraie voiture.

Étapes :
1. Alimenter le T7 par USB dans une vraie voiture
2. Pairer avec le système Bluetooth réel de la voiture (pas un haut-parleur de test)
3. Conduire 30+ minutes avec un flux HLS et un flux Icecast
4. Tester : démarrage, coupures de tunnel (perte réseau), reprise, qualité audio sur le système de la voiture
5. Tester les boutons du volant (play/pause/next) si AVRCP a passé en Phase D

**Critères de réussite** :
- ✅ Audio de qualité acceptable sur le système réel de la voiture
- ✅ Une perte réseau (tunnel) est suivie d'une reprise automatique
- ✅ Le pairing avec un vrai système auto fonctionne (pas juste un haut-parleur de test)

**Pourquoi cette phase** : les haut-parleurs de test et les systèmes auto réels se comportent différemment en AVRCP et en gestion de connexion. Un prototype qui marche sur le banc mais pas dans une vraie voiture n'a pas validé le vrai cas d'usage.

---

## 8. Livrable du prototype

À la fin, un document court (2-4 pages) qui répond à la question de la section 1, avec :

1. **Verdict** : ESP-ADF go / no-go, avec le niveau de confiance
2. **Tableau de mesures** : mémoire libre, CPU libre, coupures/heure, latences — chiffres réels (T7, et WROVER-E si reçu)
3. **Liste des points de friction** rencontrés (doc manquante, config obscure, bugs ADF, défaut PSRAM WROVER-B)
4. **Modes d'échec observés**, s'il y en a
5. **Recommandation d'architecture** :
   - Soit « basculer le plan principal vers ESP-ADF »
   - Soit « rester sur squeezelite-esp32 + proxy HLS »
   - Soit « hybride » (ADF pour l'audio, IDF pour le reste) si Phase D révèle des limites
6. **Estimation révisée** de l'effort produit selon le verdict
7. **Note WROVER-B vs WROVER-E** : tout écart observé attribuable au chip de prototype, à re-tester sur le module cible

Ce document remplace les spéculations actuelles par des faits mesurés.

---

## 9. Ce que le prototype NE fait PAS

Important pour ne pas dériver vers un produit déguisé en prototype :

| On ne fait pas | Pourquoi |
|---|---|
| Captive portal complet | Phase D teste juste qu'un serveur HTTP coexiste. Le portail vient après le verdict. |
| Provisioning série / install page | Hors scope. Le prototype est flashé manuellement. |
| Télémétrie complète | Phase D teste juste qu'on peut intercepter le flux. Pas de backend. |
| OTA | Hors scope. Validé séparément une fois l'architecture choisie. |
| Playlist de 5 stations | Phase D teste juste le changement de flux. La gestion playlist vient après. |
| Personnalisation / branding | Aucun rapport avec la question d'architecture. |
| Optimisation poussée | On mesure la marge brute. L'optimisation vient avec le produit. |
| Joliesse du code | C'est jetable. Le code propre vient avec le produit. |
| Portage sur WROVER-E | Sauf la re-validation Phase C. Le gros du test reste sur le T7. |

Le prototype répond à UNE question. Tout ajout qui ne sert pas cette question est une distraction.

---

## 10. Estimation et calendrier

| Phase | Durée | Cumulé |
|---|---|---|
| A — Setup + radios isolées (WiFi, A2DP séparément) | 2-3 jours | 3 j |
| B — Fusion réseau → A2DP (test critique) | 2-3 jours | 6 j |
| C — Mesures stabilité + ressources | 2-3 jours | 9 j |
| D — Extensibilité | 2 jours | 11 j |
| E — Conditions réelles | 1 jour | 12 j |
| Rédaction du verdict | 0.5 jour | 12.5 j |
| **Total** | | **~2.5 semaines** |

Avec une marge pour les imprévus (toolchain, config T7 non standard, PSRAM cache fix, doc ADF lacunaire), prévoir **3 semaines calendaires**.

**Point de sortie anticipé** : si la Phase B échoue de façon non contournable (HLS+A2DP ne coexistent pas, et pas à cause du WROVER-B), le prototype s'arrête là — verdict no-go en ~6 jours, on revient au plan proxy. C'est un succès du prototype : avoir tranché vite et pas cher.

---

## 11. Décisions qui dépendent du verdict

Selon le résultat, voici ce qui change dans les plans existants :

**Si ESP-ADF est retenu** :
- Le plan principal (`esp32-radio-bt-dev-plan.md`) est réécrit : §3 (pourquoi ESP-IDF) devient « pourquoi ESP-ADF », §9-10 (pipeline audio) adoptent le modèle pipeline ADF, §13 (composants) change, §18 (codecs) se simplifie (codecs ADF inclus), les références squeezelite-esp32 sont remplacées par les exemples ADF
- HLS est inclus nativement → la question du proxy server-side disparaît
- Opus, FLAC, etc. deviennent gratuits → on peut élargir la compatibilité stations
- Le plan télémétrie / install / Phase 2 restent largement valides (indépendants du framework audio)

**Si ESP-ADF est rejeté** :
- On garde le plan principal actuel (squeezelite-esp32 comme référence, Helix + Opus)
- On planifie le proxy HLS server-side comme sous-projet (document séparé à rédiger)
- On documente pourquoi ESP-ADF n'a pas convenu (utile pour ne pas y revenir sans raison)

**Si verdict hybride** :
- ADF pour le cœur audio, IDF nu pour captive portal / télémétrie / OTA
- Plan principal révisé pour refléter la frontière entre les deux
- Plus complexe, à peser sérieusement contre les deux options pures

---

## 12. Démarrage immédiat — check-list

Ce qu'on peut faire dès aujourd'hui avec le T7 en main :

- [ ] Confirmer câble USB de données (pas charge seule) + drivers CH9102/CP2104 installés
- [ ] Installer ESP-ADF, figer la version, noter le commit
- [ ] Configurer le build pour ESP32 générique + PSRAM cache fix
- [ ] Commander le WROVER-E-N8R8 en parallèle (pour re-validation Phase C plus tard)
- [ ] Identifier 2-3 flux de test : un MP3 Icecast, un AAC Icecast, un HLS (idéalement stations cibles réelles)
- [ ] Avoir un haut-parleur Bluetooth de test sous la main
- [ ] Créer le dépôt git du prototype
- [ ] Lancer Phase A.1 (compilation + flash + détection PSRAM)

---

## 13. Questions ouvertes

- **Quelle version d'ESP-ADF figer?** Recommandation : dernière release stable `release/v2.x`. À confirmer en regardant les notes de version récentes.
- **A-t-on accès à des flux HLS de stations réellement cibles?** Idéalement tester contre les flux qu'on voudra vraiment servir. Sinon, BBC/Radio France comme proxy.
- **Qui exécute le prototype?** Si c'est toi, le calendrier dépend de ta disponibilité. Si c'est délégué, il faut quelqu'un à l'aise avec ESP-IDF/ADF.
- **Le WROVER-E arrive quand?** Tant qu'il n'est pas là, Phases A-E tournent sur le T7. La re-validation Phase C attend la carte cible — ne pas bloquer le reste pour ça.
