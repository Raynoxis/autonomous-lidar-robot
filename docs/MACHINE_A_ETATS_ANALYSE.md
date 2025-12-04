# Machine à États - Analyse Complète et Plan d'Implémentation

**Date**: 2025-12-04
**Robot**: MakersPet Mini (120mm)
**ROS2**: Iron
**Conteneur**: makerspet-mini-web:latest

---

## Table des Matières

1. [Résumé Exécutif](#résumé-exécutif)
2. [Machine à États Définitive](#machine-à-états-définitive)
3. [Chronologie Réelle Observée](#chronologie-réelle-observée)
4. [Erreurs Corrigées](#erreurs-corrigées)
5. [Modifications Requises du Code](#modifications-requises-du-code)
6. [Paramètres Officiels KaiAI](#paramètres-officiels-kaiaai)
7. [Diagramme de Transitions](#diagramme-de-transitions)

---

## Résumé Exécutif

### Tests Effectués

| Test | Résultat | Durée | Observations |
|------|----------|-------|--------------|
| ✅ Démarrage conteneur | Succès | 25s | Nav2 auto-activé, robot détecté @ 5.04 Hz |
| ❌ Exploration #1-3 | Échec | 9-14s | **MAUVAIS PARAMÈTRES** (costmap, frame, frontier_size) |
| ✅ Exploration #4 | Succès | 45s+ | **BONS PARAMÈTRES** - Navigation continue |
| ✅ Navigation manuelle | Succès | 11s | Goal succeeded à (0.5, 0.5) |

### Problèmes Identifiés et Résolus

1. ❌ **explore_lite plantait systématiquement** → ✅ **Paramètres corrigés**
2. ❌ **API ROS2 Control plantait** → ⚠️ **Non testé après correction**
3. ✅ **Nav2 fonctionne parfaitement**
4. ✅ **SLAM fonctionne**
5. ✅ **Robot détection automatique fonctionne**

---

## Machine à États Définitive

### États Principaux (10 états)

```
┌─────────────────────────────────────────────────────────────────┐
│                    SÉQUENCE DE DÉMARRAGE                         │
├─────────────────────────────────────────────────────────────────┤
│ 1. initial               Pas de connexion WebSocket             │
│ 2. connecting_ws         Connexion RosBridge en cours           │
│ 3. ws_connected          WebSocket OK, vérif Nav2 en cours      │
│ 4. container_ready       Nav2 actif (≥3 nodes), attente robot   │
│ 5. robot_ready           Système complet opérationnel           │
├─────────────────────────────────────────────────────────────────┤
│                      MODES OPÉRATIONNELS                         │
├─────────────────────────────────────────────────────────────────┤
│ 6. exploring             Exploration autonome active            │
│ 7. navigating            Navigation vers goal en cours          │
├─────────────────────────────────────────────────────────────────┤
│                       ÉTATS D'ERREUR                             │
├─────────────────────────────────────────────────────────────────┤
│ 8. ws_error              Erreur connexion WebSocket             │
│ 9. container_error       Nav2 nodes insuffisants                │
│ 10. robot_lost           Perte données robot (temporaire)       │
└─────────────────────────────────────────────────────────────────┘
```

### Détail de Chaque État

#### 1. `initial`
- **Description**: État de départ, pas de connexion
- **Durée**: Indéfini (jusqu'à action utilisateur)
- **Conditions d'entrée**:
  - Au lancement de l'application
  - Après un `disconnect()` complet
- **Conditions de sortie**:
  - Utilisateur clique "Connect"
- **Indicateurs visuels**:
  - Couleur: `⚪ Gris` (`text-gray-400`)
  - Animation: Aucune
  - Message: "Système prêt - Cliquez sur Connect"
- **Boutons actifs**: `Connect` uniquement
- **Données système**:
  - `connected: false`
  - Tous les topics: `false`
  - Tous les nodes: `false`

---

#### 2. `connecting_ws`
- **Description**: Tentative de connexion au RosBridge WebSocket
- **Durée observée**: 1-2 secondes
- **Conditions d'entrée**:
  - Action: `connect()` appelée
  - Depuis: `initial`
- **Conditions de sortie**:
  - ✅ Succès → `ws_connected`
  - ❌ Échec → `ws_error`
- **Indicateurs visuels**:
  - Couleur: `🟡 Jaune` (`text-yellow-500`)
  - Animation: `animate-pulse`
  - Message: "Connexion au RosBridge WebSocket..."
  - Barre de progression: 20%
- **Boutons actifs**: Aucun (en cours)
- **Actions déclenchées**:
  - `rosService.connect()` lancé
  - Monitoring désactivé

---

#### 3. `ws_connected`
- **Description**: WebSocket connecté, vérification des nodes Nav2
- **Durée observée**: 2-5 secondes
- **Conditions d'entrée**:
  - WebSocket connection établie
  - Callback `onConnected` déclenché
  - Depuis: `connecting_ws`
- **Conditions de sortie**:
  - ✅ Nav2 ready (≥3 critical nodes) → `container_ready`
  - ❌ Timeout ou erreur → `container_error`
- **Indicateurs visuels**:
  - Couleur: `🟡 Jaune` (`text-yellow-500`)
  - Animation: `animate-pulse`
  - Message: "RosBridge OK - Vérification Nav2..."
  - Détails: "Détection des nodes Nav2... (X/4 actifs)"
  - Barre de progression: 40%
- **Boutons actifs**: `Disconnect`, `Emergency Stop`
- **Actions déclenchées**:
  - `startMonitoring()` lancé
  - `subscribeToTopics()` lancé
  - Check nodes toutes les 500ms
- **Nodes critiques surveillés**:
  ```javascript
  ['/slam_toolbox', '/bt_navigator', '/controller_server', '/planner_server']
  ```

---

#### 4. `container_ready`
- **Description**: Nav2 opérationnel, en attente du robot
- **Durée observée**: 10-20 secondes (dépend du boot du robot)
- **Conditions d'entrée**:
  - ≥3 critical Nav2 nodes actifs
  - Depuis: `ws_connected`
- **Conditions de sortie**:
  - ✅ Robot détecté (scan + battery) → `robot_ready`
  - ❌ Perte Nav2 nodes → `container_error`
- **Indicateurs visuels**:
  - Couleur: `🟡 Jaune` (`text-yellow-500`)
  - Animation: `animate-pulse`
  - Message: "Nav2 opérationnel - En attente du robot..."
  - Détails: "Nav2: 4/4 • Robot: 0/2 nodes • Attente /scan..."
  - Barre de progression: 60%
- **Boutons actifs**: `Disconnect`, `Emergency Stop`
- **Flags surveillés**:
  - `scanDataReceived: false`
  - `batteryDataReceived: false`
  - `nodes['/kaiaai_telemetry_node']: false`
- **Critères de détection robot**:
  ```javascript
  scanDataReceived && batteryDataReceived && nodes['/kaiaai_telemetry_node']
  ```

---

#### 5. `robot_ready`
- **Description**: Système complètement opérationnel
- **Durée**: Indéfinie (état stable)
- **Conditions d'entrée**:
  - Nav2 actif (4/4 nodes)
  - Robot détecté (scan @ 5 Hz + battery)
  - Depuis: `container_ready`
- **Conditions de sortie**:
  - `startExploration()` → `exploring`
  - `sendNavigationGoal()` → `navigating`
  - Perte robot → `robot_lost`
  - Perte Nav2 → `container_error`
- **Indicateurs visuels**:
  - Couleur: `🟢 Vert` (`text-green-500`)
  - Animation: Aucune
  - Message: "Système opérationnel - Prêt à naviguer"
  - Détails: "Robot OK • Nav2: 4/4 • Topics: 3/3 actifs"
  - Barre de progression: 100%
- **Boutons actifs**: TOUS
  - Connect: ❌ disabled
  - Disconnect: ✅
  - Emergency Stop: ✅
  - Joystick: ✅
  - Start Explore: ✅
  - Stop Explore: ❌ disabled
  - Save/Load/Clear Map: ✅
  - Set Home: ✅
  - Navigation (clic carte): ✅
- **Données système complètes**:
  ```javascript
  {
    connected: true,
    scanDataReceived: true,
    batteryDataReceived: true,
    nodes: {
      '/slam_toolbox': true,
      '/bt_navigator': true,
      '/controller_server': true,
      '/planner_server': true,
      '/kaiaai_telemetry_node': true,
      '/robot_state_publisher': true
    },
    topics: {
      '/scan': true,
      '/map': true,
      '/odom': true,
      '/cmd_vel': true,
      '/battery_state': true
    }
  }
  ```

---

#### 6. `exploring`
- **Description**: Exploration autonome active (explore_lite)
- **Durée**: Variable (jusqu'à arrêt manuel ou fin auto)
- **Conditions d'entrée**:
  - Action: `startExploration()` appelée
  - `/explore_node` actif
  - Depuis: `robot_ready`
- **Conditions de sortie**:
  - ✅ `stopExploration()` manuel → `robot_ready`
  - ✅ Exploration terminée auto → `robot_ready`
  - ❌ Perte robot → `robot_lost`
  - ❌ `/explore_node` crash → `robot_ready` (avec log erreur)
- **Indicateurs visuels**:
  - Couleur: `🟢 Vert` (`text-green-500`)
  - Animation: `animate-pulse`
  - Message: "Exploration autonome en cours..."
  - Détails: "Explore ✓ • Nav2: 4/4 • Goal actif"
  - Barre de progression: 100% (pulse)
- **Boutons actifs**:
  - Connect: ❌
  - Disconnect: ✅
  - Emergency Stop: ✅
  - Joystick: ❌ (robot en autonomie)
  - Start Explore: ❌
  - **Stop Explore**: ✅ (highlighted)
  - Save Map: ✅
  - Navigation manuelle: ❌
- **Monitoring spécifique**:
  - Vérifier présence `/explore_node` toutes les 500ms
  - Surveiller logs de navigation
  - Détecter "All frontiers traversed" ou "No frontiers found"

---

#### 7. `navigating`
- **Description**: Navigation vers un goal manuel
- **Durée**: Variable (dépend de la distance)
- **Conditions d'entrée**:
  - Action: `sendNavigationGoal(x, y)` appelée
  - Goal accepté par bt_navigator
  - Depuis: `robot_ready`
- **Conditions de sortie**:
  - ✅ Goal succeeded → `robot_ready`
  - ✅ Goal canceled → `robot_ready`
  - ❌ Goal failed → `robot_ready` (avec log erreur)
  - ❌ Perte robot → `robot_lost`
- **Indicateurs visuels**:
  - Couleur: `🟢 Vert` (`text-green-500`)
  - Animation: `animate-pulse`
  - Message: "Navigation vers objectif..."
  - Détails: "Destination: (X.XX, Y.YY) • Distance: Z.Zm"
  - Barre de progression: Calculée selon distance
- **Boutons actifs**:
  - Connect: ❌
  - Disconnect: ✅
  - Emergency Stop: ✅
  - Joystick: ❌
  - Start Explore: ❌
  - Cancel Goal: ✅ (highlighted)
  - Save Map: ✅
  - Nouveau goal: ✅ (annule l'ancien)
- **Monitoring spécifique**:
  - Écouter feedback de `/navigate_to_pose` action
  - Calculer distance restante
  - Détecter "Goal succeeded/canceled/failed"

---

#### 8. `ws_error`
- **Description**: Erreur de connexion WebSocket
- **Durée**: Permanente (nécessite reconnexion)
- **Conditions d'entrée**:
  - Erreur WebSocket détectée
  - Callback `onError` ou `onClose` déclenché
  - Depuis: N'importe quel état connecté
- **Conditions de sortie**:
  - `disconnect()` puis `connect()` → `connecting_ws`
- **Indicateurs visuels**:
  - Couleur: `🔴 Rouge` (`text-red-500`)
  - Animation: Aucune
  - Message: "Erreur RosBridge - Connexion perdue"
  - Détails: "Vérifiez que le serveur est accessible"
  - Barre de progression: 0%
- **Boutons actifs**:
  - Disconnect: ✅
  - Connect: ✅ (pour réessayer)
  - Tous les autres: ❌
- **Actions auto**:
  - `stopMonitoring()`
  - Réinitialisation des flags

---

#### 9. `container_error`
- **Description**: Nav2 nodes insuffisants ou crashés
- **Durée**: Permanente (nécessite reconnexion ou redémarrage conteneur)
- **Conditions d'entrée**:
  - Nombre de critical nodes < 3
  - Depuis: `ws_connected` ou `container_ready`
- **Conditions de sortie**:
  - Reconnexion → `connecting_ws`
  - Redémarrage conteneur externe
- **Indicateurs visuels**:
  - Couleur: `🔴 Rouge` (`text-red-500`)
  - Animation: Aucune
  - Message: "Erreur conteneur - Nodes Nav2 manquants"
  - Détails: "Nav2: X/4 actifs • Redémarrage requis"
  - Barre de progression: 40% (bloqué)
- **Boutons actifs**:
  - Disconnect: ✅
  - Connect: ✅ (réessayer)
  - Tous les autres: ❌
- **Cause possible**:
  - Conteneur en cours de démarrage (< 30s après boot)
  - Crash d'un node Nav2
  - Ressources insuffisantes

---

#### 10. `robot_lost`
- **Description**: Perte temporaire des données robot
- **Durée**: Variable (récupérable automatiquement)
- **Conditions d'entrée**:
  - `scanDataReceived: false` OU `batteryDataReceived: false`
  - Depuis: `robot_ready`, `exploring`, ou `navigating`
- **Conditions de sortie**:
  - ✅ Robot revient (scan + battery OK) → `robot_ready`
  - ❌ Timeout > 60s → Rester dans cet état
- **Indicateurs visuels**:
  - Couleur: `🟠 Orange` (`text-orange-500`)
  - Animation: `animate-pulse` (pour montrer l'attente)
  - Message: "Robot déconnecté - Reconnexion..."
  - Détails: "Vérifiez l'alimentation et le WiFi du robot"
  - Barre de progression: 60% (dégradé)
- **Boutons actifs**:
  - Disconnect: ✅
  - Emergency Stop: ✅ (au cas où)
  - Tous les autres: ❌
- **Actions auto**:
  - Annuler navigation/exploration en cours
  - `cancelNavigation()` si goal actif
  - `stopExploration()` si explore actif
  - Publier `cmd_vel` à 0 (arrêt sécurisé)
- **Récupération auto**:
  - Monitoring continue toutes les 100ms
  - Dès que données reviennent → `robot_ready`

---

## Chronologie Réelle Observée

### Démarrage Complet (T=0 → T=25s)

```
T+0s    │ Container start
        │ ├─ Web Server started (port 8082)
        │ ├─ Micro-ROS Agent started (port 8888)
        │ ├─ Telemetry Node started
        │ ├─ Robot State Publisher started
        │ └─ Nav2 + SLAM + RosBridge launching...
        │
T+5s    │ STATE: initial → connecting_ws (user clicks Connect)
        │ WebSocket connection attempt...
        │
T+7s    │ STATE: connecting_ws → ws_connected
        │ ✓ RosBridge WebSocket connected
        │ ✓ Topics subscribed (/tf, /map, /scan, /battery_state)
        │ ✓ Monitoring started (100ms topics, 500ms nodes, 200ms state)
        │
T+10s   │ STATE: ws_connected → container_ready
        │ ✓ Nav2 nodes detected: 4/4
        │   - /slam_toolbox
        │   - /bt_navigator (state: active [3])
        │   - /controller_server
        │   - /planner_server
        │ ⏳ Waiting for robot...
        │
T+15s   │ ✓ /scan topic detected
        │ ⏳ Waiting for scan data...
        │
T+20s   │ ✓ LiDAR data received: LDROBOT-LD14P @ 5.04 Hz
        │ ✓ Battery data received
        │ ✓ /kaiaai_telemetry_node active
        │
T+25s   │ STATE: container_ready → robot_ready
        │ ✅ SYSTEM READY!
        │ Active nodes: 27
        │ Active topics: /scan, /map, /odom, /cmd_vel, /battery_state
```

### Exploration Test #4 (SUCCÈS)

```
T+0s    │ STATE: robot_ready → exploring
        │ Command: ros2 run explore_lite explore --ros-args
        │   -p robot_base_frame:=base_link
        │   -p costmap_topic:=map
        │   -p costmap_updates_topic:=map_updates
        │   -p planner_frequency:=0.15
        │   -p min_frontier_size:=0.75
        │
T+0.4s  │ ✓ /explore_node created (29 nodes total)
        │ ⏳ Waiting for costmap topic: map
        │
T+3.4s  │ ✓ Costmap received
        │ ✓ Connected to Nav2 server
        │
T+5s    │ ✓ First frontier detected
        │ ✓ Goal sent to bt_navigator
        │
T+5-45s │ 🟢 NAVIGATION ACTIVE
        │ Controller passing new path every 1s
        │ Robot moving autonomously
        │ SLAM building map
        │
T+45s   │ ✓ Goal succeeded
        │ ⏳ Searching for next frontier...
        │
T+XX    │ User: pkill -9 explore_lite
        │ STATE: exploring → robot_ready
        │ ✓ Exploration stopped
```

### Navigation Manuelle (SUCCÈS)

```
T+0s    │ STATE: robot_ready → navigating
        │ Current position: x=-0.312, y=0.583, yaw=-165°
        │ Goal: x=0.5, y=0.5
        │ Distance: ~0.9m
        │
T+0.1s  │ ✓ Action server accepted goal
        │ ✓ Global planner computing path...
        │
T+3s    │ ✓ Path computed
        │ ✓ Controller started
        │ Controller passing new path every 1s
        │
T+3-11s │ 🟢 NAVIGATION ACTIVE
        │ Robot moving toward goal
        │ Path updates: T+4s, T+5s, T+6s, T+7s, T+8s, T+9s, T+10s
        │
T+11s   │ ✓ Goal succeeded
        │ STATE: navigating → robot_ready
        │ Final position: near (0.5, 0.5)
```

---

## Erreurs Corrigées

### 1. Paramètres explore_lite INCORRECTS ❌ → ✅

**Problème**: Exploration plantait systématiquement après 9-14 secondes

**Tests échoués #1-3** :
```bash
# ❌ MAUVAIS PARAMÈTRES
ros2 run explore_lite explore --ros-args \
  -p robot_base_frame:=base_footprint \        # FAUX
  -p costmap_topic:=/global_costmap/costmap \  # FAUX
  -p costmap_updates_topic:=/global_costmap/costmap_updates \  # FAUX
  -p planner_frequency:=0.33 \                 # FAUX
  -p min_frontier_size:=0.5                    # FAUX
```

**Résultat** :
- "All frontiers traversed/tried out" après 9-14s
- "No frontiers found" (test #3)
- Robot bougeait brièvement puis s'arrêtait

**Cause** :
- `robot_base_frame: base_footprint` au lieu de `base_link`
- `costmap_topic: /global_costmap/costmap` (Nav2) au lieu de `map` (SLAM)
- `planner_frequency: 0.33` trop élevé (officiel: 0.15)
- `min_frontier_size: 0.5` trop petit (officiel: 0.75)

**Solution - Test #4 SUCCÈS** ✅ :
```bash
# ✅ BONS PARAMÈTRES (source: github.com/kaiaai/m-explore-ros2)
ros2 run explore_lite explore --ros-args \
  -p robot_base_frame:=base_link \
  -p costmap_topic:=map \
  -p costmap_updates_topic:=map_updates \
  -p visualize:=true \
  -p planner_frequency:=0.15 \
  -p progress_timeout:=30.0 \
  -p min_frontier_size:=0.75
```

**Référence officielle** :
- Fichier: `https://raw.githubusercontent.com/kaiaai/m-explore-ros2/main/explore/config/params.yaml`

---

### 2. API ROS2 Control (ros_api.py) - Erreur HTTP ❌

**Problème détecté** :
```python
# Ligne 33 dans ros_api.py
content_length = int(self.headers['Content-Length'])
# TypeError: int() argument must be a string, not 'NoneType'
```

**Cause** :
- Requête POST sans `Content-Length` header
- Curl par défaut n'envoie pas ce header si pas de body

**Impact** :
- `/api/explore/start` retourne "Empty reply from server"
- Impossible d'utiliser l'API

**Solution requise** :
```python
# À corriger dans ros_api.py
def do_POST(self):
    # Avant (ligne 33) :
    content_length = int(self.headers['Content-Length'])

    # Après :
    content_length = int(self.headers.get('Content-Length', 0))
    if content_length > 0:
        post_data = self.rfile.read(content_length)
    else:
        post_data = b''
```

**Test de validation** :
```bash
curl -X POST http://192.168.0.10:8083/api/explore/start
# Devrait retourner: {"success": true, "data": {"pid": 12345}}
```

---

### 3. États "exploration_error" et "exploration_available" - Inutiles ❌

**Problème** :
- `exploration_error`: Jamais utilisé dans le code
- `exploration_available`: Redondant avec `robot_ready` + check `mapData !== null`

**Analyse** :
```typescript
// État actuel dans types/state.ts
export const SystemState = {
  // ...
  EXPLORATION_AVAILABLE: 'exploration_available',  // ❌ Supprimable
  EXPLORING: 'exploring',
  EXPLORATION_ERROR: 'exploration_error',          // ❌ Supprimable
}
```

**Solution** :
- Supprimer ces 2 états
- Gérer l'exploration depuis `robot_ready` directement
- Transition directe : `robot_ready → exploring`
- Si explore_node crash : rester dans `exploring` mais afficher erreur dans logs

---

### 4. Logique de détection robot - Incohérente ⚠️

**Problème actuel** :
```typescript
// useRobotStore.ts ligne 542-544
const robotDataOK = scanDataReceived && batteryDataReceived;
const robotNodeOK = nodes['/kaiaai_telemetry_node'];
// ❌ batteryDataReceived n'est pas vraiment nécessaire
```

**Observation** :
- Le robot fonctionne même si `batteryDataReceived: false`
- La battery n'est pas critique pour la navigation
- `/scan` topic à 5 Hz est suffisant

**Solution recommandée** :
```typescript
// Critère plus robuste
const robotDataOK = scanDataReceived;  // Scan suffit
const robotNodeOK = nodes['/kaiaai_telemetry_node'];

// Optionnel : ajouter timeout de scan
const scanTimeout = Date.now() - lastScanTimestamp < 2000; // 2s max
```

---

## Modifications Requises du Code

### Fichiers à Modifier

```
web/src/
├── types/
│   └── state.ts                  [MODIFIER] Simplifier états
├── store/
│   └── useRobotStore.ts          [MODIFIER] Logique transitions
├── hooks/
│   └── useButtonStates.ts        [MODIFIER] Logique boutons
├── components/
│   └── Header.tsx                [MODIFIER] Messages et couleurs
├── services/
│   └── apiService.ts             [VÉRIFIER] Appels API
└── ros_api.py                     [CORRIGER] Bug Content-Length
```

---

### 1. `types/state.ts` - Simplifier les États

**Avant** (12 états) :
```typescript
export const SystemState = {
  INITIAL: 'initial',
  CONNECTING_WS: 'connecting_ws',
  WS_CONNECTED: 'ws_connected',
  CONTAINER_READY: 'container_ready',
  ROBOT_READY: 'robot_ready',
  EXPLORATION_AVAILABLE: 'exploration_available',  // ❌ À SUPPRIMER
  EXPLORING: 'exploring',
  NAVIGATING: 'navigating',
  WS_ERROR: 'ws_error',
  CONTAINER_ERROR: 'container_error',
  ROBOT_LOST: 'robot_lost',
  EXPLORATION_ERROR: 'exploration_error',          // ❌ À SUPPRIMER
} as const;
```

**Après** (10 états) ✅ :
```typescript
export const SystemState = {
  // Démarrage
  INITIAL: 'initial',
  CONNECTING_WS: 'connecting_ws',
  WS_CONNECTED: 'ws_connected',
  CONTAINER_READY: 'container_ready',
  ROBOT_READY: 'robot_ready',

  // Opérationnel
  EXPLORING: 'exploring',
  NAVIGATING: 'navigating',

  // Erreurs
  WS_ERROR: 'ws_error',
  CONTAINER_ERROR: 'container_error',
  ROBOT_LOST: 'robot_lost',
} as const;

export type SystemState = typeof SystemState[keyof typeof SystemState];
```

---

### 2. `store/useRobotStore.ts` - Corriger Transitions

#### A. Supprimer logique `exploration_available`

**Avant** (lignes 556-559) :
```typescript
// Check if map is loaded for exploration
if (mapData && systemState === 'robot_ready') {
  transitionToState('exploration_available');  // ❌ SUPPRIMER
}
```

**Après** ✅ :
```typescript
// Pas de transition auto - start explore directement depuis robot_ready
// La présence de mapData est vérifiée dans le hook useButtonStates
```

#### B. Simplifier détection robot

**Avant** (lignes 542-544) :
```typescript
const robotDataOK = scanDataReceived && batteryDataReceived;
```

**Après** ✅ :
```typescript
const robotDataOK = scanDataReceived;  // Scan suffit
// Optionnel : ajouter vérification fraîcheur des données
```

#### C. Corriger transition exploring

**Avant** (lignes 562-566) :
```typescript
if (nodes['/explore_node'] && systemState === 'exploration_available') {
  transitionToState('exploring');
}
```

**Après** ✅ :
```typescript
// Ne PAS auto-transitionner vers exploring
// Seulement lors de startExploration() explicite
```

#### D. Gérer arrêt propre de l'exploration

**Ajouter dans `stopExploration()`** :
```typescript
stopExploration: async () => {
  const { addLog, transitionToState, systemState } = get();

  if (systemState !== 'exploring') {
    addLog('⚠ Exploration not active');
    return;
  }

  addLog('Stopping exploration...');

  // 1. Arrêter le node explore_lite
  const response = await apiService.stopExploration();

  if (response.success) {
    // 2. Annuler navigation en cours
    rosService.cancelNavigation();

    // 3. Arrêt sécurisé robot
    rosService.publishVelocity(0, 0);

    // 4. Attendre que explore_node disparaisse
    setTimeout(() => {
      get().checkNodes();
      if (!get().nodes['/explore_node']) {
        addLog('✓ Exploration stopped');
        transitionToState('robot_ready');
      }
    }, 2000);
  } else {
    addLog(`✗ Failed to stop exploration: ${response.message}`);
  }
},
```

---

### 3. `hooks/useButtonStates.ts` - Simplifier Logique

**Avant** :
```typescript
canStartExplore:
  connected &&
  (systemState === 'exploration_available' || systemState === 'robot_ready'),
```

**Après** ✅ :
```typescript
canStartExplore:
  connected &&
  systemState === 'robot_ready' &&
  mapData !== null,  // Vérifier qu'on a une carte

canStopExplore:
  connected &&
  systemState === 'exploring',
```

---

### 4. `components/Header.tsx` - Mettre à Jour Messages

**Avant** (lignes 8-22) :
```typescript
const messages: Record<SystemState, string> = {
  initial: 'Système initialisé - Cliquez sur Connect',
  connecting_ws: 'Connexion au ROS Bridge WebSocket...',
  ws_connected: 'ROS Bridge OK - Vérification conteneur...',
  container_ready: 'Conteneur opérationnel - En attente du robot...',
  robot_ready: 'Robot connecté - Tous systèmes opérationnels',
  exploration_available: 'Prêt pour exploration autonome',  // ❌ SUPPRIMER
  exploring: 'Exploration autonome en cours...',
  navigating: 'Navigation vers objectif...',
  ws_error: 'Erreur ROS Bridge - Vérifiez le serveur',
  container_error: 'Erreur conteneur - Nodes manquants',
  robot_lost: 'Robot déconnecté - Vérifiez alimentation',
  exploration_error: 'Erreur exploration - Node arrêté',    // ❌ SUPPRIMER
};
```

**Après** ✅ :
```typescript
const messages: Record<SystemState, string> = {
  initial: 'Système prêt - Cliquez sur Connect',
  connecting_ws: 'Connexion au RosBridge WebSocket...',
  ws_connected: 'RosBridge OK - Vérification Nav2...',
  container_ready: 'Nav2 opérationnel - En attente du robot...',
  robot_ready: 'Système opérationnel - Prêt à naviguer',
  exploring: 'Exploration autonome en cours...',
  navigating: 'Navigation vers objectif...',
  ws_error: 'Erreur RosBridge - Connexion perdue',
  container_error: 'Erreur conteneur - Nodes Nav2 manquants',
  robot_lost: 'Robot déconnecté - Reconnexion...',
};
```

**Mettre à jour `getStatusDetails()`** :
```typescript
const getStatusDetails = (state: SystemState): string => {
  const nav2Count = nav2Nodes.filter(n => nodes[n]).length;
  const robotCount = robotNodes.filter(n => nodes[n]).length;
  const topicCount = criticalTopics.filter(t => topics[t]).length;

  switch(state) {
    case 'ws_connected':
      return `Détection Nav2... (${nav2Count}/4 actifs)`;

    case 'container_ready':
      return `Nav2: ${nav2Count}/4 • Attente robot (${robotCount}/2 nodes)`;

    case 'robot_ready':
      return `Robot OK • Nav2: ${nav2Count}/4 • Topics: ${topicCount}/3`;

    case 'exploring':
      const exploreActive = nodes['/explore_node'] ? '✓' : '✗';
      return `Explore ${exploreActive} • Nav2: ${nav2Count}/4 • Frontières actives`;

    case 'navigating':
      return `Nav2 actif • Distance restante: X.Xm`;

    case 'robot_lost':
      const scanOK = topics['/scan'] ? '✓' : '✗';
      const batteryOK = topics['/battery_state'] ? '✓' : '✗';
      return `Scan ${scanOK} • Battery ${batteryOK} • Reconnexion en cours...`;

    default:
      return '';
  }
};
```

---

### 5. `web/ros_api.py` - Corriger Bug Content-Length

**Ligne 33** :
```python
# ❌ AVANT
content_length = int(self.headers['Content-Length'])

# ✅ APRÈS
content_length = int(self.headers.get('Content-Length', 0))
```

**Ajouter gestion body vide** (après ligne 33) :
```python
if content_length > 0:
    post_data = self.rfile.read(content_length)
    try:
        request_body = json.loads(post_data.decode('utf-8'))
    except json.JSONDecodeError:
        self.send_error(400, "Invalid JSON")
        return
else:
    request_body = {}  # Empty body OK for some endpoints
```

---

### 6. `services/apiService.ts` - Ajouter Méthodes Manquantes

**Ajouter** :
```typescript
export const apiService = {
  // ... existing methods

  /**
   * Start autonomous exploration
   */
  async startExploration(): Promise<ApiResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/api/explore/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
      });
      return await response.json();
    } catch (error) {
      return { success: false, message: `Network error: ${error}` };
    }
  },

  /**
   * Stop autonomous exploration
   */
  async stopExploration(): Promise<ApiResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/api/explore/stop`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
      });
      return await response.json();
    } catch (error) {
      return { success: false, message: `Network error: ${error}` };
    }
  },
};
```

---

## Paramètres Officiels KaiAI

### explore_lite (m-explore-ros2)

**Source**: `https://github.com/kaiaai/m-explore-ros2/blob/main/explore/config/params.yaml`

```yaml
explore_lite:
  ros__parameters:
    robot_base_frame: base_link              # ✅ PAS base_footprint
    return_to_init: true
    costmap_topic: map                        # ✅ PAS /global_costmap/costmap
    costmap_updates_topic: map_updates        # ✅ PAS /global_costmap/costmap_updates
    visualize: true
    planner_frequency: 0.15                   # ✅ PAS 0.33
    progress_timeout: 30.0
    potential_scale: 3.0
    orientation_scale: 0.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    min_frontier_size: 0.75                   # ✅ PAS 0.5
```

**Commande complète** :
```bash
ros2 run explore_lite explore --ros-args \
  -p robot_base_frame:=base_link \
  -p costmap_topic:=map \
  -p costmap_updates_topic:=map_updates \
  -p visualize:=true \
  -p planner_frequency:=0.15 \
  -p progress_timeout:=30.0 \
  -p min_frontier_size:=0.75
```

---

### Nav2 Parameters (makerspet_mini)

**Source**: `/app/ros_ws/install/makerspet_mini/share/makerspet_mini/config/navigation.yaml`

**Comparaison avec notre fichier** :

| Paramètre | Officiel | Notre config | Status |
|-----------|----------|--------------|--------|
| `base_frame_id` (AMCL) | `base_footprint` | `base_footprint` | ✅ |
| `robot_base_frame` (bt_navigator) | `base_link` | `base_link` | ✅ |
| `laser_max_range` (AMCL) | `-1.0` | `-1.0` | ✅ |
| `update_min_a` (AMCL) | `0.05` | `0.05` | ✅ |
| `update_min_d` (AMCL) | `0.05` | `0.05` | ✅ |
| `transform_tolerance` (bt_navigator) | `0.1` | absent | ⚠️ |
| `wait_for_service_timeout` | `5000` | absent | ⚠️ |

**Paramètres manquants à ajouter** :
```yaml
bt_navigator:
  ros__parameters:
    transform_tolerance: 0.1              # ⚠️ AJOUTER
    wait_for_service_timeout: 5000        # ⚠️ AJOUTER
```

**Conclusion** : Notre config est globalement correcte. Les paramètres manquants sont optionnels.

---

## Diagramme de Transitions

### Flux Normal (Happy Path)

```
┌──────────────┐
│   initial    │
└──────┬───────┘
       │ connect()
       ▼
┌──────────────┐
│connecting_ws │ (1-2s)
└──────┬───────┘
       │ WS OK
       ▼
┌──────────────┐
│ ws_connected │ (2-5s)
└──────┬───────┘
       │ Nav2 ≥3 nodes
       ▼
┌──────────────┐
│container_    │ (10-20s)
│   ready      │
└──────┬───────┘
       │ scan + battery OK
       ▼
┌──────────────┐
│ robot_ready  │◄─────────────────┐
└──┬────────┬──┘                  │
   │        │                     │
   │        │ startExplore()      │
   │        ▼                     │
   │   ┌──────────┐               │
   │   │exploring │───────────────┤
   │   └──────────┘ stopExplore() │
   │                              │
   │ sendGoal()                   │
   ▼                              │
┌──────────┐                      │
│navigating│──────────────────────┘
└──────────┘    goal succeeded
```

### Flux d'Erreurs

```
         ┌─────────────┐
         │ ws_error    │◄──── WS error
         └──────┬──────┘
                │ disconnect() + connect()
                ▼
         ┌─────────────┐
    ┌───│connecting_ws │
    │   └─────────────┘
    │
    │   ┌─────────────┐
    └──►│container_   │◄──── Nav2 < 3 nodes
        │   error     │
        └─────────────┘

         ┌─────────────┐
    ┌───│ robot_lost  │◄──── Perte scan/battery
    │   └──────┬──────┘
    │          │ auto-recovery
    │          ▼
    │   ┌─────────────┐
    └──►│ robot_ready │
        └─────────────┘
```

### Matrice de Transitions Complète

| Depuis / Vers | initial | connecting_ws | ws_connected | container_ready | robot_ready | exploring | navigating | ws_error | container_error | robot_lost |
|---------------|---------|---------------|--------------|-----------------|-------------|-----------|------------|----------|-----------------|------------|
| **initial** | - | connect() | - | - | - | - | - | - | - | - |
| **connecting_ws** | - | - | WS OK | - | - | - | - | WS error | - | - |
| **ws_connected** | - | - | - | Nav2 ≥3 | - | - | - | WS error | Nav2 <3 | - |
| **container_ready** | - | - | - | - | robot OK | - | - | WS error | Nav2 <3 | - |
| **robot_ready** | disconnect | - | - | - | - | startExplore | sendGoal | WS error | Nav2 <3 | robot lost |
| **exploring** | - | - | - | - | stopExplore | - | - | WS error | Nav2 <3 | robot lost |
| **navigating** | - | - | - | - | goal done | - | - | WS error | Nav2 <3 | robot lost |
| **ws_error** | disconnect | reconnect | - | - | - | - | - | - | - | - |
| **container_error** | disconnect | reconnect | - | - | - | - | - | - | - | - |
| **robot_lost** | - | - | - | - | robot back | - | - | WS error | - | - |

---

## Check-list d'Implémentation

### Phase 1 : Corrections Critiques

- [ ] Corriger `ros_api.py` ligne 33 (Content-Length bug)
- [ ] Tester API avec `curl -X POST http://IP:8083/api/explore/start`
- [ ] Supprimer états `exploration_available` et `exploration_error` de `state.ts`
- [ ] Mettre à jour tous les types dépendants

### Phase 2 : Store et Hooks

- [ ] Modifier `useRobotStore.ts` : supprimer logique `exploration_available`
- [ ] Simplifier détection robot (scan suffit)
- [ ] Corriger transitions explore (pas d'auto-transition)
- [ ] Améliorer `stopExploration()` avec arrêt propre
- [ ] Mettre à jour `useButtonStates.ts` (supprimer ref exploration_available)

### Phase 3 : UI

- [ ] Mettre à jour `Header.tsx` : messages simplifiés
- [ ] Améliorer `getStatusDetails()` avec infos pertinentes
- [ ] Mettre à jour couleurs selon nouvelle machine à états
- [ ] Tester tous les états visuellement

### Phase 4 : Services

- [ ] Vérifier `apiService.ts` : méthodes start/stop exploration présentes
- [ ] Tester appels API depuis le front
- [ ] Ajouter gestion d'erreurs robuste

### Phase 5 : Tests de Validation

- [ ] Test démarrage complet (initial → robot_ready)
- [ ] Test exploration avec bons paramètres
- [ ] Test navigation manuelle
- [ ] Test récupération erreurs (WS, container, robot_lost)
- [ ] Test transitions explore → robot_ready → exploring
- [ ] Test boutons activés/désactivés selon états

### Phase 6 : Documentation Code

- [ ] Ajouter commentaires JSDoc sur chaque état
- [ ] Documenter transitions dans le code
- [ ] Créer fichier CHANGELOG.md avec modifications

---

## Notes Finales

### Points de Vigilance

1. **explore_lite est sensible aux paramètres** : Toujours utiliser les paramètres officiels KaiAI
2. **robot_lost doit être récupérable** : Ne pas forcer disconnect, attendre que le robot revienne
3. **Pas d'auto-transition vers exploring** : Uniquement sur action explicite utilisateur
4. **Emergency stop doit fonctionner dans TOUS les états connectés**

### Améliorations Futures

1. Ajouter état `waypoint_following` pour navigation multi-points
2. Implémenter timeout sur `container_ready` (si robot ne vient jamais)
3. Ajouter métriques de performance (temps de démarrage, success rate goals)
4. Implémenter sauvegarde auto de carte toutes les N minutes en exploration
5. Ajouter notification sonore sur erreurs critiques

---

**Document créé le**: 2025-12-04
**Testé avec**: MakersPet Mini, ROS2 Iron, makerspet-mini-web:latest
**Status**: Prêt pour implémentation
