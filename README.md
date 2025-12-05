# MakersPet Mini – Web ROS2 Stack

Conteneur ROS2 (Iron) autonome pour le robot MakersPet Mini (120 mm) embarquant une IHM web moderne. L’image assemble la pile Nav2 + slam_toolbox, l’agent micro-ROS, rosbridge, l’API HTTP de contrôle et le front React/Vite déjà buildé.

## Fonctionnalités actuelles
- Connexion WebSocket à rosbridge (`ws://<host>:9092`) depuis l’IHM.
- Visualisation de la carte en direct (occupancy grid), pose robot, overlay Leaflet, clic carte pour envoyer un goal de navigation ou définir la pose initiale.
- Joystick virtuel pour publier du `cmd_vel`.
- Gestion exploration auto : start/stop de `explore_lite`, suivi d’état via l’API (`/api/explore/*`).
- Commandes Nav2 : envoi/cancel de goal, suivi de statut (`/api/nav/*`).
- Gestion cartes slam_toolbox : save/load/clear via API (`/api/map/*`), cartes persistées dans `./maps`.
- Tableau de bord statut : surveillance topics critiques (`/scan`, `/map`, `/cmd_vel`, etc.) et nodes Nav2/rosbridge/telemetry/explore.
- Journal des commandes côté IHM (50 derniers événements).
- API HTTP légère (Python) exposée sur `:8083` pour piloter explore/nav/cartes.

## Prérequis
- Docker (compose v2 recommandé).
- Accès réseau au robot (esp32 + LiDAR) et aux ports utilisés (8888 micro-ROS agent, 9092 rosbridge, 8082 IHM, 8083 API).

## Lancer avec Docker Compose
```bash
cd /home/raynox/PROJECTS/autonomous-lidar-robot
docker compose build           # construit l’image makerspet-mini-web:latest
docker compose up -d           # démarre le conteneur en host network
```

Volumes montés automatiquement :
- `./maps` → `/app/maps` (cartes slam_toolbox persistées)
- `./logs` → `/app/logs` (logs rosbridge, nav, explore, web, API)
- `./config` → `/app/config` (params navigation/telemetry)

Ports exposés (mode host) :
- 8082 : interface web statique (`/app/web/dist` servie par `serve.py`)
- 8083 : API HTTP de contrôle (`web/ros_api.py`)
- 9092 : rosbridge WebSocket
- 8888 : micro-ROS agent (UDP4)

Arrêt :
```bash
docker compose down
```

## Cycle de build manuel (optionnel)
Pour reconstruire uniquement le front :
```bash
cd web
npm ci
npm run build
```
Le Dockerfile multi-stage reconstruit automatiquement le front via la première étape `webbuilder`.

## Utilisation rapide de l’IHM
1. Ouvrir `http://<host>:8082/index.html`.
2. Vérifier/ajuster l’URL rosbridge (par défaut `ws://192.168.0.10:9092`), puis **Connect**.
3. Contrôles disponibles :
   - **Joystick virtuel** pour `cmd_vel`.
   - **Carte interactive** : clic pour envoyer un goal ou définir la pose initiale.
   - **Exploration** : Start/Stop `explore_lite`.
   - **Cartes** : Save/Load/Clear slam_toolbox.
   - **Emergency Stop** pour couper immédiatement les commandes.
4. Suivre les statuts topics/nodes et le log des actions dans le panneau de droite.

## Arborescence utile
- `docker-compose.yml` : service `makerspet-mini-web` (host network, volumes).
- `Dockerfile` : build multi-stage (Node 20 → dist front, base `osrf/ros:iron-desktop-full`, workspace Nav2 + micro-ROS + rosbridge + telemetry + makerspet_mini).
- `scripts/entrypoint.sh` : orchestration au démarrage (serveur web, micro-ROS agent, telemetry, rosbridge, API, Nav2/slam_toolbox, activation lifecycle).
- `web/` : front React/TypeScript (Vite + Tailwind), services ROS (roslibjs) et API HTTP.
- `config/` : `navigation.yaml`, `telem.yaml`, etc.
- `launch/` : `robot_web.launch.py` pour Nav2 + slam_toolbox.

## Notes
- Le conteneur attend automatiquement le topic `/scan`, la TF `odom -> base_footprint`, puis active Nav2 ; il reste vivant même si le robot arrive plus tard.
- Logs consultables dans `./logs/*.log` côté hôte.
