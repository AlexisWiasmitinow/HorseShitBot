# Formation — Perception & Navigation Autonome ROS 2 (Jazzy)

> **Objectif** : Apprendre au stagiaire un workflow complet de robotique autonome, de la collecte de données terrain jusqu'au déploiement de la stack de navigation (Nav2) et de la perception temps réel (YOLO sur Jetson).
> 
> **Repo de référence** : `HorseShitBot` (ROS 2, Raspberry Pi / Jetson Nano)
> 
> **Durée** : 1 journée — **matin 9h00-12h00** / **après-midi 13h00-17h00**

---

## 1. Architecture du repo existant (connaître la base)

### 1.1. Packages ROS 2

| Package | Rôle |
|---------|------|
| `horseshitbot` | Package principal Python (ament_python). Contient les nœuds, drivers, launch files et la web UI. |
| `horseshitbot_interfaces` | Package d'interfaces (CMake). Messages et services custom pour les actionneurs et le bus MKS. |

### 1.2. Nœuds existants (points d'entrée)

```
mks_bus_node          → bus Modbus RTU (MKS SERVO57D)
wheel_driver_node     → /cmd_vel, odometrie, backend switchable MKS/ODrive
lift_node / brush_node / bin_door_node  → actionneurs avec homing par stall
gamepad_teleop_node   → manette Bluetooth (evdev), publie /cmd_vel
web_dashboard_node    → FastAPI sur le port 8080 (teleop, parametres, rosbags, video)
status_screen_node    → ecran TFT SPI ILI9341 240×320 embarqué
bag_recorder_node     → enregistrement rosbag2 (format MCAP) programmatique
lidar_node            → interface LiDAR
```

### 1.3. Points forts du repo pour la formation

- **Rosbags programmables** : deux instances `perception_recorder` et `mapping_recorder`. On peut démarrer/arrêter via la manette (bouton `Options` / `D-Pad Left`) ou le dashboard web.
- **Topics déjà publiés** :
  - `/camera/color/image_raw`
  - `/camera/aligned_depth_to_color/image_raw`
  - `/scan`
  - `/odom`
  - `/tf`
- **Web dashboard** : visualisation, téléchargement de rosbags, config manette, streaming MJPEG.
- **Scripts de test** : dans `test_scripts/`, pour valider chaque capteur/actionneur hors ROS.

---

## 2. Matin : Collecte de données & Perception (3h00) — 9h-12h

---

### 09h00 – 09h30 : Introduction & exploration du repo (30 min)

**À faire ensemble :**
1. `colcon build --packages-select horseshitbot horseshitbot_interfaces`
2. `source install/setup.bash`
3. Lancer le robot en mode "mapping" (caméra + lidar) :
   ```bash
   ros2 launch horseshitbot robot_launch.py enable_camera:=true enable_lidar:=true
   ```
4. Ouvrir le dashboard à `http://<IP_ROBOT>:8080` et identifier les cartes (wheel status, camera, LiDAR, enregistrement).

**Exercice du stagiaire :**
- Tracer l'architecture ROS 2 sur un papier : quels nœuds publient/consomment quels topics ?
- Identifier dans `src/horseshitbot/launch/robot_launch.py` à quoi sert chaque toggle (`enable_camera`, `enable_mks`, `enable_lidar`).

---

### 09h30 – 10h30 : Téléopération & enregistrement de rosbags (1h00)

**Contexte :**
Le `bag_recorder_node` utilise `rosbag2_py.SequentialWriter` pour écrire des fichiers MCAP horodatés dans `~/rosbags/`.

**Manette (Data Frog)** :
- Left stick : conduite
- D-Pad Up/Down : lift
- RB / LB : brush / bin door (hold)
- A : E-STOP / B : reprise
- Options (Start) : `reference_all`
- **Select (Menu)** : toggle bag `perception`
- **D-Pad Left** : toggle bag `mapping`

**Exercice pratique :**
1. Se connecter avec la manette. Vérifier le topic `/gamepad/status` :
   ```bash
   ros2 topic echo /gamepad/status
   ```
2. Conduire le robot dans l'environnement de test.
3. Démarrer un rosbag **mapping** (LiDAR + `/odom`) via la manette.
4. Démarrer un rosbag **perception** (caméra couleur + profondeur) via le dashboard ou la manette.
5. Vérifier les fichiers dans `~/rosbags/`.
6. **Commandes utiles** :
   ```bash
   ros2 bag info ~/rosbags/horseshitbot_YYYY-MM-DD_HH-MM-SS
   ros2 bag play ~/rosbags/horseshitbot_YYYY-MM-DD_HH-MM-SS --clock
   ```

**Cours / Démo enseignant :**
- Expliquer la différence entre `perception_recorder` (caméra) et `mapping_recorder` (LiDAR + odométrie + TF).
- Montrer le contenu de `bag_recorder_node.py` (lignes ~35-88) : mapping `topic <-> type` et les groupes de topics.

---

### 10h30 – 10h45 : Pause (15 min)

---

### 10h45 – 11h30 : Extraction d'images du rosbag pour YOLO (45 min)

**Objectif** : Transformer le rosbag perception en dataset d'images annotables.

**Script prêt à l'emploi** : `scripts/extract_images_from_bag.py`

Utilisation :
```bash
python3 scripts/extract_images_from_bag.py ~/rosbags/horseshitbot_2026-... \
  -o dataset/images/train
```

**Exercice du stagiaire :**
- Lire le source du script pour comprendre `rosbag2_py.SequentialReader`, `deserialize_message`, et la conversion `sensor_msgs/Image` → OpenCV.
- L'exécuter sur le rosbag perception enregistré ce matin.
- Vérifier qu'on obtient bien un dossier `dataset/images/train/frame_000000.jpg`...

**Format YOLO attendu** (à préparer pour l'annotation) :
```
dataset/
├── images/
│   ├── train/...
│   └── val/...
└── labels/
    ├── train/...
    └── val/...
```

---

### 11h30 – 12h00 : Annotation & entraînement YOLO (30 min)

**Annotation (15 min)** :
- Utiliser **LabelImg** ou **Roboflow** pour annoter les objets de l'environnement (ex: `cone`, `obstacle`, `porte`).
- Exporter au format YOLOv8 (fichiers `.txt` avec `class x_center y_center width height` normalisés).

**Entraînement (15 min — démo / PC dédié)** :
- Installer Ultralytics :
  ```bash
  pip install ultralytics
  ```
- Lancer un entraînement court (5-10 epochs) en local pour valider la pipeline :
  ```python
  from ultralytics import YOLO
  model = YOLO("yolov8n.pt")
  model.train(data="dataset/dataset.yaml", epochs=10, imgsz=640)
  ```
- Discuter de la taille du modèle : `yolov8n` (nano) pour inférence rapide sur Jetson vs `yolov8s/m/l`.

**Point clé :** On ne fait pas l'entraînement complet ici. On valide que le stagiaire sait :
1. Extraire des images d'un rosbag.
2. Annoter au format YOLO.
3. Lancer un `train` Ultralytics.
4. Comprendre les métriques (`mAP50`, `precision`, `recall`).

---

### 12h00 – 13h00 : Déjeuner (1h00)

---

## 3. Après-midi : SLAM & Navigation Autonome (4h00) — 13h-17h

---

### 13h00 – 14h00 : SLAM Toolbox — cartographie depuis le rosbag mapping (1h00)

**Prérequis** (à installer si ce n’est pas fait) :
```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

**Guide technique dédié** : `docs/slam_guide.md`
- Pourquoi SLAM Toolbox (et pas Cartographer / gmapping)
- Configuration optimisée pour le **YDLidar T-MINI Plus**
- Paramétrage spécifique à l’odométrie **open-loop** du robot
- Troubleshooting

**Mode "offline mapping"** (à partir du rosbag enregistré le matin) :
1. Lancer SLAM Toolbox en mode mapping :
   ```bash
   ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
   ```
2. Jouer le rosbag mapping en mode `use_sim_time` :
   ```bash
   ros2 bag play ~/rosbags/horseshitbot_... --clock
   ```
3. Visualiser dans RViz2 :
   - Ajouter `/map`
   - Ajouter `/scan` (LaserScan)
   - Vérifier que le LiDAR s'aligne avec la trajectoire odométrique.

**Points d'attention pédagogiques :**
- Le `wheel_driver_node` publie de l'odométrie par **commande** (RPM désiré → intégration), pas par retour codeur. C'est une odo "open-loop". Parler de la dérive et du rôle du LiDAR pour la fermer.
- Le `base_link → laser` est déjà publié dans `robot_launch.py` (lignes 106-113). Vérifier que les offsets (`z=0.15`, `yaw=3.14159`) correspondent au montage réel.

**Exercice du stagiaire :**
- Sauvegarder la carte au format `.pgm` + `.yaml` via SLAM Toolbox :
  ```bash
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
    data: '/home/<user>/maps/salle_test'"
  ```
- Copier les fichiers `salle_test.pgm` et `salle_test.yaml` dans `src/horseshitbot/maps/` (voir `maps/README.md`).
- Ouvrir le `.pgm` avec un visualiseur d'image pour comprendre comment SLAM représente l'occupation.

---

### 14h00 – 14h15 : Pause (15 min)

---

### 14h15 – 15h15 : Nav2 — Navigation autonome (1h00)

**Prérequis** :
```bash
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-nav2-amcl
```

**Contenu pédagogique :**

1. **Architecture Nav2** (schéma au tableau) :
   - `Planner Server` (global planner : NavFn / A*)
   - `Controller Server` (local planner : Regulated Pure Pursuit)
   - `Behavior Tree` (XML)
   - `AMCL` (localisation sur carte connue)
   - `Costmap 2D` (global + local)

2. **Fichiers mis à disposition dans le repo** :
   - `src/horseshitbot/config/nav2_params.yaml` — config des serveurs Nav2 (**custom YOLO obstacles** dans le `local_costmap`)
   - `src/horseshitbot/launch/nav2_bringup_launch.py` — inclusion du `bringup_launch.py` officiel
   - `src/horseshitbot/maps/` — stockage des cartes SLAM

3. **Configuration minimale à adapter si besoin** :
   - `amcl` : `scan_topic: /scan`, `odom_frame_id: odom`
   - `controller_server` : `FollowPath` (Regulated Pure Pursuit)
   - `global_costmap` : `static_layer` (carte) + `obstacle_layer` (LiDAR)
   - `local_costmap` : `obstacle_layer` (LiDAR + **yolo_obstacles** PointCloud2)

4. **Lancer Nav2 avec la carte statique** :
   ```bash
   ros2 launch horseshitbot nav2_bringup_launch.py map:=$(pwd)/src/horseshitbot/maps/salle_test.yaml
   ```

**Exercice du stagiaire :**
- Lire `config/nav2_params.yaml` et identifier la section `local_costmap/obstacle_layer/yolo_obstacles`.
- Lancer Nav2 en localisation ET en navigation.
- Envoyer un **Goal Pose** via RViz2 et observer le robot planifier puis suivre le chemin.
- Comprendre la différence entre la carte **statique** (`/map`) et la **costmap** (`/global_costmap`, `/local_costmap`).

---

### 15h15 – 16h00 : Intégration perception + navigation (45 min)

**Concept** :
Nav2 utilise le `local_costmap` pour éviter les obstacles. La détection YOLO peut injecter des points 3D d'obstacles dans ce costmap via un topic `sensor_msgs/PointCloud2`.

**Nœud fourni** : `src/horseshitbot/nodes/yolo_detector_node.py`

Architecture :
```
Jetson / Pi
  ├─ yolo_detector_node
  │   ├─ abonne : /camera/color/image_raw
  │   │           /camera/aligned_depth_to_color/image_raw
  │   │           /camera/aligned_depth_to_color/camera_info
  │   ├─ publie : /yolo/detections    (String JSON)
  │   └─ publie : /yolo/obstacles     (PointCloud2) → obstacle_layer Nav2
  └─ Nav2 (planner + controller + AMCL)
```

Le nœud fait **l'alignement depth → couleur** : pour chaque bounding box YOLO, il lit la profondeur au centre du pixel, projette en 3D via la matrice intrinsèque `K`, et publie un nuage de points. Le `nav2_params.yaml` a déjà la source `yolo_obstacles` configurée dans le `local_costmap`.

**Exercice du stagiaire :**
- Explorer le code de `yolo_detector_node.py`.
- Identifier la fonction `_pixel_to_3d` et expliquer le rôle de `fx`, `fy`, `cx`, `cy`.
- Lancer le nœud à la main en simulation (si N/A, discuter de la démarche) :
  ```bash
  ros2 run horseshitbot yolo_detector_node --ros-args -p model_path:=~/models/best.pt
  ```
- Vérifier que `/yolo/obstacles` apparaît dans la liste des topics.

---

### 16h00 – 16h15 : Pause (15 min)

---

### 16h15 – 17h00 : Déploiement terrain sur Jetson (45 min)

**Objectif** : tout doit tourner sur le robot embarqué avec un seul launch.

**1. Modèle YOLO optimisé**

Script fourni : `scripts/convert_yolo_tensorrt.sh`
```bash
# Exporter best.pt → ONNX → TensorRT FP16 (Jetson)
./scripts/convert_yolo_tensorrt.sh ~/models/best.pt
# Résultat : ~/models/best.engine
```

**Pourquoi ?** Le `.pt` PyTorch est lent sur Jetson. Le `.engine` TensorRT exploite l'accélération GPU.

**2. Launch file intégré**

Fichier fourni : `launch/autonomy_launch.py`

Usage terrain "tout en un" :
```bash
ros2 launch horseshitbot autonomy_launch.py map:=~/maps/salle_test.yaml
```

Ce launch démarre simultanément :
- `robot_launch.py` — base, caméra, LiDAR, roues, manette, dashboard, écran
- `yolo_detector_node` — perception en temps réel
- `nav2_bringup_launch.py` — AMCL, planner, controller, costmaps

**3. Checklist déploiement en salle**

| # | Vérification | Commande / Méthode |
|---|--------------|-------------------|
| 1 | TF arbre complet | `ros2 run tf2_tools view_frames` |
| 2 | Topics actifs | `ros2 topic list` |
| 3 | Fréquence caméra | `ros2 topic hz /camera/color/image_raw` |
| 4 | Fréquence LiDAR | `ros2 topic hz /scan` |
| 5 | État AMCL | Vérifier `/amcl_pose` dans RViz2 |
| 6 | Goal autonome | Envoyer via RViz2, observer planification + suivi |

**Point pédagogique important :**
- Le `wheel_driver_node` utilise un **ramping pass-through**. Nav2 envoie `/cmd_vel` à 20 Hz. Le `watchdog_sec` (0.8 s) est compatible, mais si le stagiaire voit des saccades, parler du throttle côté `velocity_smoother` de Nav2 (`nav2_params.yaml`).
- L'odométrie est open-loop : AMCL est **indispensable** pour corriger la dérive sur la carte.

---

## 4. Fichiers créés / mis à jour dans le repo

| Fichier | Rôle |
|---------|------|
| `scripts/extract_images_from_bag.py` | Extraction d'images JPEG depuis rosbag perception |
| `src/horseshitbot/nodes/yolo_detector_node.py` | Inférence YOLO + projection depth → obstacles `PointCloud2` |
| `src/horseshitbot/config/nav2_params.yaml` | Configuration Nav2 avec `yolo_obstacles` dans le `local_costmap` |
| `src/horseshitbot/config/slam_toolbox_config.yaml` | Config SLAM Toolbox pré-tunée YDLidar T-MINI + odo open-loop |
| `docs/slam_guide.md` | Guide complet SLAM (choix, tuning, troubleshooting) |
| `src/horseshitbot/launch/nav2_bringup_launch.py` | Lance AMCL + Nav2 via le `bringup_launch.py` officiel |
| `src/horseshitbot/launch/autonomy_launch.py` | **Launch "tout-en-un"** terrain |
| `src/horseshitbot/maps/` | Répertoire pour stocker les cartes SLAM (`.pgm` + `.yaml`) |
| `scripts/convert_yolo_tensorrt.sh` | Export `.pt` → ONNX → TensorRT `.engine` sur Jetson |

---

## 5. Questions / Pièges classiques du stagiaire

1. **"Pourquoi mon `ros2 bag play` ne publie rien dans RViz ?"**  
   → Vérifier `use_sim_time:=true` côté SLAM/Nav2 et `--clock` côté `ros2 bag play`.

2. **"Pourquoi Nav2 ne reçoit pas mon odom ?"**  
   → Vérifier que `odom` est bien le `frame_id` du message `Odometry` et que la TF `odom → base_link` existe.

3. **"Pourquoi ma costmap est vide ?"**  
   → Le LiDAR doit publier sur `/scan` avec le bon `frame_id` (`laser`). Vérifier avec `ros2 topic echo /scan | head`.

4. **"Pourquoi le robot ne suit pas le chemin ?"**  
   → Le `controller_server` publie sur `/cmd_vel`. Vérifier que `wheel_driver_node` est lancé et que personne d'autre ne publie sur `/cmd_vel`.

5. **"Pourquoi YOLO est lent sur le Jetson ?"**  
   → Ne pas utiliser le modèle `.pt` PyTorch brut. Utiliser le `.engine` TensorRT généré par `convert_yolo_tensorrt.sh`.

6. **"Pourquoi les obstacles YOLO n'apparaissent pas dans la local_costmap ?"**  
   → Vérifier que le topic `/yolo/obstacles` publie bien un `PointCloud2` et que son `header.frame_id` est connu dans l'arbre TF (ex. `camera_color_optical_frame`).

---

## 6. Ressources complémentaires

- **SLAM Toolbox** : https://github.com/SteveMacenski/slam_toolbox
- **Nav2** : https://navigation.ros.org/
- **Ultralytics YOLOv8** : https://docs.ultralytics.com/
- **RealSense ROS2** : déjà installé via `ros-$ROS_DISTRO-realsense2-camera`
- **Jetson inference / TensorRT** : `trtexec`, `jetson-inference`

---

## 7. Fiche récapitulative "Matin / Après-midi"

| Phase (horaire) | Compétences acquises | Livrable |
|-------------------|----------------------|----------|
| 09h00 – 09h30 | Architecture ROS2, launch files, dashboard | Schéma des nœuds/topics |
| 09h30 – 10h30 | Manette, rosbags, dashboard / CLI | 2 rosbags (perception + mapping) |
| 10h45 – 11h30 | Python rosbag2, cv2, dataset | Dossier `dataset/images/` |
| 11h30 – 12h00 | LabelImg/Roboflow, Ultralytics | Dataset annoté + métriques validées |
| 13h00 – 14h00 | SLAM Toolbox, odom LiDAR/odo, TF | `map.pgm` + `map.yaml` |
| 14h15 – 15h15 | Nav2, AMCL, BT, costmap, RViz2 Goal | Robot qui navigue en autonomie |
| 15h15 – 16h00 | YOLO, depth projection, PointCloud2, obstacle_layer | Intégration perçue dans costmap |
| 16h15 – 17h00 | TensorRT, launch intégré, checklist terrain | Launch `autonomy_launch.py` opérationnel |

---

*Formation préparée pour le repo `HorseShitBot` — ROS 2 Jazzy / Humble.*
