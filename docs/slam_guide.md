# Guide SLAM pour HorseShitBot (YDLidar T-MINI Plus)

> **SLAM recommandé** : [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — ROS 2 natif, maintenu par Steve Macenski (mainteneur Nav2).
> 
> **Pourquoi pas Cartographer ?** Cartographer fonctionne, mais sa configuration est verbeuse (fichiers Lua, pbstream) et moins pédagogique pour une formation d’une journée.
> **Pourquoi pas gmapping / hector ?** Ce sont des packages ROS 1 non portés officiellement sur ROS 2.

---

## 1. Pourquoi SLAM Toolbox avec le T-MINI Plus ?

| Critère | SLAM Toolbox | Avantage pour le robot |
|---------|--------------|------------------------|
| **Compatibilité** | Natif ROS 2 (`sensor_msgs/LaserScan`) | Le `lidar_node` publie déjà sur `/scan` |
| **Mode offline** | `online_sync_launch.py` + `--clock` | Permet de générer une carte à partir du rosbag enregistré le matin |
| **Open-loop odometry** | Scan-to-map matching robuste | Corrige la dérive de l’odométrie par intégration des RPM (pas de codeurs absolus) |
| **Sortie** | `.pgm` + `.yaml` directement | Compatible Nav2 / `map_server` sans conversion |
| **Lifelong mapping** | `lifelong_launch.py` disponible | Pour cartographier de grandes surfaces sur la durée |

---

## 2. Caractéristiques du Lidar utilisées par SLAM

Depuis `lidar_node.py` :

| Paramètre | Valeur | Impact sur SLAM |
|-----------|--------|-----------------|
| `frame_id` | `laser` | Le TF `base_link → laser` doit exister (déjà dans `robot_launch.py`) |
| `topic` | `/scan` | SLAM Toolbox s’y abonne par défaut |
| `range_min` | 0.02 m | Très proche du robot → bien pour détecter les obstacles à l’arrière |
| `range_max` | 12.0 m | Portée correcte pour intérieur / hangar |
| `angle_offset` | -90° | Compensation de l’orientation physique du capteur (géré par le driver) |
| `scan_bins` | 720 | Résolution angulaire 0.5° — très bien pour un lidar low-cost |

> ⚠️ **Attention** : le `base_link → laser` dans `robot_launch.py` a un `yaw=3.14159` (π rad). Le lidar est monté à 180° par rapport à l’avant du robot. SLAM Toolbox s’en fiche : il utilise le TF arbre pour transformer le scan dans `odom`.

---

## 3. Installation

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

Vérifier la présence des launch files :
```bash
ros2 pkg prefix slam_toolbox
# Doit afficher /opt/ros/$ROS_DISTRO/share/slam_toolbox
```

---

## 4. Configuration recommandée (`slam_toolbox_config.yaml`)

Créer `src/horseshitbot/config/slam_toolbox_config.yaml` :

```yaml
slam_toolbox:
  ros__parameters:
    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------
    mode: "mapping"          # "mapping", "localization", "lifelong"
    solver_plugin: "solver_plugins::CeresSolver"
    ceres_linear_solver: "SPARSE_SCHUR"
    ceres_preconditioner: "SCHUR_JACOBI"
    ceres_trust_strategy: "LEVENBERG_MARQUARDT"
    ceres_dogleg_type: "TRADITIONAL_DOGLEG"
    ceres_loss_function: "None"

    # ------------------------------------------------------------------
    # Topics
    # ------------------------------------------------------------------
    odom_frame: "odom"
    map_frame: "map"
    base_frame: "base_link"
    scan_topic: "/scan"

    # ------------------------------------------------------------------
    # Scan matching
    # ------------------------------------------------------------------
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.15      # mètre (petit robot, odo open-loop)
    minimum_travel_heading: 0.15       # radian
    scan_buffer_size: 15
    scan_buffer_maximum_scan_distance: 10.0

    # ------------------------------------------------------------------
    # Loop closure
    # ------------------------------------------------------------------
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5    # tolérance pour matcher des scans éloignés
    loop_search_maximum_distance: 5.0  # distance max pour chercher une boucle
    loop_search_maximum_angle: 3.14    # 180°
    do_loop_closing: true
    loop_match_minimum_chain_size: 3
    loop_match_maximum_variance_coarse: 0.4
    loop_match_minimum_response_coarse: 0.1
    loop_match_minimum_response_fine: 0.1

    # ------------------------------------------------------------------
    # Correspondance (scan-to-map)
    # ------------------------------------------------------------------
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # ------------------------------------------------------------------
    # Carte
    # ------------------------------------------------------------------
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349   # ~0.2°
    coarse_search_angle_offset: 0.349    # ~20°
    coarse_angle_resolution: 0.0349       # ~2°
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true

    # ------------------------------------------------------------------
    # Rasterisation
    # ------------------------------------------------------------------
    resolution: 0.05        # 5 cm / pixel
    max_laser_range: 12.0   # ne pas dépasser le range_max du lidar
```

### Points clefs de cette config pour HorseShitBot

- `minimum_travel_distance: 0.15` : avec une odo open-loop qui dérive, on veut **beaucoup de recouvrement** entre scans. 15 cm garantit un bon overlap sans saturer le CPU.
- `link_scan_maximum_distance: 1.5` : le robot est petit et évolue dans des espaces potentiellement étroits ; on autorise un matching plus large que la config par défaut (0.3).
- `max_laser_range: 12.0` : identique au `range_max` du driver. Ne pas augmenter artificiellement sinon les rayons sans retour créent du bruit dans la carte.
- `resolution: 0.05` : bon compromis précision / poids mémoire pour un Jetson.

---

## 5. Mapping en temps réel (live)

**Pré-requis** : le robot est allumé, le lidar tourne (`/scan` publie), l’odométrie est active (`/odom`), et la TF `base_link → laser` existe.

```bash
# Lancer le robot (si pas déjà fait)
ros2 launch horseshitbot robot_launch.py enable_camera:=false

# Dans un autre terminal, lancer SLAM Toolbox
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=$(pwd)/src/horseshitbot/config/slam_toolbox_config.yaml \
  use_sim_time:=false
```

Visualiser dans RViz2 :
- Topic `/map` (OccupancyGrid)
- Topic `/scan` (LaserScan), couleur par intensité si besoin
- TF : vérifier que `odom → base_link → laser` est cohérent

Conduire le robot lentement (le T-MINI peut manquer des points en virage rapide).

---

## 6. Mapping offline depuis rosbag (recommandé pour la formation)

C’est le mode utilisé pendant la formation (13h00-14h00) car il permet de rejouer, corriger, et ne pas dépendre de la batterie du robot.

```bash
# 1. Lancer SLAM Toolbox en mode simulé
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=$(pwd)/src/horseshitbot/config/slam_toolbox_config.yaml \
  use_sim_time:=true

# 2. Jouer le rosbag (le bag du mapping, qui contient /scan + /odom + /tf)
ros2 bag play ~/rosbags/horseshitbot_YYYY-MM-DD_HH-MM-SS --clock

# 3. RViz2 (optionnel, pour visualiser la carte se construire)
rviz2
```

> ⏱️ Un rosbag de 5-10 minutes d’une salle de 10×10 m suffit largement.

---

## 7. Sauvegarder la carte

### Méthode A : Service ROS 2 (recommandé)

```bash
# Créer le dossier de destination
mkdir -p ~/maps

# Appeler le service de sauvegarde
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: '/home/$(whoami)/maps/salle_test'"
```

Résultat :
```
~/maps/
├── salle_test.pgm
└── salle_test.yaml
```

### Méthode B : CLI map_saver (legacy mais fonctionnel)

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f ~/maps/salle_test
```

---

## 8. Intégration Nav2 — passer de la carte à la navigation

1. Copier les fichiers `.pgm` + `.yaml` dans le repo :
   ```bash
   cp ~/maps/salle_test.* src/horseshitbot/maps/
   ```
2. Cette carte est ensuite utilisée par `nav2_bringup_launch.py` → `map_server` → AMCL.
   ```bash
   ros2 launch horseshitbot nav2_bringup_launch.py \
     map:=$(pwd)/src/horseshitbot/maps/salle_test.yaml
   ```

---

## 9. Troubleshooting SLAM Toolbox + T-MINI

| Symptôme | Cause probable | Solution |
|----------|----------------|----------|
| **“No laser scan received”** | TF `base_link → laser` manquante | Vérifier `robot_launch.py` (lignes 106-113) ou `ros2 run tf2_tools view_frames` |
| **Carte floue / brouillard** | `minimum_travel_distance` trop petit + robot trop lent / immobile | Augmenter à 0.25 ; assurer un mouvement continu |
| **La carte se dédouble** | Odométrie open-loop dérive trop entre deux scans | Conduire plus lentement en ligne droite ; vérifier que le lidar scan rate est stable |
| **Boucle de fermeture ratée** | Espace trop peu texturé (couloir blanc) | Ajouter des objets distinctifs ; baisser `link_scan_maximum_distance` à 0.8 |
| **Points fantômes derrière le robot** | `range_min` très bas (0.02 m) et réflexions | Augmenter `range_min` à 0.1 m dans `lidar_node` (filtre) ou ignorer les très courtes distances |
| **Carte trop petite / coupée** | Le rosbag s’est arrêté avant la fin | Continuer la lecture ; ou utiliser `--rate 0.5` pour ralentir et laisser SLAM suivre |
| **CPU saturé sur Jetson** | `resolution: 0.05` est lourd | Passer à `resolution: 0.10` si la précision n’est pas critique |

---

## 10. Mode localization (pour aller plus loin)

Une fois la carte générée, on peut passer SLAM Toolbox en mode **localisation** (sans modifier la carte) :

```bash
# Éditer slam_toolbox_config.yaml : mode: "localization"
# Ajouter :
# map_file_name: "/home/user/maps/salle_test"
# map_start_pose: [0.0, 0.0, 0.0]

ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=$(pwd)/src/horseshitbot/config/slam_toolbox_config.yaml \
  use_sim_time:=false
```

Dans ce mode, SLAM se comporte comme AMCL + une légère correction scan-to-map. **Nav2 fournit déjà AMCL**, donc ce mode n’est pas nécessaire pour la stack autonome du projet, mais c’est utile à connaître.

---

*Guide écrit pour le lidar **YDLidar T-MINI Plus** du HorseShitBot — baud 230400, frame `laser`, TF `base_link→laser` fournie par le launch robot.*
