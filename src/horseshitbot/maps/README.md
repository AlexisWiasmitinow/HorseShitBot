# Maps

Place ici les cartes générées par SLAM Toolbox :

- `nom_carte.pgm` — image de la carte (occupation)
- `nom_carte.yaml` — métadonnées (résolution, origine, etc.)

## Générer une carte

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: '/home/<user>/maps/salle_test'"
```

## Utiliser la carte

Copier les fichiers `.pgm` + `.yaml` dans ce dossier, puis :

```bash
# Navigation autonome avec la carte
ros2 launch horseshitbot autonomy_launch.py map:=$(ros2 pkg prefix horseshitbot)/share/horseshitbot/maps/salle_test.yaml
```
