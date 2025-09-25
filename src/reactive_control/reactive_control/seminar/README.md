# Wall follow pas à pas

## 1. Ouvrir le devcontainer de simulation dans vscode

1. Référez-vous au [tutoriel](https://github.com/vaul-ulaval/vaul-wiki/wiki/Utilisation-du-serveur-de-simulation-%28Serge%29) pour ouvrir la simulation.
2. Le code que vous aurez à modifier aujourd’hui se trouve dans [tutorial.py](./tutorial.py).
3. La présentation power point expliquant le lidar et le PID est [ici](https://docs.google.com/presentation/d/1AMCOOB5uCNjKcj3Ota_mzNHFe4FhCAA36ZkOAaJu1uM/edit?usp=sharing).

---

## 2. Ouvrir le layout Foxglove

1. Téléchargez le layout : [foxglove_layout_autodrive.json](https://github.com/vaul-ulaval/vaul-wiki/blob/main/foxglove_layout_autodrive.json)
2. Ouvrez-le dans Foxglove avec l’option **Import from file...**

<img width="697" height="201" alt="foxglove import" src="https://github.com/user-attachments/assets/aa884ff7-dc23-4226-b8e5-4e35034f4d25" />

---

## 3. Lancer l’algorithme Wall-Follow et visualiser les rayons du lidar

1. Dans le terminal du devcontainer, exécutez :

```bash
   ros2 run reactive_control wall_follow_node
```

2. Dans Foxglove, vous devriez maintenant voir le robot la visualization d'un des rayons du lidar en vert. <img width="183" height="168" alt="lidar ray" src="https://github.com/user-attachments/assets/f1d2751a-2ef8-4c62-b7b2-eb9447c9829d" />

3. Ajustez le **lidar_viz_angle** pour afficher un rayon lidar différent.

<img width="848" height="209" alt="angle parameter" src="https://github.com/user-attachments/assets/803f3712-20c2-4e7d-8374-dc73d054e708" />

---

## 4. Ajuster le contrôleur Wall-Follow (TODO #1)

Vous pouvez expérimenter avec les paramètres dans [tutorial.py](./tutorial.py) et observer leur impact sur le comportement du robot.

### Params

```python
THETA_DEG = 60
LOOKAHEAD = 0.6  # m
DESIRED_DISTANCE_FROM_WALL = 0.5  # m
KP = 1.2
KD = 0.0
```

> **Note :** Le nœud doit être redémarré pour que les changements prennent effet.

```bash
# Arrêter avec Ctrl-C, puis redémarrer :
ros2 run reactive_control wall_follow_node
```

---

## 5. Implémentation de la fonction `lidar_angle_to_distance` (TODO #2)

- Retourne la mesure du lidar à un angle donné theta_rad.
- Pour un angle de 0 rad, la fonction doit retourner la mesure directement devant le robot.
- Utilisez des print pour mieux comprendre les variables.
- Validez votre code en comparant le comportement de votre robot avec celui de la solution fournie.

## 6. Implémentation de la fonction `apply_pd` (TODO #3)

- Cette fonction calcule l’angle de braquage à appliquer en fonction de l’erreur :
  > distance désirée au mur – distance mesurée.
- Référez-vous à la section PID du [power point](https://docs.google.com/presentation/d/1AMCOOB5uCNjKcj3Ota_mzNHFe4FhCAA36ZkOAaJu1uM/edit?usp=sharing) pour implémenter cette fonction.

## 7. Implémentation de la fonction `compute_future_distance_to_wall` (TODO #4)

- Cette fonction est la logique principale du wall follow.
- Référez-vous à la [documentation](https://github.com/f1tenth/f1tenth_lab3_template?tab=readme-ov-file#iii-wall-following) pour l'implémenter.

## 8. Implémentation de la fonction `compute_throttle_command` (TODO #5)

- Félicitation, vous avez codé toute la logique du wall follow ! 🎉
- Vous pouvez maintenant implémenter la fonction `compute_throttle_command` pour optimiser la vitesse du robot. Voici quelques pistes de solutions :

1. Vitesse en fonction de l'angle de braquage.
2. Modélisation de la vitesse maximale possible selon la friction statique entre les pneus et la piste. [Page allo-prof friction statique](https://www.alloprof.qc.ca/fr/eleves/bv/physique/la-force-de-frottement-p1018)
