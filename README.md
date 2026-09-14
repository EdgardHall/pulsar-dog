# pulsar-dog

Socle de connexion, enveloppe de sécurité et téléopération pour un **Unitree Go2 EDU**.

L'idée : avant d'écrire quoi que ce soit d'ambitieux (perception, autonomie, agent),
avoir une couche basse dans laquelle on a confiance — une liaison qui se diagnostique,
des commandes bornées, un watchdog qui arrête le chien quand le programme qui le pilote
meurt, et des logs de tout ce qui s'est passé.

Tout est testable **sans robot** grâce au backend `sim`.

## Installation

```bash
git clone https://github.com/EdgardHall/pulsar-dog
cd pulsar-dog
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
```

Le SDK Unitree n'est pas sur PyPI, il s'installe depuis les sources :

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python
pip install -e unitree_sdk2_python
```

## Premiers pas

```bash
# 1. Sans robot : tout le stack tourne sur le simulateur intégré
pulsar-dog --backend sim info
pulsar-dog --backend sim teleop

# 2. Avec le robot : poser l'IP et vérifier la liaison, sans rien bouger
pulsar-dog setup --interface enp3s0        # --dry-run pour voir les commandes d'abord
pulsar-dog doctor

# 3. Une lecture de télémétrie
pulsar-dog info

# 4. Téléop, chien sur un sol dégagé, 1 m autour de lui
pulsar-dog teleop --max-vx 0.3 --max-vyaw 0.5 --record

# 5. Marche pilotée par une policy, 20 s, enveloppe réduite
pulsar-dog walk --policy command --duration 20 --max-vx 0.25 --record

# 6. Relire ce qui s'est passé
pulsar-dog replay logs/walk-*.jsonl
```

Première séance avec le robot : suis [`docs/RUNBOOK.md`](docs/RUNBOOK.md), qui donne la
sortie attendue à chaque étape.

`doctor` vérifie l'interface réseau, l'adresse IP, le ping du robot et la présence du
SDK — c'est ce qui explique la quasi-totalité des « le SDK se bloque sans message ».

### Câblage

Le Go2 EDU expose son réseau interne `192.168.123.0/24` sur le port Ethernet. Côté PC :

```bash
sudo ip addr add 192.168.123.99/24 dev eth0
sudo ip link set eth0 up
ping 192.168.123.161
```

Détails, variantes Wi-Fi et pièges : [`docs/GO2_EDU_NOTES.md`](docs/GO2_EDU_NOTES.md).

## Téléopération

| Touche | Action |
|---|---|
| `z`/`w` `s` | avancer / reculer |
| `q`/`a` `d` | translation latérale gauche / droite |
| `j` `l` ou ← → | rotation |
| `espace` | stop |
| `+` `-` | échelle de vitesse (0.1 à 1.0) |
| `1` `2` `3` | debout / couché / balance stand |
| `m` | damp — les moteurs se relâchent, le chien s'affaisse |
| `e` / `r` | arrêt d'urgence / réarmement |
| `esc` | quitter (arrête le robot avant) |

AZERTY et QWERTY sont mappés tous les deux. Un terminal ne signale jamais le
relâchement d'une touche : chaque appui rafraîchit une échéance de 220 ms, donc
lâcher la touche suffit à arrêter le robot.

### Manette

```bash
pulsar-dog gamepad-probe                   # relever les indices de TON pad
pulsar-dog teleop --input gamepad --max-vx 0.2 --record
```

Deux choses que le clavier ne peut pas donner, et qui comptent dès que 15 kg marchent
pour de vrai :

- **des axes analogiques** — un stick demande 0.12 m/s, une touche ne demande jamais
  que le maximum ;
- **un homme-mort** — le mouvement n'est autorisé que tant qu'un bouton est *maintenu*.
  Tu lâches, tu poses la manette, tu t'éloignes : le robot s'arrête. C'est une propriété
  du périphérique, pas du logiciel qui remarque que quelque chose ne va pas.

La numérotation des axes et boutons change d'un pad, d'un driver et d'un OS à l'autre —
d'où `gamepad-probe`. Si la manette est débranchée en cours de route, la boucle s'arrête :
c'est elle qui autorise le mouvement, sans elle plus rien n'est autorisé.

## Locomotion pilotée par une policy

La couche `locomotion/` reprend la structure des environnements Isaac Lab / Isaac Sim —
managers de commandes, d'observations, de terminaisons, et une boucle de policy décimée —
pour qu'un contrôleur développé en simulation se déploie sans réécrire sa plomberie.

```bash
pulsar-dog walk --describe          # la disposition exacte du vecteur d'observation
pulsar-dog walk --policy command    # baseline : suit la commande, boucle ouverte
pulsar-dog walk --policy patrol     # aller-retour déterministe
pulsar-dog walk --policy policy.pt  # un export TorchScript issu de l'entraînement
```

```
observation dim=15
  [ 0: 3] base_lin_vel        [ 3: 6] base_ang_vel
  [ 6: 9] projected_gravity   [ 9:12] velocity_commands
  [12:15] last_action
```

Point important : **l'action est la vitesse du corps, pas un couple articulaire**. La
policy sort trois valeurs normalisées qui passent par `set_velocity()` — donc par
l'écrêtage, la rampe et le watchdog. Une policy qui déraille est bornée par les mêmes
garde-fous que le clavier.

Les terminaisons d'Isaac Lab deviennent ici la couche de sécurité : basculement au-delà
de 35°, corps effondré, batterie basse, télémétrie périmée ⇒ arrêt d'urgence verrouillé.
Une fin de durée est en revanche un arrêt propre.

Détails et checklist sim2real : [`docs/ISAAC_LAB_BRIDGE.md`](docs/ISAAC_LAB_BRIDGE.md).

## Relecture des runs

```
$ pulsar-dog replay logs/walk-20260914T183434Z.jsonl
600 samples, duration 12.1s, rate 49.7 Hz
  odometry    2.35 m walked (drifts - not a position)
  vx cmd      +0.00 .. +0.30  |████████████████████▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁████████████████████|
  vx meas     +0.00 .. +0.30  |▅████████████████████▄▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▅███████████████████|
  tracking    mean |cmd-meas| on vx = 0.011 m/s
```

Texte brut volontairement : la première chose qu'on fait après un run raté, c'est lire son
log en SSH avec rien d'installé. Un `tracking` gros et constant, c'est une échelle qui ne
correspond pas ; un `tracking` qui grandit, c'est de la latence. Une ligne `GAP` veut dire
que la boucle a calé.

## Sécurité

Les garde-fous sont dans le code, pas dans la doc :

- **Bornes** (`SafetyLimits`) : toute commande est écrêtée avant d'atteindre le robot.
  Par défaut 0.6 m/s en avant, 0.4 m/s latéral, 0.8 rad/s en rotation — bien en dessous
  de ce dont le Go2 est capable, ce qui est voulu pour une première session.
- **Limitation d'accélération** : pas d'échelon de vitesse, le corps reste posé.
- **Watchdog** : sans nouvelle commande pendant `command_timeout_s` (0.35 s), la boucle
  ramène la consigne à zéro. Un téléop qui plante arrête le chien.
- **Arrêt d'urgence** : `dog.emergency_stop()` coupe la consigne sans rampe et se
  verrouille jusqu'à `clear_emergency_stop()`. Avec `damp_on_estop`, les moteurs sont
  aussi relâchés — utile quand le robot bascule, dangereux quand il est en hauteur.
- **Échecs backend** : 10 erreurs consécutives dans la boucle ⇒ arrêt d'urgence verrouillé.
- **Sortie propre** : `close()` arrête le robot avant de fermer le lien, y compris
  depuis une exception ou un Ctrl-C.

Réglable par variables d'environnement (`PULSAR_MAX_VX`, `PULSAR_CMD_TIMEOUT`,
`PULSAR_DAMP_ON_ESTOP`, …) ou par options CLI.

## Utilisation comme bibliothèque

```python
import time

from pulsar_dog import Config, PulsarDog, Velocity

with PulsarDog(Config.from_env()) as dog:
    dog.stand_up()
    dog.balance_stand()
    for _ in range(50):              # ~2 s : il faut réémettre la consigne
        dog.set_velocity(Velocity(vx=0.3, vyaw=0.2))
        time.sleep(0.04)
    dog.stop()
    dog.stand_down()
# le lien est fermé et le robot arrêté, même si le bloc lève une exception
```

## Architecture

```
cli.py          doctor · setup · gamepad-probe · replay · info · stand · sit ·
                damp · recovery · teleop · record · walk
robot.py        PulsarDog — un thread de contrôle possède le backend
safety.py       écrêtage, limitation d'accélération, watchdog
config.py       Config / SafetyLimits / NetworkConfig, surchargeables par env
net.py          diagnostic de liaison
telemetry.py    enregistrement JSONL
analysis.py     relecture des .jsonl, tracés ASCII
teleop/         frontaux de pilotage : clavier · manette (homme-mort)
locomotion/     commands · observations · policy · terminations · runner
transport/      base.py (contrat) · sdk2.py (robot réel) · sim.py (sans robot)
```

Un seul thread parle au backend. Les appelants posent une consigne ou déposent une
commande dans une file ; la boucle republie la consigne rampée à 50 Hz. Le SDK ne voit
donc jamais d'appels concurrents, et le robot continue de recevoir des commandes même
quand l'appelant est occupé.

`--backend auto` utilise le robot réel si `unitree_sdk2py` est importable, et **refuse
de démarrer** sinon : une session de téléop qui pilote silencieusement un simulateur est
pire qu'une session qui ne démarre pas.

## Tests

```bash
pytest          # 150 tests, aucun robot requis
ruff check .
```

## Ce qui n'est pas là (volontairement)

- **Contrôle bas niveau des 12 moteurs** : demande de libérer le service sport d'Unitree,
  donc de retirer la couche qui empêche le robot de tomber. Faisable, mais avec un
  harnais et dans un projet séparé. `locomotion/` est structuré pour que ce jour-là,
  seuls l'espace d'action et les termes articulaires changent.
- **Caméra / LIDAR L1** : `VideoClient` et le nuage de points, prochaine étape logique.
- **ROS 2** : le Go2 EDU parle DDS, un pont est possible si le besoin arrive.
