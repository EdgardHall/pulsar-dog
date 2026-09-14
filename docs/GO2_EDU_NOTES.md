# Go2 EDU — notes de terrain

Ce que ce projet suppose de ton robot, et pourquoi.

**Vérifié contre les sources.** Tout ce qui suit a été confronté au code de
[`unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python) et de
[`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2) — pas à un souvenir de
documentation. Les endroits où une incertitude subsiste sont marqués ⚠️.

La référence officielle reste le
[centre développeur Unitree](https://support.unitree.com/home/en/developer)
(Quick start, Sports services, Basic services).

## Réseau

Le Go2 EDU expose son réseau interne sur le port Ethernet du corps.

| Élément | Adresse | Remarque |
|---|---|---|
| Calculateur embarqué (Ethernet) | `192.168.123.161` | confirmé ; cible de `PULSAR_ROBOT_IP` |
| Ton PC sur ce lien | `192.168.123.99/24` | confirmé ; n'importe quelle autre adresse du `/24` marche |
| Wi-Fi en mode point d'accès | `192.168.12.1` | ⚠️ non vérifié ici ; sert à l'app et à WebRTC, pas au SDK DDS |

Le sous-réseau `192.168.123.0/24` est en dur dans les SDK eux-mêmes (les clients audio
testent `ip.find("192.168.123.") == 0`), et `.161` / `.99` sont les adresses de la
procédure d'installation Unitree.

Autres cartes du bus interne (carte de contrôle moteur, etc.) vivent aussi dans le
`192.168.123.0/24`. `arp-scan -l -I eth0` ou `ip neigh` après un ping du broadcast donne
la liste réelle de ton unité — plus fiable que n'importe quelle table de documentation.

Configuration côté PC :

```bash
ip -br link                                   # trouver le nom de l'interface
sudo ip addr add 192.168.123.99/24 dev eth0
sudo ip link set eth0 up
ping -c3 192.168.123.161
```

Le nom `eth0` est rarement le bon sur une machine moderne (`enp3s0`, `enx…` sur un
adaptateur USB). Passe-le avec `--interface` ou `PULSAR_INTERFACE`.

**Le Wi-Fi n'est pas une option pour le SDK.** Le SDK DDS a besoin du multicast sur
l'interface ; en pratique il faut le câble. Le Wi-Fi sert à l'app mobile et à la voie
WebRTC (`go2_webrtc_connect`), qui est un chemin complètement différent — c'est celui
des Go2 Air/Pro, qui n'ont pas d'accès DDS.

## DDS

Le robot publie et consomme sur CycloneDDS, **domaine 0**, sur l'interface que tu passes
à `ChannelFactoryInitialize(0, "eth0")`.

Topics utilisés ici :

| Topic | Type | Contenu |
|---|---|---|
| `rt/sportmodestate` | `SportModeState_` | mode, gait, odométrie, vitesse, IMU, forces de pied |
| `rt/lowstate` | `LowState_` | états moteurs, IMU brut, BMS (dont `soc`, la batterie) |

**Piège à connaître : le nom du topic d'état dépend du robot.** Le Go2 publie sur
`rt/sportmodestate`, mais les A2, AS2 et **Go2-W** utilisent `rt/lf/sportmodestate`
(« lf » = low frequency). Copier un exemple pris au mauvais endroit donne un abonnement
qui ne reçoit jamais rien — une liaison qui a l'air saine et une télémétrie vide.

- Go2 : `unitree_sdk2/example/go2/go2_sport_client.cpp` → `rt/sportmodestate`
- Go2-W, A2, AS2 : leurs exemples respectifs → `rt/lf/sportmodestate`

`rt/lowstate` est confirmé par `example/go2/low_level/go2_stand_example.py`.

Champs vérifiés dans les IDL : `SportModeState_` porte `mode`, `gait_type`, `position[3]`,
`body_height`, `velocity[3]`, `yaw_speed`, `foot_force[4]`, `error_code` et `imu_state`
(lui-même avec `rpy[3]`, `quaternion`, `gyroscope`, `accelerometer`). La batterie est dans
`LowState_.bms_state.soc`.

Pour plus tard : `rt/lowcmd` (commande moteur bas niveau), `rt/utlidar/switch` (allumer le
lidar) et les topics `rt/utlidar/*` (nuage de points du L1).

Si `pulsar-dog doctor` est vert mais que rien n'arrive, c'est presque toujours le
multicast : vérifie que l'interface est bien celle du câble, et qu'aucun VPN ou bridge
Docker ne capte la route.

## Sport mode (haut niveau)

`SportClient` pilote le contrôleur de locomotion d'Unitree — c'est ce que `pulsar-dog`
utilise. Les appels retournent `0` en succès et un code d'erreur sinon ; `Sdk2Backend`
lève une exception sur tout code non nul.

Séquence d'initialisation, telle que dans l'exemple officiel :

```python
sport = SportClient()      # enableLease=False par défaut
sport.SetTimeout(10.0)     # héritée de ClientBase
sport.Init()               # enregistre les API IDs
```

Méthodes utilisées ici : `StandUp`, `StandDown`, `BalanceStand`, `RecoveryStand`, `Damp`,
`Move(vx, vy, vyaw)`, `StopMove`.

Beaucoup d'autres existent : `Sit` / `RiseSit`, `Euler`, `SpeedLevel`, `Hello`, `Stretch`,
`Pose`, `Scrape`, `Heart`, les sauts et saltos (`FrontFlip`, `BackFlip`, `LeftFlip`,
`FrontJump`, `FrontPounce`, `HandStand`), les démarches (`FreeWalk`, `FreeBound`,
`StaticWalk`, `TrotRun`, `ClassicWalk`, `CrossStep`, `WalkUpright`), l'évitement
(`FreeAvoid`, `SwitchAvoidMode`) et `AutoRecoverySet` / `AutoRecoveryGet`.
`Sdk2Backend._call()` résout la méthode par son nom, donc ajouter l'une d'elles est une
ligne — et il dit clairement quand ton build ne l'a pas.

**`RecoveryStand` est la commande d'un robot tombé**, là où `StandUp` suppose qu'il est
déjà sur ses pattes : `pulsar-dog recovery`.

⚠️ `SwitchJoystick(on: bool)` existe et touche à la manette. Je ne l'utilise pas : la
manette est l'arrêt d'urgence physique, on ne la désactive pas depuis le logiciel.

`Move()` doit être **réémis en continu**. C'est précisément ce que fait la boucle de
contrôle à 50 Hz, et pourquoi le watchdog est le garde-fou central de ce projet.

Détail de convention vérifié : `Move()` passe par `_CallNoReply`, qui renvoie `0` si
l'envoi a réussi et un code d'erreur sinon. Les autres commandes passent par `_Call` et
renvoient le code de réponse du robot. `Sdk2Backend` lève sur tout code non nul dans les
deux cas ; une erreur d'envoi répétée finit par verrouiller l'arrêt d'urgence.

## Bas niveau

Le contrôle direct des 12 moteurs passe par `rt/lowcmd`, et demande de libérer le
service sport (`MotionSwitcherClient.ReleaseMode()`) — sans quoi deux contrôleurs se
battent pour les mêmes moteurs. Hors périmètre ici, volontairement : c'est le chemin
par lequel on casse un robot, et il mérite son propre projet, son propre banc et un
harnais de suspension.

## Checklist avant de faire marcher le chien

1. Zone dégagée, au moins 1 m autour du robot, sol non glissant.
2. **La manette reste dans tes mains.** C'est l'arrêt d'urgence physique ; elle a la
   priorité sur le SDK, et la commande de damp de la manette couche le chien quoi qu'il
   arrive côté logiciel. Une touche `e` dans un terminal n'est pas un arrêt d'urgence
   au sens sécurité, c'est un arrêt logiciel.
3. Batterie > 30 %. Un Go2 à plat se couche sans prévenir.
4. Premier essai à vitesse réduite : `--max-vx 0.2 --max-vyaw 0.3`.
5. Premier essai **en marche sur place** : rotation seule, puis translation.
6. `--record` dès le premier run. La télémétrie du run où ça s'est mal passé est
   exactement celle qu'on regrette de ne pas avoir.

## Pièges connus

- **Le chien ignore les commandes** : un autre contrôleur a la main (l'app mobile, la
  manette, un service sport encore actif). Ferme l'app.
- **Abonnement silencieux sur un Go2-W** : mauvais topic, voir l'encadré DDS plus haut.
- **`ChannelFactoryInitialize` se bloque** : mauvaise interface, ou pas d'adresse dans
  le `/24` du robot. `pulsar-dog doctor` le dit.
- **La télémétrie arrive mais `mode` reste à 0** : le robot est en veille, il faut un
  `StandUp` (ou un lever depuis la manette) avant que les commandes de marche fassent
  quoi que ce soit.
- **L'odométrie dérive.** `position` dans `SportModeState_` est de l'intégration
  propriceptive, pas une localisation. Sur quelques dizaines de secondes elle est
  utilisable, sur une pièce entière non.
- **Ne jamais faire `Damp()` sur un robot debout en hauteur** : les moteurs se relâchent
  et il tombe. C'est pour ça que `damp_on_estop` est à `False` par défaut.

## Références

- SDK Python : https://github.com/unitreerobotics/unitree_sdk2_python
- SDK C++ : https://github.com/unitreerobotics/unitree_sdk2
- Documentation développeur Unitree : https://support.unitree.com/home/en/developer
- Voie WebRTC (Go2 Air/Pro) : https://github.com/legion1581/go2_webrtc_connect
