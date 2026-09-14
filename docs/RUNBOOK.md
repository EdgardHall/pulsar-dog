# Runbook — première séance avec le chien

Répétition faite sur le simulateur, sortie réelle copiée ici. Ce soir, compare :
**tout écart est un signal**, au lieu d'avoir à deviner si c'est normal.

Les sorties ci-dessous viennent du backend `sim`. Sur le vrai robot, les différences
attendues sont marquées ⚠️.

---

## Avant de brancher

- [ ] Chien **par terre**, sol non glissant, 1 m dégagé autour
- [ ] Batterie > 50 %
- [ ] Manette chargée, allumée, **dans tes mains**
- [ ] App mobile **fermée** (sinon elle se dispute le contrôleur avec le SDK)
- [ ] Robot démarré depuis ~40 s

---

## 1. Le réseau

```
$ pulsar-dog --interface enp3s0 setup --dry-run
interface enp3s0, host 192.168.123.99/24
[ok] would run: sudo ip addr add 192.168.123.99/24 dev enp3s0
[ok] would run: sudo ip link set enp3s0 up
```

Vérifie que l'interface est la bonne (`ip -br link`), puis relance **sans** `--dry-run` :
il applique et enchaîne le diagnostic.

```bash
export PULSAR_INTERFACE=enp3s0     # pour ne plus le retaper
pulsar-dog setup
```

## 2. Le diagnostic

Voici à quoi ressemble un échec — c'est ce que tu verras si le câble n'est pas branché :

```
$ pulsar-dog --interface enp3s0 doctor
[FAIL] interface exists: 'enp3s0' among eth0, ifb0, ifb1, lo
[FAIL] host address on robot subnet: enp3s0 has no IPv4 address; set one on
       192.168.123.161's subnet, e.g. sudo ip addr add 192.168.123.99/24 dev enp3s0
[FAIL] robot responds to ping: 192.168.123.161 did not reply
[FAIL] unitree_sdk2py importable: missing - pip install -e path/to/unitree_sdk2_python

4 check(s) failed - see docs/GO2_EDU_NOTES.md
```

**Ce que tu veux voir ce soir : quatre `[ok]` et `link looks healthy`.** Tant que ce n'est
pas le cas, ne va pas plus loin — c'est exactement ce que cette commande existe pour éviter.

Le cas le plus probable : `interface exists` échoue parce que le nom n'est pas le bon.
La liste des interfaces est dans le message.

## 3. Première connexion, en lecture seule

```
$ pulsar-dog info
INFO pulsar_dog.robot: connected via sim backend
backend: sim
mode=0  gait=0  battery=100%  height=0.080m  pos=(0.00, 0.00, 0.08)  vel=(0.00, 0.00, 0.00)  rpy=(0.00, 0.00, 0.00)
INFO pulsar_dog.robot: disconnected
```

⚠️ Sur le robot : `backend: sdk2`, une vraie valeur de batterie, et une `height` qui
dépend de sa posture réelle.

**Le robot ne bouge pas.** Si tu vois la ligne `mode=…`, la liaison DDS fonctionne — c'est
le vrai franchissement de la soirée.

Si la commande se fige sans rien afficher : le multicast ne passe pas. Mauvaise interface,
ou un VPN / bridge Docker qui capte la route.

## 4. Première commande

Manette en main.

```
$ pulsar-dog stand
INFO pulsar_dog.robot: connected via sim backend
stand_up done - mode=1  gait=0  battery=100%  height=0.320m  pos=(0.00, 0.00, 0.32)  ...
INFO pulsar_dog.robot: disconnected
```

Le chien se lève. `height` passe de ~0.08 à ~0.32, `mode` de 0 à 1.

⚠️ Les valeurs de `mode` du vrai robot dépendent du firmware ; ce qui compte est
qu'elles **changent**.

Pour le recoucher : `pulsar-dog sit`.

**S'il est tombé** : `pulsar-dog recovery` (`RecoveryStand`), pas `stand` — `stand`
suppose qu'il est déjà sur ses pattes. `recovery` lève aussi l'arrêt d'urgence, puisque
c'est une chute qui l'a verrouillé.

## 5. Téléop

```bash
pulsar-dog teleop --max-vx 0.2 --max-vyaw 0.3 --record
```

Ordre des essais, dans cet ordre exactement :

1. **Rotation seule** (`j` / `l`) — il tourne sur place, tu vois qu'il répond sans qu'il
   parte nulle part
2. **Marche arrière** (`s`) — tu as le mur derrière toi dans le champ de vision
3. **Marche avant** (`z`)

Avec la manette (une fois les indices relevés avec `pulsar-dog gamepad-probe`) :

```bash
pulsar-dog teleop --input gamepad --max-vx 0.2 --record
```

Le bouton homme-mort doit être **maintenu** pour que les sticks agissent. La ligne d'état
affiche `ARMED` quand il est tenu, ` - ` sinon. Relâche : le chien s'arrête. C'est le
comportement à tester **en premier**, avant de toucher aux sticks.

## 6. Relire ce qui s'est passé

```
$ pulsar-dog replay logs/walk-20260914T183434Z.jsonl
walk-20260914T183434Z.jsonl: 600 samples
  duration    12.1s
  rate        49.7 Hz
  battery     100% -> 100%
  max tilt    0.0 deg
  odometry    2.35 m walked (drifts - not a position)
  vx cmd      +0.00 .. +0.30  |████████████████████▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁████████████████████|
  vx meas     +0.00 .. +0.30  |▅████████████████████▄▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▅███████████████████|
  vyaw cmd    -0.40 .. +0.00  |████████████████████▇▆▅▄▃▂▁▁▁▁▁▁▁▁▂▃▄▅▆▇████████████████████|
  vyaw meas   -0.40 .. +0.00  |█████████████████████▇▅▄▃▂▂▁▁▁▁▁▁▁▂▂▃▄▅▇████████████████████|
  tracking    mean |cmd-meas| on vx = 0.011 m/s
```

**À quoi ressemble un log sain** (c'en est un, celui d'une policy `patrol`) :

- `rate` proche de la fréquence demandée — ici 49.7 pour 50 Hz
- `vx meas` suit `vx cmd` avec un décalage d'un ou deux caractères : c'est la rampe
  d'accélération, elle est voulue
- `tracking` petit et **constant**
- aucune ligne `GAP`

**Ce qui doit t'alerter :**

| Signe | Ce que ça veut dire |
|---|---|
| `tracking` gros et constant | une échelle ne correspond pas |
| `tracking` qui grandit | de la latence |
| `vx meas` plat alors que `vx cmd` bouge | le robot n'exécute pas — un autre contrôleur a la main |
| une ligne `GAP` | la boucle a calé, ou la liaison a été perdue |
| `max tilt` élevé | il a penché beaucoup plus que tu ne l'as vu |

---

## En cas de problème

**La manette est ton arrêt d'urgence.** Elle a la priorité sur le SDK. La touche `e` du
clavier et le bouton d'e-stop du pad sont des arrêts **logiciels** : ils passent par le même
chemin que tout le reste, donc si ce chemin est cassé, ils le sont aussi.

| Symptôme | Cause la plus probable |
|---|---|
| le chien ignore tout | l'app mobile ou la manette a la main |
| `doctor` vert, aucune télémétrie | multicast : interface, VPN, bridge Docker |
| `ChannelFactoryInitialize` se fige | pas d'adresse dans le `/24` du robot |
| il se couche sans prévenir | batterie |
| `stand` renvoie un code d'erreur non nul | le robot refuse la posture — vérifie qu'il est à plat au sol |
| il est tombé et ne se relève pas | `pulsar-dog recovery`, pas `stand` |

---

## À rapporter

Si quelque chose cloche, garde ces trois choses : le `.jsonl` du run, la sortie de
`pulsar-dog doctor`, et la sortie complète de la commande qui a échoué (avec `-v`).
