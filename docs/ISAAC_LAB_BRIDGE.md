# De la simulation au Go2 — la couche `locomotion/`

Le module `pulsar_dog.locomotion` reprend la structure *manager-based* des
environnements Isaac Lab / Isaac Sim, parce que c'est celle qui rend une politique
transférable : ce qui compte au moment de passer sur le robot réel, ce n'est pas le
réseau, c'est que **le vecteur d'observation soit identique, dans le même ordre, avec
les mêmes échelles, au même pas de temps**. Un manager nommé rend ça vérifiable ;
un `np.concatenate` écrit à la main, non.

## Correspondance

| Isaac Lab | Ici |
|---|---|
| `CommandManager` / `UniformVelocityCommandCfg` | `locomotion/commands.py` |
| `ObservationManager`, `ObsTerm` | `locomotion/observations.py` |
| `ActionManager`, `action_scale` | `runner.ActionCfg` |
| `TerminationManager`, `DoneTerm` | `locomotion/terminations.py` |
| boucle `env.step()`, `decimation` | `runner.LocomotionRunner` |
| `env.reset()` après terminaison | arrêt du robot + arrêt d'urgence si faute |

## Le vecteur d'observation

```
$ pulsar-dog walk --describe
observation dim=15
  [ 0: 3] base_lin_vel        scale 2.0
  [ 3: 6] base_ang_vel        scale 0.25
  [ 6: 9] projected_gravity   scale 1.0
  [ 9:12] velocity_commands   scale (2.0, 2.0, 0.25)
  [12:15] last_action         scale 1.0
```

C'est l'ensemble de termes classique en locomotion quadrupède, **moins** les positions
et vitesses articulaires (12 + 12). Ce n'est pas un oubli : elles ne servent qu'à une
politique qui commande les articulations, donc au contrôle bas niveau, hors périmètre
ici (voir plus bas).

`projected_gravity` est la gravité exprimée dans le repère du corps. À plat elle vaut
`(0, 0, -1)` ; le lacet n'intervient pas, ce qui en fait une observation d'orientation
utilisable sans référence de cap. C'est aussi ce qui alimente la terminaison
`bad_orientation`.

`term_slices()` donne la position de chaque bloc : c'est l'outil pour vérifier qu'une
policy exportée lit bien ce qu'elle croit lire.

## L'espace d'action

**Ici l'action est la vitesse du corps, pas un couple articulaire.** La policy sort 3
valeurs normalisées, multipliées par `ActionCfg.scale` pour donner `(vx, vy, vyaw)` :

```
action ∈ [-1, 1]³  ──scale──▶  Velocity  ──▶  PulsarDog.set_velocity()
                                                 │
                                    écrêtage, rampe, watchdog
                                                 ▼
                                          SportClient.Move()
```

Conséquence directe : **toutes les protections existantes s'appliquent encore**. Une
policy qui part en vrille est écrêtée par `SafetyLimits`, rampée en accélération, et
coupée par le watchdog si la boucle décroche. Une policy qui commanderait les 12 moteurs
n'aurait aucun de ces filets.

Garder les actions normalisées permet aussi de rejouer le même export à une enveloppe de
vitesse différente sans réentraîner : on change `ActionCfg.scale`, pas le réseau.

## Décimation

En simulation la policy décide à 50 Hz pendant que la physique tourne à 200 Hz. Le même
découpage existe ici :

| | fréquence | rôle |
|---|---|---|
| `LocomotionRunner` | `policy_hz` (50 Hz) | décide |
| thread de contrôle de `PulsarDog` | `control_hz` (50 Hz) | republie la consigne rampée |
| contrôleur Unitree | interne | marche |

`LocomotionCfg.decimation(control_hz)` donne le rapport. `RunResult.late_steps` compte les
pas où la policy a dépassé son propre budget de temps — c'est le premier chiffre à
regarder quand le comportement ne ressemble pas à la simulation.

## Terminaisons = sécurité

En simulation, une terminaison coûte un reset. Sur le robot, elle arrête une machine de
15 kg. La distinction est portée par `Termination.is_fault` :

| Terme | Seuil par défaut | Faute ? |
|---|---|---|
| `bad_orientation` | 35° d'inclinaison | oui → arrêt d'urgence verrouillé |
| `base_height` | corps sous 0.15 m | oui |
| `battery` | SOC sous 20 % | oui |
| `telemetry_stale` | état de plus de 0.5 s | oui |
| `time_limit` | durée demandée | non → arrêt propre |

Tous les termes sont évalués à chaque pas, même après qu'un autre a déclenché : un terme
à état comme `time_limit` doit continuer à compter.

## Déployer une policy entraînée

1. Exporter l'acteur en TorchScript depuis l'entraînement (`torch.jit.save`).
2. Vérifier la forme : `pulsar-dog walk --describe` doit donner la dimension attendue
   par le réseau. `TorchScriptPolicy` refuse une observation de mauvaise taille plutôt
   que de produire une action silencieusement fausse.
3. Rejouer d'abord sans robot :
   `pulsar-dog --backend sim walk --policy policy.pt --duration 30 --record`
4. Puis sur le robot, enveloppe réduite :
   `pulsar-dog walk --policy policy.pt --duration 20 --max-vx 0.25 --max-vyaw 0.4 --record`
5. Comparer les logs sim/réel sur `action` et `velocity`. Un écart systématique, c'est
   une échelle qui ne correspond pas ; un écart qui grandit, c'est de la latence.

### Checklist sim2real

- [ ] même ordre de termes, mêmes échelles, même clip
- [ ] même `dt` de policy qu'à l'entraînement
- [ ] plages de commande de l'entraînement ⊇ plages utilisées ici
- [ ] `late_steps` ≈ 0 sur un run complet
- [ ] `telemetry_stale` jamais déclenché

## Ce qui n'est pas franchi

Une vraie policy de locomotion articulaire (12 positions cibles à 50 Hz, PD à 200 Hz sur
`rt/lowcmd`) demande de libérer le service sport d'Unitree, donc de retirer précisément la
couche qui empêche le robot de tomber. C'est faisable, ça se fait, mais ça se fait avec un
harnais de suspension, sur un banc, et dans un projet séparé — pas en extension de la
téléopération. La couche `locomotion/` est conçue pour que ce jour-là, seuls l'espace
d'action et les termes articulaires changent : commandes, observations, terminaisons,
décimation et journalisation restent en place.
