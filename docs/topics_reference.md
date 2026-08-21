# Reference dei topic ROS 2 — `int_sys_fp`

Riferimento completo di ogni topic del sistema: nome, tipo di messaggio, chi lo pubblica,
chi lo consuma, frequenza e semantica dei campi (unità e frame). Pensato per poter scrivere
codice di parsing MATLAB senza aprire i sorgenti C++.

Riferito allo stato del codice **prima** delle modifiche descritte in
`docs/simulation_guide.md`. Le insidie della Sezione 6 sono la parte da leggere per prima se
stai scrivendo post-processing.

---

## 1. Convenzione dei namespace

Definita in `include/pose_dynamics.hpp:110-117`, spawn in `launch/simulation_common.py:71-87`.

| `robot_id` | Prefisso topic | Entità Gazebo | Spawn (x, y, z) | Indice negli array UWB | Campo in `AnchorDist`/`RobotDist` |
|---|---|---|---|---|---|
| 0 | *(nessuno, root)* | `turtlebot3_burger_1` | (0.0, 0.0, 0.01) | `0` | `distances_a1[0]` / `distances_r1` |
| 1 | `/tb3_2` | `turtlebot3_burger_2` | (1.0, 0.0, 0.01) | `1` | `distances_a1[1]` / `distances_r2` |
| 2 | `/tb3_3` | `turtlebot3_burger_3` | (0.0, 1.0, 0.01) | `2` | `distances_a1[2]` / `distances_r3` |

> **Trappola dell'off-by-one.** Il suffisso numerico nei nomi dei campi è **1-based**
> (`distances_r2`), mentre `robot_id` e gli indici degli array sono **0-based**. Quindi
> `robot_id = 1` → namespace `/tb3_2` → campo `distances_r2` → indice `1`.

**Vicini** (`pose_dynamics.hpp:97-104`), in ordine crescente saltando sé stessi:
`neighborIds(0) = {1,2}`, `neighborIds(1) = {0,2}`, `neighborIds(2) = {0,1}`.

**Nomi delle istanze dei nodi** (`simulation_common.py:90,100,108,115,124,129`):
`uwb_sensor_emulator`, `controller_node_{0,1,2}`, `pose_ekf_node_{0,1,2}` oppure
`pose_ukf_node_{0,1,2}`, `distance_kf_node`, `trajectory_planner`.

**Posizioni degli anchor** (`sensor_params.yaml:3,13,23`), frame world:
anchor 1 = `(0, 0, 0)`, anchor 2 = `(10, 0, 0)`, anchor 3 = `(0, 10, 0)` metri.

---

## 2. Messaggi custom

Generati come `int_sys_fp/msg/<Nome>` (`CMakeLists.txt:24-30`), definizioni in
`custom_messages/`.

### 2.1 `AnchorDist` — distanze robot↔anchor

```
std_msgs/Header header
float64[] distances_a1
float64[] distances_a2
float64[] distances_a3
```

| Campo | Lunghezza a runtime | Semantica |
|---|---|---|
| `header.stamp` | — | Tempo di **simulazione** (`use_sim_time:=True`). `UWB_utils_emulator.cpp:311` |
| `header.frame_id` | — | `"uwb_frame"` per i grezzi, `"kf_filtered"` per l'output del KF legacy. **Non è un frame TF reale**: nessuno lo pubblica. |
| `distances_a1` | sempre 3 | `distances_a1[r]` = distanza in **metri** dal robot `r` all'**anchor 1** |
| `distances_a2` | sempre 3 | idem, anchor 2 |
| `distances_a3` | sempre 3 | idem, anchor 3 |

> **Il layout è trasposto rispetto all'intuizione**: il *nome del campo* seleziona
> l'**anchor**, l'*indice dell'array* seleziona il **robot** (`UWB_utils_emulator.cpp:320-327`).
> In MATLAB la matrice 3×3 completa si ricostruisce come `D(robot+1, anchor+1) = [a1'; a2'; a3']'`.

### 2.2 `RobotDist` — distanze robot↔robot

```
std_msgs/Header header
float64[] distances_r1
float64[] distances_r2
float64[] distances_r3
```

Lunghezza sempre 2. Ogni riga è la lista dei vicini in ordine crescente saltando sé stessi:

| Campo | Robot proprietario | `[0]` | `[1]` |
|---|---|---|---|
| `distances_r1` | robot 0 (root) | d(0,1) → verso `/tb3_2` | d(0,2) → verso `/tb3_3` |
| `distances_r2` | robot 1 (`/tb3_2`) | d(1,0) → verso root | d(1,2) → verso `/tb3_3` |
| `distances_r3` | robot 2 (`/tb3_3`) | d(2,0) → verso root | d(2,1) → verso `/tb3_2` |

Metri. Simmetriche in aritmetica esatta ma **numericamente diverse** tra righe: il rumore è
estratto in modo indipendente per ogni elemento (`UWB_utils_emulator.cpp:357-359`), quindi
`distances_r1[0] != distances_r2[0]`.

### 2.3 `FsmState` — stato della macchina a stati del controllore

```
std_msgs/Header header
uint8   phase             # 0 = FORMATION, 1 = TRACKING
float64 formation_error   # metri
```

Riempito in `Regulator_node.cpp:327-331`.

- `header.frame_id` **non viene mai impostato** → stringa vuota.
- `phase`: vale `1` solo quando `current_phase == TRACKING`. Attenzione: commuta solo
  quando la **rampa di blending è completata** (`Regulator_node.cpp:320-323`), cioè
  `phase_transition_ramp_s` = 1.5 s (`controller.yaml:21`) **dopo** l'inizio della
  transizione.
- `formation_error`: `sqrt(e01² + e02² + e12²)` con `eij = ‖p_i − p_j‖ − 1.5 m`
  (`Regulator_node.cpp:216-228`; distanza desiderata hardcoded a 1.5 m alla riga `:103`).
  Calcolato dalle posizioni **stimate**, non dalle UWB grezze. Metri.

### 2.4 `PoseEstimateDebug` — trasparenza completa del filtro

Scritto in modo identico da entrambi i filtri: `PoseEKF_node.cpp:262-291`, `UKF.cpp:348-376`.

| Campo | Tipo | Unità / frame | Significato |
|---|---|---|---|
| `header.stamp` | Time | sim time | |
| `header.frame_id` | string | — | sempre `"world"` |
| `robot_id` | `uint8` | — | 0/1/2 — permette il demux senza dipendere dal namespace |
| `x` | `float64` | m, frame **world** | x̂ a posteriori |
| `y` | `float64` | m, frame world | ŷ a posteriori |
| `theta` | `float64` | **rad, wrappato in [−π, π]**, CCW da +X world | θ̂ a posteriori |

**`p[9]` — covarianza a posteriori P, 3×3 row-major** (`PoseEKF_node.cpp:273-275`):

| idx | elemento | unità | | idx | elemento | unità |
|---|---|---|---|---|---|---|
| `p[0]` | P(x,x) | m² | | `p[5]` | P(y,θ) | m·rad |
| `p[1]` | P(x,y) | m² | | `p[6]` | P(θ,x) | m·rad |
| `p[2]` | P(x,θ) | m·rad | | `p[7]` | P(θ,y) | m·rad |
| `p[3]` | P(y,x) | m² | | `p[8]` | P(θ,θ) | rad² |
| `p[4]` | P(y,y) | m² | | | | |

> **MATLAB**: `P = reshape(p, 3, 3)'` — la trasposizione serve perché `reshape` è
> column-major mentre questo array è row-major. Valore iniziale `P0 = 10·I`
> (`pose_filter_params.yaml:12`).

**`z[6]`, `z_pred[6]`, `innovation[6]` — layout dei 6 canali di misura**
(`pose_dynamics.hpp:54-67`, assemblati in `PoseEKF_node.cpp:223-225`):

| idx | canale | contenuto di `z` | contenuto di `z_pred` | unità |
|---|---|---|---|---|
| 0 | anchor 0 | distanza UWB robot→anchor 1 | `‖(x̂,ŷ) − (0,0)‖` | m |
| 1 | anchor 1 | distanza UWB robot→anchor 2 | `‖(x̂,ŷ) − (10,0)‖` | m |
| 2 | anchor 2 | distanza UWB robot→anchor 3 | `‖(x̂,ŷ) − (0,10)‖` | m |
| 3 | neighbor 0 | distanza UWB verso il vicino di **id minore** | `‖(x̂,ŷ) − p̂_n0‖` | m |
| 4 | neighbor 1 | distanza UWB verso il vicino di **id maggiore** | `‖(x̂,ŷ) − p̂_n1‖` | m |
| 5 | theta_imu | yaw IMU rumoroso (vedi §6) | `θ_pred` | rad |

- `innovation = z − z_pred`, con **solo l'indice 5 angle-wrapped** (`PoseEKF_node.cpp:53`).
  Gli indici 0–4 sono differenze in metri.
- Le posizioni dei vicini `p̂_n` sono **volutamente stale**: è l'ultima `pose_estimate`
  ricevuta, congelata per l'update (`PoseEKF_node.cpp:111-116`). Nessuna fusione della
  cross-covarianza — è la scelta "naive" della stima distribuita.

**`q_diag[3]`** — diagonale del rumore di processo usata nel ciclo:
`q_diag[0] = q_diag[1] = 0.001 m²`; `q_diag[2] = theta_var + R_ω·dt²`
(`PoseEKF_node.cpp:218-219`). Con σ giroscopio = 2e-4 rad/s → `R_ω = 4e-8` → il termine di
inflazione vale ≈1.6e-11, numericamente invisibile. **In pratica `q_diag` resta costante a
`[0.001, 0.001, 0.001]` per tutta la run.**

**`r_diag[6]`** — diagonale del rumore di misura (`PoseEKF_node.cpp:227-233`):
`r_diag[0..2] = σ_anchor²= 1e-4 m²`; `r_diag[3..4] = σ_robot² = 1e-4 m²` (indicizzato
sull'id **del vicino**, non del proprietario); `r_diag[5] = 0.02² = 4e-4 rad²`.
**Costante per tutta la run** — nessun rumore adattivo. `R` è strettamente diagonale.

**Scalari:**

| Campo | Unità / frame | Note |
|---|---|---|
| `have_imu` | — | `true` da quando arriva il **primo** messaggio IMU; non torna mai `false` |
| `v_cmd` | m/s, **body frame** | ultimo `cmd_vel.linear.x` — valore **comandato**, non misurato |
| `omega_imu` | rad/s, **body frame** | ultimo `imu.angular_velocity.z`, include il rumore Gazebo (σ=2e-4) |
| `dt` | s | **sempre la costante `0.02`** (`PoseEKF_node.cpp:83`), non il tempo trascorso reale |

---

## 3. Topic pubblicati dai nodi `int_sys_fp`

### 3.1 `/uwb/anchor_distances` — `int_sys_fp/msg/AnchorDist`

| | |
|---|---|
| **Publisher** | `uwb_sensor_emulator` — `UWB_utils_emulator.cpp:60-61`. Istanza globale unica, senza namespace |
| **Subscriber** | `pose_ekf_node` (`PoseEKF_node.cpp:100-102`), `pose_ukf_node` (`UKF.cpp:189-191`), `distance_kf_node` (`KF.cpp:46-48`, spento di default) |
| **Frequenza** | 50 Hz — timer da `sensor_params.yaml:7` (`anchor 1: frequency`), **non** dal launch arg |
| **Contenuto** | 3 array da 3 double, distanze euclidee robot→anchor in metri, calcolate nel frame world dalle posizioni `/odom`, poi rumorizzate e saturate |
| **Registrato** | sì (`simulation_common.py:164`) |

### 3.2 `/uwb/robot_distances` — `int_sys_fp/msg/RobotDist`

Stesso publisher, stesso timer (pubblicati nella stessa callback `publish_data()`,
`UWB_utils_emulator.cpp:307-366`), stessi subscriber. Distanze inter-robot in metri, frame
world. Registrato.

### 3.3 `/uwb/filtered_anchor_distances`, `/uwb/filtered_robot_distances`

| | |
|---|---|
| **Publisher** | `distance_kf_node` — `KF.cpp:55-56`, `:58-59`. **Esiste solo con `enable_legacy_kf:=true`** (default `false`) |
| **Subscriber** | **nessuno.** Completamente fuori dal path di controllo |
| **Frequenza** | nominalmente 50 Hz (`KF.cpp:30-31,62-64`) |
| **Contenuto** | 15 distanze filtrate da altrettanti KF scalari indipendenti. Mappatura indice piatto → messaggio in `KF.cpp:377-402`: indici 0–8 = anchor in ordine robot-major (`robot*3 + anchor`), 9–14 = inter-robot (`9 + robot*2 + k`) |
| **Registrato** | elencato in `BAG_TOPICS` (`simulation_common.py:165`) ma **assente da ogni bag di default**, perché il nodo non viene lanciato. Il codice MATLAB deve tollerarne l'assenza |

### 3.4 `{ns}/pose_estimate` — `geometry_msgs/msg/PoseWithCovarianceStamped`

Istanze: `/pose_estimate`, `/tb3_2/pose_estimate`, `/tb3_3/pose_estimate`.

| | |
|---|---|
| **Publisher** | `pose_ekf_node` (`PoseEKF_node.cpp:118-119`) **oppure** `pose_ukf_node` (`UKF.cpp:207-208`) — mutuamente esclusivi via `filter_type` |
| **Subscriber** | `regulator_node` proprio (`Regulator_node.cpp:67-73`, legge x, y **e** yaw); `regulator_node` dei 2 vicini (`:77-83`, solo x, y); `pose_ekf_node`/`pose_ukf_node` dei 2 vicini (`PoseEKF_node.cpp:111-116`, solo x, y) |
| **Frequenza** | 50 Hz. **Gated**: `step()` esce subito finché non sono arrivate sia le distanze anchor sia quelle robot (`PoseEKF_node.cpp:212`), quindi nulla viene pubblicato prima dei primi messaggi UWB |

**Contenuto** (`PoseEKF_node.cpp:243-260`):
- `header.frame_id = "world"`, `header.stamp` = sim time
- `pose.pose.position.x/.y` in metri, frame world. **`position.z` resta 0.0**
- `pose.pose.orientation` — θ codificato come **quaternione di solo yaw**:
  `z = sin(θ/2)`, `w = cos(θ/2)`, `x = y = 0`. Si recupera con `theta = 2*atan2(qz, qw)`;
  poiché il filtro wrappa θ in [−π,π], risulta sempre `qw ≥ 0`, quindi la decodifica è
  esatta e univoca
- `pose.covariance` — 36 elementi row-major su `[x, y, z, rot_x, rot_y, rot_z]`. Tutti
  azzerati, poi **solo 5 scritti**:

  | indice | (riga, col) | valore |
  |---|---|---|
  | 0 | (x, x) | `P(0,0)` |
  | 1 | (x, y) | `P(0,1)` |
  | 6 | (y, x) | `P(1,0)` |
  | 7 | (y, y) | `P(1,1)` |
  | 35 | (rot_z, rot_z) | `P(2,2)` |

  Gli altri 31 valgono esattamente 0.0. **I cross-termini x–θ e y–θ vengono buttati via**
  (indici 5, 11, 30, 31) pur essendo non nulli dentro il filtro → usare `pose_debug.p[]`.

**Registrato**: sì (`simulation_common.py:169`).

### 3.5 `{ns}/pose_debug` — `int_sys_fp/msg/PoseEstimateDebug`

| | |
|---|---|
| **Publisher** | `PoseEKF_node.cpp:121-122` / `UKF.cpp:210-211` |
| **Subscriber** | **nessuno** a runtime. Solo bag e PlotJuggler — **è il topic principale per MATLAB** |
| **Frequenza** | 50 Hz, stessa callback `step()`, pubblicato subito dopo `pose_estimate` → i due topic sono 1:1 con timestamp identici |
| **Contenuto** | vedi §2.4 |
| **Registrato** | sì (`simulation_common.py:170`) |

### 3.5b Topic aggiuntivi in `gain_mode:=sdre_ci_experimental`

Creati solo quando quella modalità è attiva; nei bag `one_step_riccati` semplicemente non
compaiono (`ros2 bag record` tollera un topic elencato che non esiste mai).

| Topic | Tipo | Contenuto |
|---|---|---|
| `{ns}/pose_debug_riccati` | `PoseEstimateDebug` | Stima A — one-step Riccati (`x_A`, `P_A`) |
| `{ns}/pose_debug_sdre` | `PoseEstimateDebug` | Stima B — guadagno stazionario dalla DARE (`x_B`, `P_B`) |
| `{ns}/pose_debug_ci` | `PoseEstimateDebug` | Stima fusa (`x_ci`, `P_ci`). Pubblicato **solo** se la fusione è riuscita |
| `{ns}/gain_mode_debug` | `std_msgs/msg/Float64MultiArray` | `[w, iterazioni_dare, dare_ok, residuo_dare]` |

- `{ns}/pose_debug` continua a portare **la stima effettivamente sul path di controllo**:
  quella fusa se `ci_feedback:=true`, quella classica altrimenti. È ciò che permette agli
  script di analisi esistenti di funzionare invariati sui bag SDRE-CI.
- **Tutti i messaggi di un ciclo condividono lo stesso `header.stamp`**, quindi i quattro
  topic si uniscono con una join esatta sul timestamp.
- `gain_mode_debug` usa un tipo `std_msgs` **builtin**: MATLAB lo legge senza generare
  messaggi custom.
- `dare_ok == 0` significa che il solver non ha converso e quel ciclo è ricaduto sulla stima
  classica. La frazione di cicli con `dare_ok == 0` è la prima cosa da guardare.

---

### 3.6 `{ns}/cmd_vel` — `geometry_msgs/msg/Twist`

| | |
|---|---|
| **Publisher** | `regulator_node` — `Regulator_node.cpp:40`, pubblicato a `:386` |
| **Subscriber** | plugin Gazebo `libgazebo_ros_diff_drive.so`; `pose_ekf_node` (`:92-94`); `pose_ukf_node` (`UKF.cpp:181-183`) — **i filtri usano solo `linear.x`** come input `v` del modello di processo |
| **Frequenza** | 50 Hz (`controller.yaml:2`). **Gated**: non pubblica nulla finché non ha visto tutti e tre gli stream `pose_estimate` (`Regulator_node.cpp:281-285`) |

**Contenuto — frame body** (`Regulator_node.cpp:371-384`):
- `linear.x = desired_speed · max(0.5, cos(heading_error))` m/s. `desired_speed` saturata a
  `max_vel_r[0] = 0.22` m/s. Il floor `max(0.5, ·)` fa sì che **il robot non scenda mai
  sotto metà velocità nemmeno guardando a 90°+ dalla direzione voluta** — comportamento
  intenzionale ma non ovvio
- `linear.y` esplicitamente `0.0`; `linear.z`, `angular.x`, `angular.y` mai assegnati → 0.0
- `angular.z = 2.5·heading_error + 0.8·d(heading_error)/dt` rad/s (guadagni **hardcoded**,
  non da YAML), saturata a `max_omega_r[0] = 2.84` rad/s
- **`Twist` non ha header → nessun timestamp.** In MATLAB si usa il receive time del bag

**Registrato**: sì (`simulation_common.py:168`).

### 3.7 `{ns}/fsm_state` — `int_sys_fp/msg/FsmState`

| | |
|---|---|
| **Publisher** | `Regulator_node.cpp:48`, pubblicato a `:331` |
| **Subscriber** | le altre due istanze di `regulator_node` (`:52-59`) — consenso AND: un vicino conta come pronto se `phase==1` **oppure** se il suo `formation_error < 0.15` (`:306-308`) |
| **Frequenza** | 50 Hz, stessa `control_loop()` gated |
| **Registrato** | sì (`simulation_common.py:171`) |

### 3.8 `{ns}/tracking_error` — `std_msgs/msg/Float64MultiArray`

| | |
|---|---|
| **Publisher** | `Regulator_node.cpp:43`, da `publish_tracking_error()` (`:401-406`) |
| **Subscriber** | nessuno |
| **Frequenza** | 50 Hz, `control_loop()` gated |
| **Contenuto** | `data = [ex, ey]`, esattamente 2 elementi, **metri, frame world**: `desired_position − centroid`. Il campo `layout` è lasciato **completamente vuoto** |
| **Registrato** | sì (`simulation_common.py:172`) |

Ogni robot pubblica la **propria** stima della stessa quantità: i tre topic portano valori
quasi identici ma non bit-identici, perché ogni controllore ha copie ricevute in momenti
diversi delle pose dei vicini.

### 3.9 `{ns}/centroid_position` — `std_msgs/msg/Float64MultiArray`

`data = [cx, cy]`, metri, frame world. Media non pesata delle tre posizioni **stimate**
(`Regulator_node.cpp:207-214`). Publisher `:45`, pubblicato a `:289-291`, 50 Hz gated,
nessun subscriber, `layout` vuoto. Stessa avvertenza sulle tre copie quasi-identiche.
Registrato (`simulation_common.py:173`).

### 3.10 `{ns}/desired_trajectory_array` — `std_msgs/msg/Float64MultiArray`

| | |
|---|---|
| **Publisher** | `Regulator_node.cpp:44` — pubblicato **dentro `trajectory_callback`** (`:202-204`), *non* nel control loop |
| **Frequenza** | guidata dagli arrivi di `/desired_trajectory`, cioè **50 Hz sul clock di simulazione** e non sul timer wall-clock del controllo. Con RTF ≠ 1 il rate differisce da `tracking_error`/`centroid_position` |
| **Contenuto** | `data = [x_des, y_des]`, metri, frame world — puro eco della Pose del planner |
| **Registrato** | sì, tutti e tre (`simulation_common.py:174-175`) |

### 3.11 `/desired_trajectory` — `geometry_msgs/msg/Pose`

| | |
|---|---|
| **Publisher** | `trajectory_planner` — `trajectory_planner.py:18`. **Topic globale unico, senza namespace** — riferimento broadcast condiviso |
| **Subscriber** | tutte e tre le istanze di `regulator_node` (`Regulator_node.cpp:62-64`). Vengono letti **solo** `position.x` e `position.y`; l'orientazione è ignorata |
| **Frequenza** | 50 Hz. `rclpy.create_timer` usa il **clock del nodo** e `use_sim_time` è `True` → **è l'unico timer a 50 Hz del sistema che gira in tempo di simulazione** |

**Contenuto** (`trajectory_planner.py:42-51`): percorso circolare attorno all'origine world.
`position.x = R·cos(φ)`, `position.y = R·sin(φ)`, `position.z = 0`, `orientation` identità
(inutilizzata). Il launch **sovrascrive** i default del file: **R = 5.0 m** e
**ω = 0.05 rad/s** (`simulation_common.py:131-132`; i valori 2.0/0.2 nel `.py` non sono
quelli che girano). Periodo = 2π/0.05 ≈ 125.7 s. `φ` avanza di `ω·dt` con `dt` hardcoded a
`1/update_rate` e wrappa a 2π, quindi `φ ∈ [0, 2π)`, mai negativo.

**`Pose` non ha header → nessun timestamp.** Registrato (`simulation_common.py:174`).

---

## 4. Topic consumati dall'esterno (Gazebo / TurtleBot3)

Plugin del modello: `/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`.
Tutti i tag `<namespace>` nel SDF sono **commentati**, quindi il namespace arriva
esclusivamente da `spawn_entity.py -robot_namespace`.

### 4.1 `{ns}/odom` — `nav_msgs/msg/Odometry` — **sottoscritto**

| | |
|---|---|
| **Publisher** | `libgazebo_ros_diff_drive.so` (`model.sdf:381-412`) |
| **Subscriber nel pacchetto** | **solo** `uwb_sensor_emulator` (`UWB_utils_emulator.cpp:75-85`). **I filtri di posa NON si sottoscrivono a odom** — non è un input dello stimatore, ed è per questo che può fare da proxy (parziale) di ground truth |
| **Frequenza** | **30 Hz** (`model.sdf:387`) — diversa da ogni topic `int_sys_fp`. Va ricampionata/interpolata prima di confrontarla con `pose_debug` |
| **Campi usati** | `pose.pose.position.x/.y/.z` → posizione del robot. `twist.twist.linear.*` viene salvato ma **non usato per niente** |

**Frame**: `header.frame_id = "odom"`, `child_frame_id = "base_footprint"`, presi
letteralmente dal SDF → **tutti e tre i robot dichiarano le stesse identiche stringhe di
frame**. `pose` è nel frame `odom`, `twist` nel frame **body**. La sorgente dell'odometria
è world-based (necessario perché l'emulatore UWB calcola le distanze inter-robot
direttamente da queste coordinate, quindi devono essere coordinate world comuni).

> **Verifica una volta a runtime**: `ros2 topic echo /tb3_2/odom --once` — se a t=0
> `position.x ≈ 1.0` allora è effettivamente world-sourced.

`pose.covariance` / `twist.covariance` contengono i default del plugin (il SDF del burger
non imposta nulla): sono costanti convenzionali, **non incertezza reale** — ignorarle.

**Registrato**: sì, tutti e tre (`simulation_common.py:166`).

### 4.2 `{ns}/imu` — `sensor_msgs/msg/Imu` — **sottoscritto**

| | |
|---|---|
| **Publisher** | `libgazebo_ros_imu_sensor.so` (`model.sdf:93-98`) |
| **Subscriber** | `pose_ekf_node` (`:96-98`), `pose_ukf_node` (`UKF.cpp:185-187`). Ogni robot ascolta **solo il proprio** IMU |
| **Frequenza** | **200 Hz** (`model.sdf:50`) — 4× il rate del filtro, quindi il filtro usa solo il campione più recente per ciclo e **scarta silenziosamente ~3 su 4** |

**Campi effettivamente usati:**
- `angular_velocity.z` → input ω del modello di processo. rad/s, **frame body**. Rumore
  Gazebo σ = 2e-4 rad/s
- `angular_velocity_covariance[8]` → `R_ω`, cioè l'elemento (z,z) della 3×3 row-major,
  rad²/s². **È effettivamente popolato** dal plugin → 4e-8
- `orientation.{w,x,y,z}` → yaw estratto con `atan2(2(wz+xy), 1−2(y²+z²))`. Riferito al
  world (REP-145)
- `orientation_covariance` **non viene mai letto**: il plugin non lo popola mai (nel SDF del
  burger non esiste un blocco di rumore sull'orientazione), quindi è tutto zeri. È questo il
  motivo per cui il rumore su θ viene iniettato via software — vedi §6, Q12
- `linear_acceleration` e la sua covarianza **non sono usati affatto**

**Frame**: `header.frame_id = "imu_link"`, **stringa identica su tutti e tre i robot**.

**Registrato**: sì, tutti e tre (`simulation_common.py:167`).

### 4.3 `/clock` — `rosgraph_msgs/msg/Clock`

Pubblicato da `libgazebo_ros_init.so`. Tutti i nodi girano con `use_sim_time: True` e il bag
è registrato con `--use-sim-time`, quindi **sia gli header sia i receive-timestamp del bag
sono in tempo di simulazione**. Il mondo usato è `empty_world.world` di `turtlebot3_gazebo`
(`simulation_common.py:59`) — il file `worlds/empty_world_highfreq.world` presente nel
workspace **non è referenziato da nessun launch file**. Registrato
(`simulation_common.py:163`).

### 4.4 Topic che esistono ma **non sono usati né registrati**

| Topic | Tipo | Frequenza | Note |
|---|---|---|---|
| `{ns}/scan` | `sensor_msgs/msg/LaserScan` | 5 Hz | 360 campioni, 0→6.28 rad, range 0.12–3.5 m. Mai sottoscritto, mai registrato |
| `{ns}/joint_states` | `sensor_msgs/msg/JointState` | 30 Hz | Encoder ruote. **Non registrato** → nessun controllo incrociato indipendente dell'odometria è possibile offline |
| `/tf` | `tf2_msgs/msg/TFMessage` | 30 Hz × 3 | **Non registrato.** Inoltre `TransformBroadcaster` pubblica sul topic **assoluto** `/tf`, quindi `-robot_namespace` non lo namespacizza: tutti e tre i robot trasmettono `odom → base_footprint` con **gli stessi nomi di frame** sullo stesso topic → collisione reale di frame TF. Irrilevante qui (nulla usa TF), ma **non provare a usare TF per il post-processing** |
| `/tf_static` | — | — | **Non esiste**: nessun `robot_state_publisher` viene lanciato. Di conseguenza non esiste nemmeno `/robot_description` |
| `/gazebo/model_states`, `/gazebo/link_states` | `gazebo_msgs/...` | — | **Non esistono**: `gzserver.launch.py` carica solo `init`, `factory`, `force_system`, **non** `libgazebo_ros_state.so`. **Non c'è alcun topic di ground truth assoluta nel sistema** |
| `/parameter_events`, `/rosout` | — | — | Non registrati → il timestamp del log `"PHASE TRANSITION START"` non è recuperabile dal solo bag |

---

## 5. Cosa finisce nel bag

`BAG_TOPICS` è definito in `launch/simulation_common.py` (**45 topic**, di cui 12 solo in `gain_mode:=sdre_ci_experimental`). La
run si ferma da sola dopo `sim_duration` secondi di **tempo simulato** (default `125.664`
= un giro di traiettoria): il nodo `run_timer` conta su `/clock` ed esce, il launch
trasforma la sua uscita in uno `Shutdown` che manda SIGINT a `ros2 bag record`, così
rosbag2 chiude e indicizza il bag correttamente invece di lasciarlo troncato.
`sim_duration:=0` disattiva il limite. La
registrazione parte **16 s dopo il launch**,
cioè dopo lo spawn del terzo robot a 13 s, con `--use-sim-time`. Output in
`~/ros2_ws/bags/int_sys_sim_<YYYY_MM_DD-HH_MM_SS>/`. Il launch file
`synchronized_system.launch.py` **non registra nulla**.

**Registrati (33)**: `/clock`; `/uwb/anchor_distances`; `/uwb/robot_distances`;
`/uwb/filtered_anchor_distances`; `/uwb/filtered_robot_distances`; `{,/tb3_2,/tb3_3}/odom`;
`{,/tb3_2,/tb3_3}/imu`; `{,/tb3_2,/tb3_3}/cmd_vel`; `{,/tb3_2,/tb3_3}/pose_estimate`;
`{,/tb3_2,/tb3_3}/pose_debug`; `{,/tb3_2,/tb3_3}/fsm_state`;
`{,/tb3_2,/tb3_3}/tracking_error`; `{,/tb3_2,/tb3_3}/centroid_position`;
`/desired_trajectory`; `{,/tb3_2,/tb3_3}/desired_trajectory_array`.

> **Un bag da solo non dice quale filtro l'ha prodotto.** I nomi dei topic sono **identici**
> per `pose_ekf_node` e `pose_ukf_node`. Registra la configurazione insieme a ogni bag.

---

## 6. Insidie di codifica — da leggere prima di scrivere parsing

**Q1 — θ è codificato in due modi diversi.** `pose_estimate` lo porta come quaternione
(`z = sin(θ/2)`, `w = cos(θ/2)`), `pose_debug.theta` come double semplice. Entrambi wrappati
in [−π,π]. **Preferire `pose_debug.theta`, e fare `unwrap()` prima di derivare o plottare.**

**Q2 — `PoseWithCovarianceStamped.covariance` è 31/36 zeri.** Uno zero all'indice 14/21/28
significa "non popolato", **non** "varianza nulla". I cross-termini x–θ e y–θ esistono nel
filtro ma vengono **scartati** qui: usare `pose_debug.p[2]`, `p[5]`, `p[6]`, `p[7]`.

**Q3 — `-1.0` è la sentinella di fuori-range, ma non arriva mai in `pose_debug.z`.**
L'emulatore scrive `-1.0` quando una distanza esce da `[min range, max range]`
(`UWB_utils_emulator.cpp:400,415`), ma **ogni consumatore la filtra con una guardia `> 0.0`
e mantiene silenziosamente il valore precedente** (`PoseEKF_node.cpp:190-191,205-206`;
inizializzato a `1.0` m). Conseguenza: **un canale saturato appare come un valore congelato
e stantio, senza alcun flag.** Per rilevare la saturazione bisogna incrociare i topic
`/uwb/*` grezzi. Con la config gaussiana di default (`min 0.0`, `max 150`) la saturazione è
di fatto irraggiungibile; con quella uniforme (`min range: 0.1`) scatta sotto i 10 cm.

**Q4 — clamp di non-negatività prima della saturazione.** `add_noise()` porta a `0.01` m
qualsiasi distanza rumorizzata negativa (`UWB_utils_emulator.cpp:377,388`). Con
`min range: 0.0` questo sopravvive alla saturazione, quindi gli array grezzi possono
contenere `0.01`: è un floor artificiale, non una misura reale.

**Q5 — indicizzazione degli array.** In `AnchorDist` l'indice è il **robot** e il nome del
campo è l'**anchor**. In `RobotDist` l'indice è il vicino in ordine crescente saltando sé
stessi. Vedi §2.1 e §2.2.

**Q6 — l'emulatore calcola distanze 3-D, il modello del filtro ne predice 2-D.**
`compute_distances()` usa una norma `Vector3d` che include z
(`UWB_utils_emulator.cpp:280-282`), mentre `pose_dynamics::h()` usa solo `(x, y)`. Con
anchor a z=0 e base del robot a z≈0 il bias è trascurabile, ma è un **mismatch di modello
reale e permanente**, visibile in `innovation[0..2]`.

**Q7 — il launch arg `uwb_frequency` è morto.** Viene passato come parametro ROS ma
`UWB_utils_emulator.cpp:191-192` lo dichiara e **lo sovrascrive immediatamente** con il
valore YAML; il periodo del timer viene da quel valore. **Cambiarlo da riga di comando non
ha alcun effetto** — va modificato `sensor_params.yaml:7`.

**Q8 — le posizioni degli anchor arrivano sempre dallo YAML gaussiano.** L'emulatore sceglie
tra `sensor_params.yaml` e `sensor_params_uniform.yaml` secondo `noise_type`, ma **entrambi
i filtri hardcodano `sensor_params.yaml`** (`PoseEKF_node.cpp:150`, `UKF.cpp:242`). L'anchor
1 sta in `(0,0,0)` nel file gaussiano e in `(-10,0,0)` in quello uniforme. **Girare con
`noise_type:=2` fornisce quindi al filtro una posizione dell'anchor 1 sbagliata di 10 m**:
trattare ogni bag a rumore uniforme come non valido per l'analisi di accuratezza finché non
è corretto.

**Q9 — timer wall-clock contro timer sim-clock.** Tutti i nodi C++ usano
`create_wall_timer` → ticchettano sul **clock reale** a 50 Hz indipendentemente da
`use_sim_time`, pur timbrando i messaggi in tempo **simulato**. `trajectory_planner.py` usa
`create_timer` → clock del nodo → tempo **simulato**. Se il real-time factor di Gazebo non è
1.0, il rate apparente in tempo simulato dei topic C++ è `50 / RTF` Hz mentre
`/desired_trajectory` resta esattamente a 50 sim-Hz. **Calcolare sempre i rate reali da
`diff(header.stamp)`, mai assumere 50 Hz.**

**Q10 — `pose_debug.dt` è la costante `0.02`, non una misura.** Non riflette la deriva di Q9
né alcun jitter.

**Q11 — `z[5]` è autoreferenziale prima del primo messaggio IMU.**
`z(5) = have_imu_ ? last_theta_imu_ : x_pred(2)`. Quando `have_imu == false` la "misura" di
θ **è** la predizione, quindi `innovation[5]` vale esattamente 0.0 e il canale θ non porta
informazione. **Filtrare `have_imu == true` prima di analizzare il canale θ.**

**Q12 — la "misura" di θ contiene rumore iniettato via software.**
`last_theta_imu_ = wrap(yaw_vero + N(0, 0.02))`, con RNG da `std::random_device` →
**non riproducibile tra run**. L'orientazione IMU di Gazebo è quasi ground truth, quindi il
rumore è deliberato; `r_diag[5] = 4e-4` corrisponde esattamente.

**Q13 — `z_pred` significa cose diverse per EKF e UKF.** EKF: esattamente `h(x_pred)`.
UKF: la media pesata dei sigma point `Σ wᵐᵢ h(χᵢ)`, che differisce da `h(x_pred)` per la
non-linearità del modello. **Non confrontarli ingenuamente tra filtri diversi.** Il canale
5 (θ) usa in entrambi una media circolare, quindi resta in (−π, π] e le due convenzioni sono
almeno confrontabili sullo stesso intervallo.

**Q14 — tre tipi di messaggio non hanno alcun header.** `geometry_msgs/Pose`
(`/desired_trajectory`), `geometry_msgs/Twist` (`cmd_vel`) e `std_msgs/Float64MultiArray`
(`tracking_error`, `centroid_position`, `desired_trajectory_array`) **non hanno timestamp**.
In MATLAB vanno allineati con il receive time del bag.

**Q15 — `Float64MultiArray.layout` è sempre vuoto.** Mai popolato. `data` ha sempre
esattamente 2 elementi in ordine fisso `[x, y]`.

**Q16 — il termine derivativo del PID sul centroide è permanentemente nullo.**
`desired_velocity` è posto a zero in `Regulator_node.cpp:99` e mai più aggiornato, quindi
`velocity_error = desired_velocity − 0`. Il guadagno `Kv_r1` di `controller.yaml:9` **non ha
alcun effetto**: il controllore è di fatto un PI più un PD separato sull'heading.

**Q17 — il fattore di blending della FSM non è osservabile.** `phase_blend_` sale da 0 a 1
in 1.5 s ma `fsm_state.phase` commuta solo alla fine. L'istante di **inizio** della
transizione si può solo dedurre a ritroso dalla commutazione, o dal log
`"PHASE TRANSITION START"` (che però non è registrato nel bag).

**Q18 — tre copie quasi identiche di centroide ed errore di tracking.** Ogni
`regulator_node` le calcola dalle proprie copie ricevute in modo asincrono. I valori
differiscono a livello 1e-3. **Scegliere un namespace in modo consistente** (robot 0 è la
scelta naturale) invece di mediarli.
