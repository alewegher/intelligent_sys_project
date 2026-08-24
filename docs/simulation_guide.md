# Guida alle simulazioni — `int_sys_fp`

Come eseguire e confrontare configurazioni diverse della simulazione in modo controllato.
Ogni fattore sperimentale è elencato con i valori ammessi, cosa cambia causalmente, e come
si attiva.

---

## 1. Comandi base

```bash
# Demo senza registrazione
ros2 launch int_sys_fp synchronized_system.launch.py

# Con registrazione bag → ~/ros2_ws/bags/int_sys_sim_<timestamp>/
ros2 launch int_sys_fp synchronized_system_with_bag.launch.py
```

La registrazione parte **16 s dopo il launch** (dopo lo spawn dell'ultimo robot a 13 s) e
cattura 45 topic (33 sempre + 12 solo in `gain_mode:=sdre_ci_experimental`).

**Le run con bag hanno durata fissa**: si fermano da sole dopo `sim_duration` secondi di
**tempo simulato** (default `125.664`, cioè un giro esatto di traiettoria), poi il bag viene
chiuso e indicizzato e tutti i processi terminano. Non serve Ctrl-C e non restano processi
appesi.

> **Perché tempo simulato e non reale**: un timer wall-clock darebbe run di lunghezza
> *simulata* diversa ogni volta che il real-time factor di Gazebo varia (carico CPU,
> configurazione più pesante), che è esattamente l'incoerenza da evitare. Poiché tutta
> l'analisi lavora su `header.stamp`, cioè in tempo simulato, anche la durata dev'essere
> definita lì. Il conteggio lo fa il nodo `run_timer`, il cui timer gira sul clock del nodo
> con `use_sim_time:=True`, quindi ticchetta su `/clock`. L'overshoot è al massimo il
> periodo di polling, 0.5 s.

Il launch di demo (`synchronized_system.launch.py`) mantiene invece `sim_duration:=0`, cioè
gira fino a Ctrl-C.

---

## 2. Fattori sperimentali

### 2.1 Attivabili da riga di comando

| Fattore | Valori | Default | Cosa cambia |
|---|---|---|---|
| `filter_type` | `ekf` \| `ukf` | `ekf` | Quale stimatore di posa gira sui 3 robot |
| `noise_type` | `1` \| `2` | `1` | 1 = gaussiano, 2 = uniforme. Seleziona `config/sensor_params.yaml` o `config/sensor_params_uniform.yaml` per l'emulatore |
| `enable_legacy_kf` | `true` \| `false` | `false` | Lancia `distance_kf_node` (KF scalare sulle distanze + MAD) |
| `enable_planner` | `true` \| `false` | `true` | Se `false` nessuno pubblica `/desired_trajectory`: i robot formano il triangolo nell'origine e non tracciano nulla |
| `enable_plotjuggler` | `true` \| `false` | `true` | Solo visualizzazione |
| `enable_bag_record` | `true` \| `false` | `true` | Solo nel launch file con bag |
| `gain_mode` | `one_step_riccati` \| `sdre_ci_experimental` | `one_step_riccati` | Attiva la fusione SDRE + Covariance Intersection |
| `ci_weight` | `[0, 1]` | `0.5` | Peso della fusione CI: 0.5 = media esatta delle due stime |
| `ci_weight_mode` | `fixed` \| `min_trace` | `fixed` | `min_trace` sceglie `w` minimizzando `trace(P_ci)` — **vedi §5.1 prima di usarlo** |
| `sdre_cov_mode` | `propagated` \| `steady_state` | `propagated` | Come si calcola `P_B` del ramo SDRE |
| `ci_feedback` | `true` \| `false` | `true` | `false` = shadow mode: la fusione viene calcolata e pubblicata ma **non** entra nel loop di controllo |
| `uwb_source` | `raw` \| `filtered` | `raw` | Se `filtered`, i filtri di posa consumano l'uscita del MAD invece delle UWB grezze |
| `enable_mad` | `true` \| `false` | `true` | Abilita il rilevatore di outlier nel KF legacy |
| `mad_window` | intero | `5` | Ampiezza della finestra scorrevole |
| `mad_kappa` | double | `3.0` | Moltiplicatore di soglia: outlier se `\|z − mediana\| > κ·scale·MAD` |
| `mad_scale` | double | `1.0` | Fattore di consistenza gaussiana — **vedi §5.2** |
| `mad_cov_inflation` | double | `1.5` | Inflazione della covarianza quando scatta un outlier |
| `distance_process_noise` | double | `0.01` | Rumore di processo del KF scalare — **vedi §5.3** |
| `radius` | double | `3.0` | Raggio della traiettoria circolare |
| `angular_velocity` | double | `0.05` | Velocità angolare del riferimento |
| `sim_duration` | double | `125.664` (bag) / `0` (demo) | Durata della run in **secondi di tempo simulato**, contati da quando parte la registrazione. `0` = fino a Ctrl-C. Allo scadere il bag viene chiuso e tutto termina |

Esempio:
```bash
ros2 launch int_sys_fp synchronized_system_with_bag.launch.py \
  filter_type:=ukf gain_mode:=sdre_ci_experimental ci_weight:=0.5
```

### 2.2 Richiedono `colcon build`

Gli YAML stanno in `src/int_sys_fp/config/` e vengono letti dal path installato in
`share/int_sys_fp/config/`, quindi modificarli richiede una ricostruzione.

> **Fallo una volta e il problema sparisce**: `colcon build --symlink-install` fa sì che
> `install(FILES ...)` crei symlink invece di copie, quindi da quel momento le modifiche
> agli YAML sono immediate.

| File | Parametro | Default | Effetto |
|---|---|---|---|
| `config/pose_filter_params.yaml` | `Q.{x_var,y_var,theta_var}` | `0.001` | Rumore di processo del filtro di posa |
| | `initial_covariance.p0_scale` | `10.0` | `P0 = p0_scale · I` |
| | `imu_orientation_noise_std` | `0.02` rad | σ del rumore sintetico su θ **e** `R(5,5)` |
| | `ukf.alpha` | `1.0` | Spread dei sigma point — **vedi §4** |
| | `ukf.beta` | `2.0` | Ottimale per distribuzioni gaussiane |
| | `ukf.kappa` | `0.0` | Regola classica `κ = 3 − L`, con L=3 |
| `config/sensor_params.yaml` | posizioni anchor, `min/max range` | vedi file | Geometria e saturazione |
| | `noise model.stddev` | `0.01` m | σ del rumore UWB — **il livello di rumore è un asse sperimentale a sé** |
| | `anchor 1: frequency` | `50` | Rate dell'emulatore UWB (il launch arg `uwb_frequency` è **morto**) |
| `config/controller.yaml` | `control_frequency` | `50.0` | Rate del loop di controllo |
| | `position_gains.Kp_r1` | `[10,10,0]` | Guadagno proporzionale — **solo `_r1` viene letto**, per tutti e tre i robot |
| | `integral_gains.Ki_r1` | `[0.01,0.01,0]` | Guadagno integrale |
| | `max_velocities.max_vel_r` | `[0.22,0.22,0]` | Saturazione di velocità |
| | `phase_transition_ramp_s` | `1.5` | Durata della rampa FORMATION→TRACKING |

---

## 3. Protocollo di confronto

### 3.1 La simulazione non è riproducibile, e va bene così

Due run della stessa identica configurazione **non producono gli stessi dati**, e non esiste
un parametro che possa cambiarlo:

- **Fisica e rumore dei sensori Gazebo**: il plugin IMU ha il proprio stream casuale; lo
  stepping ODE non è bit-riproducibile.
- **Timer wall-clock**: tutti i nodi C++ usano `create_wall_timer` pur avendo
  `use_sim_time:=True`, quindi il numero di tick per unità di tempo simulato varia col
  carico della CPU.
- **`dt` è hardcoded a 1/50** e mai misurato: il jitter dei timer entra come errore di
  modello invece di essere compensato.
- **Pose dei vicini asincrone**: ogni filtro usa l'ultimo messaggio arrivato, che dipende
  dallo scheduling. È inerente all'architettura distribuita.

Rendere deterministico tutto questo richiederebbe una sincronizzazione esplicita dei nodi
(servizi dedicati, clock guidato a passi) — un'infrastruttura fragile e sproporzionata
rispetto al valore che porterebbe. **La strada corretta per una simulazione stocastica è
statistica: si ripetono le run e si confrontano le distribuzioni, non le singole
realizzazioni.**

### 3.2 Ripetizioni: parti da 3, poi decidi

1. Vari **un solo fattore** alla volta rispetto alla baseline.
2. Esegui **3 run per configurazione** e calcola media e deviazione standard di ogni
   metrica (RMSE, errore di tracking medio, NIS medio).
3. Confronta la **varianza tra run della stessa configurazione** con la **differenza tra
   configurazioni**:
   - se la differenza tra configurazioni è molto maggiore della dispersione entro
     configurazione, 3 run bastano e la conclusione è già solida;
   - se sono confrontabili, aumenta le ripetizioni **solo su quelle configurazioni**, fino a
     che l'errore standard della media scende sotto la differenza che vuoi dichiarare.
4. Riporta sempre media ± deviazione standard su N run, **mai il valore di una singola run**.

Con `sim_duration:=125.664` (un giro) le run durano ~2.5 minuti reali a RTF≈1, quindi 3
ripetizioni costano ~8 minuti per configurazione. Poiché la durata è fissata in tempo
simulato, **tutte le run producono la stessa quantità di dati** e le metriche sono
confrontabili senza rinormalizzare.

⚠️ **Il transitorio si mangia parte del giro.** La registrazione parte a 16 s ma i robot
devono ancora convergere da FORMATION a TRACKING, quindi la finestra a regime è **più corta
di un giro completo**. Se in analisi serve un giro intero già a regime, usare
`sim_duration:=180`. Se si cambia `angular_velocity`, un giro diventa `2π/ω` secondi.

### 3.3 Un bag da solo non dice quale configurazione l'ha prodotto

I nomi dei topic sono **identici** per `pose_ekf_node` e `pose_ukf_node`. Registra sempre la
configurazione insieme al bag. Convenzione suggerita:

```
bags/
  ekf_riccati_gauss_r1/          # filtro_gainmode_rumore_ripetizione
  ekf_riccati_gauss_r2/
  ekf_riccati_gauss_r3/
  ukf_riccati_gauss_r1/
  ekf_sdreci_gauss_r1/
```

e un `config.txt` accanto a ogni bag con la riga di comando esatta.

### 3.4 Finestra di regime

Escludi il transitorio iniziale. La formazione parte con errore ≈0.71 m (spawn a
`(0,0)`, `(1,0)`, `(0,1)` contro lato desiderato 1.5 m) contro una soglia di convergenza di
0.15 m, e `P0 = 10·I` deve collassare. Usa `t > 50 s` come nel paper di riferimento, oppure
individua il primo istante in cui tutti e tre i `fsm_state.phase` valgono 1.

---

## 4. Taratura dell'UKF

La relazione è `λ = α²(L + κ) − L`, con `L = 3` (stato `[x, y, θ]`).

| α | λ | Spread sigma point | `Wm0` | Comportamento |
|---|---|---|---|---|
| `1e-3` | `−2.999997` | `1.7e-3·chol(P)` | `−1e6` | **Degenera esattamente nell'EKF** |
| `0.5` | `−2.25` | `0.87·chol(P)` | `−3.0` | Peso centrale negativo: `P` può diventare indefinita |
| `1.0` | `0` | `1.73·chol(P)` | `0` | **Default consigliato**: tutti i pesi ben condizionati |

Regole euristiche: `κ = 3 − L` (Julier) → `0` con L=3; `β = 2` è ottimale per distribuzioni
gaussiane; `α ∈ [1e-3, 1]`.

> **`α → 0` fa convergere lo *scaled UT* esattamente alla linearizzazione dell'EKF.** È una
> proprietà matematica nota, non un bug. Se giri l'UKF con `α = 1e-3` stai eseguendo un EKF
> travestito, e un confronto EKF vs UKF non può mostrare differenze. Il valore `1e-3` resta
> utile come **caso di controllo** dello sweep su α.

---

## 5. Accoppiamenti obbligati e trappole di interpretazione

### 5.1 `ci_weight_mode: min_trace` degenera con `sdre_cov_mode: propagated`

Con `propagated`, `P_B` è la covarianza propagata della stima SDRE **sullo stesso prior**
della stima classica. Ma `K_A` è per definizione il guadagno che minimizza la covarianza a
posteriori per quel prior, quindi `P_A ⪯ P_B` sempre (ordine di Loewner), quindi
`P_A⁻¹ ⪰ P_B⁻¹`, quindi `trace(P_ci(w))` è **monotona decrescente in `w`** e il minimo cade
sempre all'estremo `w = 1` — cioè sull'EKF puro.

**È matematicamente corretto**, non un difetto: è un risultato riportabile di per sé. Ma
significa che `min_trace` è un grado di libertà non banale **solo** se abbinato a
`sdre_cov_mode:=steady_state`, dove `P_B` è la covarianza stazionaria di progetto e non è
legata al prior corrente.

### 5.2 `3·MAD` non è una soglia a 3σ

Lo stimatore consistente con la gaussiana è `1.4826·MAD`. La soglia `3·MAD_grezzo` del
codice corrisponde quindi a **≈2.02σ**, non a 3σ, nonostante il paper citi "3·MAD". Il
default `mad_scale: 1.0` preserva il comportamento storico; usa `mad_scale:=1.4826` per una
vera regola a 3σ. Da dichiarare esplicitamente in tesi per non citare male la propria soglia.

### 5.3 Il KF legacy è un pass-through al ~99%

Con `distance_process_noise = 0.01` e `R = 1e-4`, il guadagno stazionario del KF scalare
vale `K ≈ 0.99`: l'uscita filtrata è quasi identica all'ingresso grezzo. Conseguenza:
**`uwb_source:=filtered` sembrerà identico a `raw` a meno che il MAD non scatti.** Per
rendere visibile l'asse "filtraggio delle distanze" bisogna abbassare
`distance_process_noise` (che infatti è esposto come parametro).

### 5.4 Il MAD attuale non fa quello che dice il paper

Il codice inserisce nella storia la misura **grezza**, ma confronta contro quella mediana la
**stima del KF**. Rileva quindi *"la stima del KF si è allontanata dai dati recenti"*, non
*"questa misura è un outlier"* — l'opposto del paper, che pre-filtra la misura. Sarà
disponibile come `mad_stage`: `post_estimate` (default, comportamento attuale) e
`pre_measurement` (riproduce il paper).

### 5.5 La CI qui è conservativa per costruzione

La Covariance Intersection è pensata per fondere stime con correlazione incrociata
**ignota**. Qui i due rami condividono lo stesso prior *e* la stessa misura, quindi sono
quasi perfettamente correlati: `P_ci` è garantita non ottimistica ma è strettamente più
grande della vera covarianza d'errore. In anello chiuso questa conservatività rientra nel
`predict` successivo e si accumula. **Aspettati che NIS/NEES risultino "underconfident" e le
ellissi 3σ gonfiate**: è comportamento corretto, non un bug.

### 5.6 `noise_type:=2` (uniforme): cosa è stato corretto

Prima era inutilizzabile: `config/sensor_params_uniform.yaml` metteva l'anchor 1 in `(-10,0,0)`
mentre i filtri leggevano sempre `config/sensor_params.yaml` dove sta in `(0,0,0)` — errore di
modello di 10 m — e `R` restava tarata su `σ=0.01` contro un rumore uniforme `±0.1`.

Ora: l'anchor 1 è allineato a `(0,0,0)`, il parametro `noise_type` viene passato anche ai
filtri (che quindi leggono lo stesso file dell'emulatore), e `R` è derivata dal modello
effettivo — per un uniforme `±a` la deviazione standard equivalente è `(max−min)/√12`, cioè
`a/√3`, **non** `a`. Usare `a` come deviazione standard renderebbe il filtro
sovra-confidente di un fattore 3 in varianza.

⚠️ **I bag a rumore uniforme registrati prima di questa correzione restano invalidi** per
l'analisi di accuratezza.

---

## 6. Decoy e comportamenti fuorvianti

Cose che sembrano configurabili o significative ma non lo sono:

| Elemento | Realtà |
|---|---|
| `config/UKF_params.yaml` | **Nessun nodo lo legge.** Il nome inganna: contiene parametri del KF legacy sulle distanze, e alcuni valori divergono da quelli realmente usati in `KF.cpp` (es. `distance_var: 0.001` contro `0.01` effettivo). Modificarlo non fa nulla |
| `uwb_frequency` (launch arg) | Passato come parametro ROS ma **sovrascritto immediatamente** dal valore YAML. Il rate va cambiato in `config/sensor_params.yaml:7` |
| `velocity_gains.Kv_*` | **Nessun effetto.** `desired_velocity` è posto a zero e mai aggiornato, quindi il termine derivativo è identicamente nullo. Il controllore è di fatto un PI più un PD separato sull'heading |
| `Kp_r2`, `Kp_r3`, `Ki_r2`, `Ki_r3`, `Kv_r2`, `Kv_r3` | **Mai letti.** Tutte e tre le istanze leggono i guadagni `_r1` |
| `Kf` (guadagno di formazione morbida) | Il log all'avvio stampa `Kf=0.50` ma **il valore effettivo è 0.8**: il costruttore sovrascrive quello caricato dallo YAML |
| `shared_process_noise`, `per_robot.*` | Chiavi presenti in `config/pose_filter_params.yaml` ma **mai lette** |
| `adaptive_noise.*` in `config/UKF_params.yaml` | **Nessun codice di rumore adattivo esiste** da nessuna parte |
| `worlds/empty_world_highfreq.world` | **Non referenziato da nessun launch file** e non installato. Il mondo usato è `empty_world.world` di `turtlebot3_gazebo` |
| `src/KF.py` | Reimplementazione Python del KF sulle distanze, **mai lanciata** da nessun launch file |
| `final_3_robots.sh` | Script legacy di un'iterazione precedente, riferisce topic che non esistono più |
| Lato della formazione | **Hardcoded a 1.5 m** in `Regulator_node.cpp:103`, non in `config/controller.yaml` |
| Soglie della FSM | Tutte hardcoded: convergenza 0.15 m, debounce 10 cicli, soglia errore tracking 0.2 m |
| Posizioni di spawn e ritardi | Hardcoded nel launch file: `(0,0)` a 6 s, `(1,0)` a 10 s, `(0,1)` a 13 s |

---

## 7. Vincolo di fattibilità della traiettoria

La velocità tangenziale del riferimento è `v = R · ω`, e il TurtleBot3 burger è limitato a
**0.22 m/s** (`config/controller.yaml:17`).

| R | ω | v tangenziale | Fattibile? |
|---|---|---|---|
| 5.0 m | 0.05 rad/s | 0.25 m/s | ❌ oltre il limite — l'errore di tracking non converge mai e l'integratore satura |
| 3.0 m | 0.05 rad/s | 0.15 m/s | ✅ 68% del limite, con margine per le correzioni di formazione |
| 4.0 m | 0.05 rad/s | 0.20 m/s | ⚠️ 91% del limite, margine insufficiente |

Serve margine perché il moto di correzione della formazione si **somma** a quello di
tracking: se il riferimento già satura la velocità, il robot non ha autorità residua per
mantenere il triangolo. Regola pratica: `R · ω ≤ 0.15 m/s`.

---

## 8. Matrice di esperimenti consigliata

Ogni riga varia **un solo fattore** rispetto alla baseline, e va eseguita **3 volte**
(§3.2).

| # | Configurazione | Domanda a cui risponde |
|---|---|---|
| 1 | baseline: `filter_type:=ekf` | Riferimento |
| 2 | `filter_type:=ukf` | L'UKF è più preciso dell'EKF? |
| 3 | `filter_type:=ukf` + sweep su `ukf.alpha` ∈ {1e-3, 0.1, 0.5, 1.0} | Quanto conta lo spread dei sigma point? A `1e-3` deve coincidere con la riga 1 |
| 4 | sweep su `noise model.stddev` ∈ {0.005, 0.01, 0.05, 0.1} | Come si traduce l'incertezza UWB in RMSE di posizione? |
| 5 | `noise_type:=2` (dopo la correzione) | Il modello di rumore conta, a parità di varianza? |
| 6 | `gain_mode:=sdre_ci_experimental ci_weight:=0.5` | La fusione SDRE+CI migliora o peggiora? |
| 7 | riga 6 + `ci_feedback:=false` | Confronto pulito: stessa traiettoria, stesse misure per tutti gli stimatori |
| 8 | riga 6 + `ci_weight_mode:=min_trace sdre_cov_mode:=steady_state` | Il peso ottimo è meglio della media? |
| 9 | `enable_legacy_kf:=true uwb_source:=filtered` | Il pre-filtraggio delle distanze aiuta la stima di stato? |
| 10 | riga 9 + `enable_mad:=false` | **Isola l'effetto del solo MAD** rispetto al solo KF scalare |
| 11 | riga 9 + `mad_stage:=pre_measurement` | La variante del paper è migliore di quella implementata? |

La riga 10 è quella che rende il MAD un vero asse: senza di essa, confrontando la riga 9
con la baseline si misura l'effetto combinato di KF scalare **e** MAD.

---

## 9. Diagnosi rapida dei problemi

| Sintomo | Causa probabile |
|---|---|
| Tutto parte ma i robot non si muovono | Nessun `pose_estimate`: il controllo è gated finché non arrivano tutti e tre. Verifica che i filtri girino: `ros2 node list` |
| Nessun filtro nella lista dei nodi | Refuso in `filter_type` (es. `EKF` maiuscolo): oggi lancia **zero** filtri senza errore |
| I nodi partono ma `ros2 topic hz /pose_debug` non dà nulla | I filtri sono gated su `have_anchor_ && have_robot_dist_`: l'emulatore UWB non sta pubblicando |
| `ModuleNotFoundError: lxml` allo spawn dei robot | Il terminale ha un virtualenv Python attivo che nasconde i pacchetti di sistema. Vedi `.vscode/settings.json` → `python.terminal.activateEnvironment: false` |
| L'errore di tracking non converge mai | Velocità tangenziale del riferimento oltre 0.22 m/s — vedi §7 |
| EKF e UKF danno risultati identici | `ukf.alpha` troppo piccolo — vedi §4 |
| La run non si ferma mai | `sim_duration:=0` (default del launch di demo). Passare un valore > 0 |
| Il bag è più corto del previsto | Il `run_timer` conta da quando parte la registrazione (16 s), non dal launch. Verificare con `ros2 bag info` |
