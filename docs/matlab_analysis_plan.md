# Piano di analisi MATLAB — post-processing bag `int_sys_fp`

Specifica delle analisi e dei plot da generare in MATLAB a partire dai bag registrati con
`synchronized_system_with_bag.launch.py`. Non contiene codice: è la guida per scrivere gli
script `.m`.

**Documenti collegati**: [`topics_reference.md`](topics_reference.md) per la semantica dei
campi e le insidie di parsing; [`simulation_guide.md`](simulation_guide.md) per come
generare i bag di ciascuna configurazione.

---

## 1. Impianto: analisi fattoriale, non run singola

Il sistema espone più fattori indipendenti (filtro, modalità di guadagno, sorgente UWB,
modello e livello di rumore, α dell'UKF). **Ogni analisi va scritta come funzione
parametrizzata sulla configurazione**, non come script su un singolo bag, e le tabelle di
risultato diventano comparative.

Struttura suggerita degli script:

```
loadRun(bagPath)        -> struct con tutte le serie temporali allineate
analyzeRun(run)         -> struct di metriche per una singola run
compareRuns(runs, factor) -> tabelle e plot comparativi lungo un fattore
```

**Un bag da solo non dice quale configurazione l'ha prodotto**: i nomi dei topic sono
identici per EKF e UKF. Leggi la configurazione dal file `config.txt` che accompagna ogni
bag (convenzione in `simulation_guide.md` §3.3).

### 1.1 Ogni metrica è una statistica su più run, mai un valore singolo

La simulazione **non è riproducibile** e non può esserlo: timer wall-clock, fisica Gazebo e
arrivo asincrono delle pose dei vicini fanno sì che due run identiche diano dati diversi
(dettagli in `simulation_guide.md` §3.1). Di conseguenza:

- Ogni configurazione va eseguita **almeno 3 volte** e ogni metrica riportata come
  **media ± deviazione standard su N run**.
- Tutte le run hanno la **stessa durata in tempo simulato** (`sim_duration`, default un giro
  di traiettoria), quindi contengono la stessa quantità di dati e le metriche sono
  confrontabili senza rinormalizzare sulla lunghezza. Verificare comunque con
  `ros2 bag info` che la durata registrata corrisponda a quella richiesta.
- Attenzione: la registrazione parte a 16 s ma i robot devono ancora convergere a TRACKING,
  quindi **la finestra a regime è più corta di un giro**. Definire la finestra sul primo
  istante in cui tutti e tre i `fsm_state.phase` valgono 1, e verificare che sia abbastanza
  lunga per l'analisi in frequenza prima di interpretarne i risultati.
- Prima di dichiarare che una configurazione è migliore di un'altra, confronta la
  **dispersione entro configurazione** con la **differenza tra configurazioni**. Se sono
  dello stesso ordine, la differenza non è dimostrata: servono più ripetizioni.
- `compareRuns` deve quindi accettare una **lista di bag per configurazione**, non un bag
  singolo, e aggregare prima di confrontare.

### 1.2 Differenze rispetto al paper di riferimento

`Int_sys_Project.pdf` descriveva un'architettura **centralizzata**: UKF sulle distanze UWB +
modulo di trilaterazione separato + rilevatore MAD a monte. L'implementazione attuale è
**distribuita**: ogni robot ha un proprio filtro che stima direttamente `[x, y, θ]` dalle
distanze (3 anchor + 2 vicini) e dall'IMU, senza trilaterazione esplicita. Il MAD vive in un
nodo legacy separato, collegabile al path di stima con `uwb_source:=filtered`.

---

## 2. Caricamento e allineamento dei dati

Prima di qualsiasi analisi. Le insidie qui sotto sono tutte documentate in
`topics_reference.md` §6 e vanno recepite nel codice di caricamento.

| Insidia | Cosa fare |
|---|---|
| `p[9]` è **row-major** | `P = reshape(p, 3, 3)'` — la trasposizione è obbligatoria |
| `theta` è wrappato in [−π, π] | `unwrap()` prima di derivare o plottare |
| `pose_debug.dt` è la costante `0.02`, non il tempo reale | Ricavare i passi da `diff(header.stamp)`, **mai** da `dt` |
| `Twist`, `Pose`, `Float64MultiArray` **non hanno header** | Allinearli con il receive time del bag |
| `/odom` gira a **30 Hz**, `pose_debug` a 50 Hz | Interpolare `/odom` sui timestamp di `pose_debug` |
| `/uwb/filtered_*` **assenti** nei bag di default | Il codice deve tollerarne l'assenza, non andare in errore |
| `pose_estimate.covariance` ha solo 5 indici su 36 popolati | Usare `pose_debug.p[]`, che ha la 3×3 completa |
| `AnchorDist`: il **campo** è l'anchor, l'**indice** è il robot | `D(robot+1, anchor+1) = [a1'; a2'; a3']'` |
| `innovation[5]` è angle-wrapped, `[0..4]` no | Trattarli separatamente |
| Timer wall-clock con RTF ≠ 1 | Verificare il rate effettivo da `diff(header.stamp)`; non assumere 50 Hz |
| Tre copie quasi identiche di centroide/tracking error | Usarne **una sola** (robot 0), non mediarle |
| I topic `pose_debug*` di uno stesso ciclo condividono **lo stesso** `header.stamp` | Unirli con una join esatta sul timestamp, non con interpolazione |

### 2.1 Finestra di regime

Escludere il transitorio: la formazione parte con errore ≈0.71 m contro una soglia di 0.15 m,
e `P0 = 10·I` deve collassare. Definire la finestra come il primo istante in cui tutti e tre
i `fsm_state.phase` valgono 1, oppure `t > 50 s` come nel paper. **Usare la stessa
definizione per tutte le configurazioni confrontate.**

---

## 3. Tracking del centroide e della formazione

- **Traiettoria stimata vs desiderata**: plot 2D di `centroid_position` sovrapposto a
  `desired_trajectory`.
- **Errore di tracking nel tempo**: `tracking_error` (ex, ey), con media a regime.
- **Errore di configurazione**: `fsm_state.formation_error` nel tempo, con marcatori sulle
  transizioni FORMATION→TRACKING dei tre robot sovrapposte — mostra se il consenso AND
  distribuito le rende sincrone o se un robot aspetta gli altri.
  > Il fattore di blending `phase_blend_` **non è pubblicato**: l'istante di *inizio* della
  > transizione si deduce solo a ritroso dalla commutazione (1.5 s prima).
- **Analisi in frequenza**: FFT + PSD (Welch, overlap 50%, finestra Hamming) dell'errore di
  tracking a regime. Nel paper centralizzato emergeva un picco a ~0.15 Hz da mode switching;
  qui ci sono 3 controllori indipendenti, quindi va confrontato tra i tre robot.
- **Scatter con ellisse 3σ** dell'errore di tracking.

> **Verifica preliminare di saturazione**: se `R·ω` supera 0.22 m/s il centroide non può
> fisicamente inseguire il riferimento e l'errore residuo non è attribuibile alla stima.
> Controllare `cmd_vel.linear.x` contro 0.22 prima di interpretare qualsiasi errore di
> tracking.

---

## 4. Qualità della stima rispetto alle misure (`pose_debug`)

Canale per canale sui 6 canali (`anchor0..2`, `neighbor0..1`, `theta_imu`), per robot:

- **Overlay `z` vs `z_pred`** nel tempo.
- **Innovazione nel tempo** con banda teorica ±√(`r_diag`).
- **Consistenza statistica (NIS)**: `NIS_k = innovation_k' · S_k⁻¹ · innovation_k`.
  Approssimazione pratica `S_k ≈ diag(r_diag)`; confrontare la media con il valore atteso
  di una chi-quadro a 6 gradi di libertà. Colma il gap che il paper segnalava (assenza di
  NEES/NIS).
- **PSD dell'innovazione**: energia strutturata residua = mismatch di modello; rumore bianco
  = filtro ben calibrato.

**Filtri da applicare prima dell'analisi:**
- Scartare i campioni con `have_imu == false`: lì `z[5]` **è** la predizione, quindi
  `innovation[5]` vale esattamente 0 e il canale θ non porta informazione.
- `z_pred` significa cose diverse per EKF (`h(x_pred)`) e UKF (media dei sigma point): **non
  confrontarli ingenuamente tra filtri diversi**.
- Un canale UWB saturato appare come valore **congelato e stantio senza alcun flag** (i
  consumatori scartano il `-1.0` e tengono l'ultimo valore valido). Per rilevare la
  saturazione bisogna incrociare i topic `/uwb/*` grezzi.

---

## 5. Precisione della stima di posizione

Ground truth di riferimento: `/odom` (interpolato a 30→50 Hz).

- **Errore di posizione**: `(x,y)` da `pose_debug` meno `(x,y)` da `odom`, per robot.
- **Tabella RMSE** per asse e totale (`sqrt(RMSE_x² + RMSE_y²)`), per robot **e per
  configurazione**.
- **Scatter con ellisse 3σ dalla covarianza reale del filtro** (`Pxx=p[0]`, `Pxy=p[1]`,
  `Pyy=p[4]`) — confronto diretto covarianza-dichiarata vs errore-osservato, complementare
  al NIS.
- **Serie temporale con banda ±1σ** da `sqrt(p[0])` / `sqrt(p[4])`, istante per istante.
- **Istogrammi con fit gaussiano**.

> **Promemoria metodologico**: `/odom` non è ground truth assoluta — non esiste alcun topic
> di ground truth nel sistema (`libgazebo_ros_state.so` non è caricato). È inoltre la stessa
> fonte da cui l'emulatore genera le distanze UWB. L'errore qui misurato è quindi "errore
> rispetto al riferimento generativo", non errore assoluto nel mondo Gazebo.

---

## 6. Dall'incertezza dei sensori alla precisione di stima

Il collegamento esplicito rumore in ingresso → precisione in uscita.

- **Tabella rumore configurato vs errore osservato**: σ UWB (`sensor_params.yaml`), σ
  orientazione IMU e Q di processo (`pose_filter_params.yaml`) affiancati all'RMSE
  osservato, per configurazione.
- **`q_diag` / `r_diag` nel tempo**: nella configurazione attuale sono **costanti** per
  tutta la run — dichiararlo esplicitamente come baseline. Il termine di inflazione
  `R_ω·dt²` su `q_diag[2]` vale ≈1.6e-11, numericamente invisibile.
- **Curva di sensitività**: RMSE di posizione vs σ UWB configurato, dallo sweep della riga 4
  della matrice esperimenti. **È il plot centrale** che risponde alla domanda "come si
  traduce l'incertezza delle misure in precisione del movimento".
- **Contributo relativo IMU vs UWB**: ampiezza relativa delle componenti dell'innovazione —
  canali anchor (0–2), neighbor (3–4), theta_imu (5).
- **Effetto del modello di rumore**: gaussiano vs uniforme a **parità di varianza**
  (per l'uniforme `±a` la varianza equivalente è `a²/3`, non `a²`).

---

## 7. Confronto EKF vs UKF, e sweep su α

- **RMSE a confronto** tra `filter_type:=ekf` e `filter_type:=ukf`, media ± dev.std su N run.
- **Innovazioni / NIS** a confronto: l'UKF gestisce meglio la non-linearità della misura
  distanza→stato anche nell'architettura distribuita per-robot?
- **Tempo di convergenza**: quanto impiega ciascun filtro a portare `trace(P)` sotto una
  soglia dopo lo spawn.
- **Sweep su `ukf.alpha`** ∈ {1e-3, 0.1, 0.5, 1.0} come asse a sé.

> **Test di sanità obbligatorio**: con `ukf.alpha = 1e-3` lo *scaled UT* degenera
> esattamente nella linearizzazione dell'EKF. La run UKF con α=1e-3 **deve** risultare
> numericamente quasi indistinguibile dalla run EKF. Se non lo è, c'è un bug — e se lo è,
> conferma che il confronto EKF/UKF va fatto con α ≈ 1.

---

## 8. Fusione SDRE + Covariance Intersection

Nelle run con `gain_mode:=sdre_ci_experimental` sono disponibili topic aggiuntivi:

| Topic | Contenuto |
|---|---|
| `{ns}/pose_debug` | **La stima che è effettivamente sul path di controllo** (fusa se `ci_feedback:=true`, classica altrimenti) |
| `{ns}/pose_debug_riccati` | Stima A — one-step Riccati (`x_A`, `P_A`) |
| `{ns}/pose_debug_sdre` | Stima B — guadagno SDRE (`x_B`, `P_B`) |
| `{ns}/pose_debug_ci` | Stima fusa (`x_ci`, `P_ci`) |
| `{ns}/gain_mode_debug` | `Float64MultiArray`: `[w, dare_iterations, dare_ok, dare_residual]` |

Poiché `/pose_debug` resta "ciò che guida il robot", **tutti gli script delle Sezioni 4–7
funzionano invariati** anche sui bag SDRE-CI.

Analisi specifiche:
- **Confronto delle tre covarianze** nel tempo: `trace(P_A)`, `trace(P_B)`, `trace(P_ci)`.
- **RMSE delle tre stime** contro `/odom`: A, B e fusa, sugli stessi campioni.
- **Serie temporale di `w`** (da `gain_mode_debug[0]`): costante 0.5 in modalità `fixed`,
  variabile in `min_trace`.
- **Salute del solver DARE**: frazione di cicli con `dare_ok == 0` (fallback alla stima A),
  e distribuzione di `dare_iterations` — dopo il primo secondo il warm start deve portarla a
  poche unità.

**Due avvertenze di interpretazione:**

1. **La CI è conservativa per costruzione.** È pensata per fondere stime con correlazione
   incrociata ignota; qui i due rami condividono lo stesso prior *e* la stessa misura,
   quindi sono quasi perfettamente correlati. `P_ci` è garantita non ottimistica ma è
   strettamente più grande della vera covarianza d'errore, e in anello chiuso questa
   conservatività rientra nel `predict` successivo e si accumula. **Aspettati NIS/NEES
   "underconfident" ed ellissi 3σ gonfiate**: è corretto, non un bug.
2. **`min_trace` degenera con `sdre_cov_mode:=propagated`.** `K_A` è per definizione ottimo
   per quel prior, quindi `P_A ⪯ P_B` sempre e `trace(P_ci(w))` è monotona decrescente in
   `w`: il minimo cade a `w = 1`, cioè sull'EKF puro. Verificarlo empiricamente sulla serie
   di `w` è un risultato riportabile. Per un `min_trace` non banale serve
   `sdre_cov_mode:=steady_state`.

**Confronto pulito**: le run con `ci_feedback:=false` (shadow mode) confrontano i tre
stimatori sulla **stessa identica traiettoria e sullo stesso identico flusso di misure** —
è la risposta metodologicamente più solida alla domanda "quale stimatore è più accurato".
In anello chiuso, invece, ogni stimatore genera una traiettoria diversa e li si confronta su
problemi diversi.

---

## 9. MAD come asse causale

Con `enable_legacy_kf:=true uwb_source:=filtered` il MAD entra davvero nel path di stima.

- **Raw vs filtrato** nel tempo, per canale: `/uwb/{anchor,robot}_distances` contro
  `/uwb/filtered_{anchor,robot}_distances`.
- **Eventi di outlier**: istanti in cui il filtrato viene agganciato alla mediana. Contarli
  e sovrapporli alle serie di innovazione della Sezione 4.
- **Effetto sulla stima di stato**: RMSE e NIS con `uwb_source:=filtered` contro
  `uwb_source:=raw`, media ± dev.std su N run.
- **Isolamento dell'effetto MAD**: confrontare `enable_mad:=true` contro `enable_mad:=false`
  **entrambi con `uwb_source:=filtered`**. Confrontare direttamente con la baseline `raw`
  misurerebbe l'effetto combinato del KF scalare **e** del MAD.
- **Confronto delle due semantiche**: `mad_stage:=post_estimate` (attuale) contro
  `pre_measurement` (variante del paper).

**Avvertenze:**
- Con `distance_process_noise = 0.01` e `R = 1e-4` il KF scalare ha guadagno stazionario
  `K ≈ 0.99`, cioè è un **pass-through al ~99%**: se il MAD non scatta, `filtered` è
  indistinguibile da `raw`. Per rendere visibile l'asse va abbassato
  `distance_process_noise`.
- La soglia `3·MAD_grezzo` corrisponde a **≈2.02σ**, non a 3σ (manca il fattore di
  consistenza gaussiana 1.4826). Da dichiarare per non citare male la propria soglia; usare
  `mad_scale:=1.4826` per una vera regola a 3σ.
- Sotto `uwb_source:=filtered` la sentinella `-1.0` non arriva mai ai filtri (il KF legacy
  la assorbe): la guardia `d > 0.0` nei filtri diventa codice morto.

---

## 10. Effort di controllo

Completa l'analisi lasciata incompleta nel paper (Fig. 14).

- **Serie temporali di `cmd_vel`**: `linear.x` e `angular.z` per i tre robot.
- **PSD di ω**: verificare se persistono le oscillazioni ad alta frequenza attribuite
  nel paper all'assenza di damping derivativo nel controllo di yaw.
- **Correlazione con le transizioni di fase**: marcatori delle commutazioni
  `fsm_state.phase` sovrapposti al plot di ω.
- **Verifica di saturazione**: frazione di campioni con `linear.x` al limite di 0.22 m/s o
  `angular.z` a 2.84 rad/s.

> Attenzione a due dettagli del controllore: `linear.x` ha un **floor** a metà velocità
> (`max(0.5, cos(heading_error))`), quindi il robot non rallenta mai sotto quel valore
> nemmeno guardando lontano dalla direzione voluta; e il termine derivativo del PID sul
> centroide è **identicamente nullo** (`Kv_*` non ha alcun effetto), quindi il controllore è
> di fatto un PI più un PD separato sull'heading.

---

## 11. Prossimi passi

1. Generare i bag della matrice esperimenti (`simulation_guide.md` §8), 3 ripetizioni per
   configurazione, aumentando solo dove la varianza tra run lo richiede (§3.2 della guida).
2. Scrivere `loadRun.m` recependo tutte le insidie della Sezione 2 — è il pezzo da cui
   dipende la correttezza di tutto il resto.
3. Scrivere `analyzeRun.m` e `compareRuns.m` seguendo le Sezioni 3–10.
4. Validare i nomi di topic e campi contro `ros2 bag info` di un bag reale e contro
   `custom_messages/*.msg`.
