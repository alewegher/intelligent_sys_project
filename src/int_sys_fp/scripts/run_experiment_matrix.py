#!/usr/bin/env python3
"""Runs the experiment matrix from docs/simulation_guide.md §8 and names the bags
per the convention in §3.3: bags/<label>_r<rep>/, with one config file per bag
collected in bags/configs/<label>_r<rep>.txt (not scattered inside each bag dir).

Not a ROS node, not installed by CMakeLists - a standalone operator script. Run it
with the venv stripped from PATH, same as any other ros2 command in this workspace:

    env -u VIRTUAL_ENV PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.venv/bin' | paste -sd:)" \\
        python3 src/int_sys_fp/scripts/run_experiment_matrix.py

Safe to interrupt and rerun: each row is independent, existing bags are not touched
unless the SAME label+rep is produced again (which overwrites, matching a rerun's intent).
"""
import os, re, subprocess, time, shutil, csv, sys, signal

WS = "/home/alessandro/ros2_ws"
BAGS = os.path.join(WS, "bags")
CONFIGS_DIR = os.path.join(BAGS, "configs")   # config files collected here, not scattered
POSE_YAML = os.path.join(WS, "src/int_sys_fp/pose_filter_params.yaml")
SENSOR_YAML = os.path.join(WS, "src/int_sys_fp/sensor_params.yaml")
MANIFEST = os.path.join(BAGS, "matrix_manifest.csv")
REPORT = os.path.join(BAGS, "matrix_report.txt")
REPS = 3
SIM_DURATION_TIMEOUT = 260  # sim_duration(125.664s) + generous startup/shutdown margin

SIMPLE = [
    ("ekf_riccati_gauss",           {}),
    ("ukf_riccati_gauss",           {"filter_type": "ukf"}),
    ("noiseunif",                   {"noise_type": "2"}),
    ("sdreci_w0.5_gauss",           {"gain_mode": "sdre_ci_experimental", "ci_weight": "0.5"}),
    ("sdreci_shadow_gauss",         {"gain_mode": "sdre_ci_experimental", "ci_weight": "0.5", "ci_feedback": "false"}),
    ("sdreci_mintrace_steady_gauss",{"gain_mode": "sdre_ci_experimental", "ci_weight_mode": "min_trace", "sdre_cov_mode": "steady_state"}),
    ("madfiltered_gauss",           {"enable_legacy_kf": "true", "uwb_source": "filtered"}),
    ("madfiltered_nomad_gauss",     {"enable_legacy_kf": "true", "uwb_source": "filtered", "enable_mad": "false"}),
]

ALPHA_SWEEP = [1e-3, 0.1, 0.5, 1.0]
STDDEV_SWEEP = [0.005, 0.01, 0.05, 0.1]


def patch_yaml_line(path, pattern, new_line, count):
    with open(path) as f:
        text = f.read()
    text2, n_sub = re.subn(pattern, new_line, text)
    if n_sub != count:
        raise RuntimeError(f"{path}: expected {count} substitutions, got {n_sub}")
    with open(path, "w") as f:
        f.write(text2)


def set_ukf_alpha(value):
    patch_yaml_line(POSE_YAML, r"alpha: [0-9.eE+-]+", f"alpha: {value}", 1)


def set_sensor_stddev(value):
    patch_yaml_line(SENSOR_YAML, r"stddev: [0-9.eE+-]+", f"stddev: {value}", 6)


def kill_stragglers():
    pats = ["gzserver", "gzclient", "pose_ekf_node", "pose_ukf_node", "regulator_node",
            "uwb_emulator", "trajectory_planner", "distance_kf_node", "run_timer.py",
            "ros2 launch int_sys_fp"]
    any_killed = False
    for p in pats:
        r = subprocess.run(["pgrep", "-f", p], capture_output=True, text=True)
        if r.stdout.strip():
            any_killed = True
            subprocess.run(["pkill", "-9", "-f", p])
    if any_killed:
        time.sleep(3)
    return any_killed


def list_bag_dirs():
    if not os.path.isdir(BAGS):
        return set()
    return {d for d in os.listdir(BAGS) if d.startswith("int_sys_sim_")
            and os.path.isdir(os.path.join(BAGS, d))}


def run_one(label, args, rep, manifest_writer):
    ros2_cmd = ["ros2", "launch", "int_sys_fp", "synchronized_system_with_bag.launch.py",
                "enable_plotjuggler:=false"] + [f"{k}:={v}" for k, v in args.items()]
    cmd_str = " ".join(ros2_cmd)
    shell_cmd = (f"source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && "
                 + cmd_str)
    before = list_bag_dirs()
    t0 = time.time()
    status = "ok"
    err = ""
    try:
        proc = subprocess.Popen(["bash", "-lc", shell_cmd], cwd=WS,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 preexec_fn=os.setsid, text=True)
        try:
            out, _ = proc.communicate(timeout=SIM_DURATION_TIMEOUT)
        except subprocess.TimeoutExpired:
            status = "TIMEOUT"
            err = "process did not exit within timeout, force-killed"
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=10)
    except Exception as e:
        status = "EXCEPTION"
        err = str(e)

    wall = time.time() - t0
    stray = kill_stragglers()
    if stray and status == "ok":
        status = "ok_had_strays"

    after = list_bag_dirs()
    new_dirs = list(after - before)
    dest_name = f"{label}_r{rep}"
    dest = os.path.join(BAGS, dest_name)
    bag_duration = ""
    if len(new_dirs) == 1:
        src = os.path.join(BAGS, new_dirs[0])
        if os.path.exists(dest):
            shutil.rmtree(dest)
        shutil.move(src, dest)

        os.makedirs(CONFIGS_DIR, exist_ok=True)
        with open(os.path.join(CONFIGS_DIR, f"{dest_name}.txt"), "w") as f:
            f.write(f"label: {label}\nrep: {rep}\ncommand: {cmd_str}\n"
                    f"wall_seconds: {wall:.1f}\nstatus: {status}\n")

        info = subprocess.run(["bash", "-lc",
            f"source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && "
            f"ros2 bag info '{dest}'"], capture_output=True, text=True)
        m = re.search(r"Duration:\s*([0-9.]+)s", info.stdout)
        bag_duration = m.group(1) if m else "?"
    elif len(new_dirs) == 0:
        status = "NO_BAG_PRODUCED"
    else:
        status = f"AMBIGUOUS({len(new_dirs)}_new_dirs)"

    manifest_writer.writerow([label, rep, cmd_str, status, f"{wall:.1f}", bag_duration, err])
    print(f"[{time.strftime('%H:%M:%S')}] {label} r{rep}: status={status} wall={wall:.1f}s "
          f"bag_duration={bag_duration}s", flush=True)
    return status


def main():
    os.makedirs(BAGS, exist_ok=True)
    kill_stragglers()

    skipped = []
    with open(MANIFEST, "a", newline="") as mf:
        w = csv.writer(mf)
        if mf.tell() == 0:
            w.writerow(["label", "rep", "command", "status", "wall_s", "bag_duration_s", "error"])

        for label, args in SIMPLE:
            for rep in range(1, REPS + 1):
                run_one(label, args, rep, w)
                mf.flush()

        for alpha in ALPHA_SWEEP:
            set_ukf_alpha(alpha)
            time.sleep(1)
            label = f"ukf_alpha{alpha}_gauss"
            for rep in range(1, REPS + 1):
                run_one(label, {"filter_type": "ukf"}, rep, w)
                mf.flush()
        set_ukf_alpha(1.0)  # restore default

        for sd in STDDEV_SWEEP:
            set_sensor_stddev(sd)
            time.sleep(1)
            label = f"noisestd{sd}_gauss"
            for rep in range(1, REPS + 1):
                run_one(label, {}, rep, w)
                mf.flush()
        set_sensor_stddev(0.01)  # restore default

        skipped.append(("row_11_mad_stage_pre_measurement",
                         "mad_stage parameter was documented as a planned extension in "
                         "KF.cpp/simulation_guide.md but was never implemented - only "
                         "post_estimate semantics exist. Skipped rather than silently "
                         "running the wrong thing."))

    with open(REPORT, "a") as rf:
        rf.write(f"Matrix run finished at {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        rf.write("Skipped:\n")
        for name, reason in skipped:
            rf.write(f"  - {name}: {reason}\n")
        rf.write(f"\nSee {MANIFEST} for the full per-run log.\n\n")
    print("MATRIX DONE. Report at", REPORT, flush=True)


if __name__ == "__main__":
    main()
