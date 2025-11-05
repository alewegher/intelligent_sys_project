# TurtleBot3 Multi-Robot con UWB Emulator - SISTEMA COMPLETO

## 🎉 SISTEMA INTEGRATO FUNZIONANTE

**3 TurtleBot3 + UWB Emulator + Nodi ROS per localizzazione e sensori**

## 🚀 Uso

### Sistema completo (TurtleBot3 + UWB + ROS):
```bash
./launch_turtlebot3_classic.sh complete
```

### Solo 3 TurtleBot3:
```bash
./launch_turtlebot3_classic.sh 3
```

### Lancio singolo TurtleBot3:
```bash
./launch_turtlebot3_classic.sh
```

### Controllo robot:
```bash
./control_robots.sh
```

## 📡 Topic disponibili

**Robot 1:** `/cmd_vel`, `/scan`, `/imu`, `/odom`, `/joint_states`
**Robot 2:** `/tb3_2/cmd_vel`, `/tb3_2/scan`, `/tb3_2/imu`, `/tb3_2/odom`  
**Robot 3:** `/tb3_3/cmd_vel`, `/tb3_3/scan`, `/tb3_3/imu`, `/tb3_3/odom`

## 🎮 Controlli rapidi

```bash
# Muovere tutti i robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
ros2 topic pub --once /tb3_2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
ros2 topic pub --once /tb3_3/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'

# Controllo con tastiera robot 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ✅ Cosa funziona

- **Differential drive completo** su tutti i robot
- **Sensori IMU, LiDAR, odometry** attivi
- **Plugin ROS 2 nativi** (non simulati)
- **Namespace separati** per controllo indipendente
- **Compatibile con SLAM, navigazione, ecc.**

## � File

- `launch_turtlebot3_classic.sh` - Launcher principale
- `final_3_robots.sh` - Sistema 3 robot  
- `control_robots.sh` - Controllo intelligente
- `README.md` - Questa documentazione

**Sistema pronto per sviluppo avanzato! 🚀**
