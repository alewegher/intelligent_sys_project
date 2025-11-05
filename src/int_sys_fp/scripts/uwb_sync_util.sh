#!/bin/bash

# Script di utilità per controllare e modificare la frequenza UWB

echo "=== UWB Sensor Synchronization Utility ==="
echo ""

# Funzione per mostrare la frequenza attuale
show_frequency() {
    echo "Current UWB sensor frequency:"
    ros2 param get /uwb_sensor_emulator uwb_sensor_frequency
    echo ""
}

# Funzione per impostare una nuova frequenza
set_frequency() {
    local new_freq=$1
    echo "Setting UWB sensor frequency to: ${new_freq} Hz"
    ros2 param set /uwb_sensor_emulator uwb_sensor_frequency ${new_freq}
    echo "Frequency updated!"
    echo ""
}

# Funzione per mostrare tutti i parameters del nodo UWB
show_all_params() {
    echo "All UWB emulator parameters:"
    ros2 param list /uwb_sensor_emulator
    echo ""
}

# Menu principale
case "$1" in
    "show")
        show_frequency
        ;;
    "set")
        if [ -z "$2" ]; then
            echo "Usage: $0 set <frequency_hz>"
            echo "Example: $0 set 100.0"
            exit 1
        fi
        set_frequency $2
        ;;
    "list")
        show_all_params
        ;;
    *)
        echo "Usage: $0 {show|set <freq>|list}"
        echo ""
        echo "Commands:"
        echo "  show        - Display current UWB sensor frequency"
        echo "  set <freq>  - Set new UWB sensor frequency (Hz)"
        echo "  list        - Show all UWB emulator parameters"
        echo ""
        echo "Examples:"
        echo "  $0 show"
        echo "  $0 set 100.0"
        echo "  $0 list"
        ;;
esac