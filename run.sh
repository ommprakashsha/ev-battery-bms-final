#!/bin/bash
# =============================================================================
# EV Battery Management System - Build & Test Script
# =============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

build_kernel() {
    echo "[*] Building kernel module..."
    cd kernel
    make clean 2>/dev/null || true
    make
    cd ..
    echo "[+] Kernel module built: kernel/bms_driver.ko"
}

build_userspace() {
    echo "[*] Building userspace components..."
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null
    make -j"$(nproc)"
    cd ..
    echo "[+] Userspace binaries built: build/bms_daemon, build/bms_dashboard, build/bms_ctl"
}

load_module() {
    if lsmod | grep -q bms_driver; then
        echo "[!] Unloading existing module..."
        sudo rmmod bms_driver 2>/dev/null || true
        sleep 0.5
    fi
    echo "[*] Loading bms_driver.ko..."
    sudo insmod kernel/bms_driver.ko
    sleep 0.5

    # Ensure device node permissions
    if [ -e /dev/bms_drv ]; then
        sudo chmod 666 /dev/bms_drv
    fi

    echo "[+] Module loaded successfully."
}

unload_module() {
    if lsmod | grep -q bms_driver; then
        echo "[*] Removing bms_driver..."
        sudo rmmod bms_driver
        echo "[+] Module unloaded."
    else
        echo "[-] Module not currently loaded."
    fi
}

run_system() {
    echo "[*] Starting BMS daemon in background..."
    sudo mkdir -p /var/log/bms
    sudo ./build/bms_daemon &
    DAEMON_PID=$!
    sleep 1

    echo "[*] Starting live dashboard..."
    ./build/bms_dashboard

    echo "[*] Stopping daemon (PID $DAEMON_PID)..."
    sudo kill "$DAEMON_PID" 2>/dev/null || true
    wait "$DAEMON_PID" 2>/dev/null || true
    echo "[+] Done."
}

case "${1:-build}" in
    build)
        build_kernel
        build_userspace
        ;;
    load)
        build_kernel
        load_module
        ;;
    unload)
        unload_module
        ;;
    run)
        build_kernel
        build_userspace
        load_module
        run_system
        ;;
    clean)
        echo "[*] Cleaning build artifacts..."
        cd kernel && make clean 2>/dev/null || true; cd ..
        rm -rf build
        echo "[+] Clean complete."
        ;;
    status)
        if [ -f ./build/bms_ctl ]; then
            ./build/bms_ctl status
        elif [ -f /proc/bms_status ]; then
            cat /proc/bms_status
        else
            echo "[-] Kernel driver not loaded or /dev/bms_drv not active."
        fi
        ;;
    inject_ov)
        echo "[*] Injecting overvoltage fault on Cell 0 (4.25V)..."
        ./build/bms_ctl inject 0 0 4.25
        ;;
    inject_ot)
        echo "[*] Injecting overtemperature fault on Cell 2 (68.0 C)..."
        ./build/bms_ctl inject 2 2 68.0
        ;;
    inject_uv)
        echo "[*] Injecting undervoltage fault on Cell 3 (2.40V)..."
        ./build/bms_ctl inject 3 1 2.40
        ;;
    clear)
        echo "[*] Clearing active faults..."
        ./build/bms_ctl clear
        ;;
    *)
        echo "Usage: $0 {build|load|unload|run|clean|status|inject_ov|inject_ot|inject_uv|clear}"
        exit 1
        ;;
esac
