#!/usr/bin/env python3
"""
Orchestrator for 7_series_threading_example.

Monitors two GPIO buttons:
  - START button (GPIO_BTN_START): launches the compiled C executable.
  - STOP  button (GPIO_BTN_STOP):  sends SIGTERM to stop it cleanly.

Wiring (BCM numbering, active-low with internal pull-up):
  GPIO 17  →  START button  →  GND
  GPIO 27  →  STOP  button  →  GND

Usage:
  python3 orchestrator.py [--executable PATH] [--start-pin N] [--stop-pin N]

The executable is expected to handle SIGTERM for a clean shutdown.
"""

import argparse
import datetime
import logging
import os
import queue
import signal
import socket
import struct
import subprocess
import sys
import time
import threading

import RPi.GPIO as GPIO

# ---------------------------------------------------------------------------
# Configuration defaults (override with CLI args)
# ---------------------------------------------------------------------------
DEFAULT_EXECUTABLE  = os.path.join(os.path.dirname(__file__), "7_series_threading_example")
DEFAULT_BTN_START   = 17   # BCM pin number
DEFAULT_BTN_STOP    = 27   # BCM pin number
DEBOUNCE_MS         = 300  # milliseconds — ignore bounces shorter than this
SIGTERM_TIMEOUT_S   = 5    # seconds to wait for clean exit before SIGKILL

# Data socket — must match SOCKET_HOST / SOCKET_PORT in the C program
SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 9000

# Binary wire format for each MIP data message (little-endian, 27 bytes):
#   B  : msg_type  (0x01=accel, 0x02=quat)
#   Q  : device_ns (uint64, nanoseconds from device reference time)
#   4f : values    (4 × float32 — accel:[x,y,z,0] or quat:[w,x,y,z])
#   H  : flags     (uint16, valid_flags for quat / 0 for accel)
MSG_TYPE_ACCEL = 0x01
MSG_TYPE_QUAT  = 0x02
_MSG_FMT  = "<BQ4fH"
_MSG_SIZE = struct.calcsize(_MSG_FMT)  # 27 bytes

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  [%(levelname)-7s]  %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("orchestrator")

# ---------------------------------------------------------------------------
# Global process handle
# ---------------------------------------------------------------------------
_process: subprocess.Popen | None = None

# Latest decoded sensor data — written by the socket receiver thread, read by anyone
_data_lock                  = threading.Lock()
_latest_accel: tuple | None = None  # (ns, x, y, z)  [g]
_latest_quat:  tuple | None = None  # (ns, w, x, y, z, flags)

# Async write queue — socket receiver puts CSV rows here; _file_writer_thread drains it.
# maxsize=10_000 ≈ 50 s of data at 200 msg/s; put_nowait drops rather than blocks on overflow.
_write_queue: queue.Queue = queue.Queue(maxsize=10_000)


def start_device(executable: str) -> None:
    global _process

    if _process is not None and _process.poll() is None:
        log.warning("El proceso ya está en ejecución (PID %d) — ignorando START", _process.pid)
        return

    if not os.path.isfile(executable):
        log.error("Ejecutable no encontrado: %s", executable)
        return

    log.info("Iniciando: %s", executable)
    _process = subprocess.Popen(
        [executable],
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    log.info("Proceso iniciado con PID %d", _process.pid)


def stop_device() -> None:
    global _process

    if _process is None or _process.poll() is not None:
        log.warning("No hay proceso activo para detener")
        _process = None
        return

    log.info("Enviando SIGTERM al proceso PID %d ...", _process.pid)
    _process.send_signal(signal.SIGTERM)

    try:
        _process.wait(timeout=SIGTERM_TIMEOUT_S)
        log.info("Proceso terminado correctamente (código %d)", _process.returncode)
    except subprocess.TimeoutExpired:
        log.warning("Timeout de %ds — forzando terminación con SIGKILL", SIGTERM_TIMEOUT_S)
        _process.kill()
        _process.wait()
        log.info("Proceso eliminado con SIGKILL")

    _process = None


# ---------------------------------------------------------------------------
# OBD-II velocity (placeholder — replace with python-obd implementation)
# ---------------------------------------------------------------------------
def read_obd_velocity() -> float:
    """
    Returns vehicle speed [m/s] from OBD-II.

    TODO: implement with python-obd:
        import obd
        conn = obd.OBD()
        r = conn.query(obd.commands.SPEED)
        return float(r.value.to("m/s").magnitude)
    """
    return 0.0


# ---------------------------------------------------------------------------
# Orientation + velocity integration
# ---------------------------------------------------------------------------
def integrate_orientation_velocity(
    quat: tuple[float, float, float, float],
    velocity_ms: float,
    dt_s: float,
) -> tuple[float, float, float]:
    """
    Projects body-frame forward velocity into the world frame via the attitude
    quaternion, then multiplies by dt to obtain an instantaneous displacement.

    quat        : (w, x, y, z) unit quaternion — body → world rotation
    velocity_ms : scalar speed [m/s] from OBD
    dt_s        : integration step [s]

    Returns: (dx, dy, dz) displacement in world frame [m]

    Assumes the vehicle's forward axis is +X in the body frame.
    TODO: accumulate total position; choose NED vs ENU convention.
    """
    if dt_s <= 0.0:
        return (0.0, 0.0, 0.0)

    w, x, y, z = quat
    # First column of the rotation matrix R(q): rotates unit +X body vector to world
    fwd_x = 1.0 - 2.0 * (y * y + z * z)
    fwd_y =       2.0 * (x * y + w * z)
    fwd_z =       2.0 * (x * z - w * y)

    dist = velocity_ms * dt_s
    return (fwd_x * dist, fwd_y * dist, fwd_z * dist)


# ---------------------------------------------------------------------------
# Asynchronous file writer
# ---------------------------------------------------------------------------
def _file_writer_thread(filepath: str) -> None:
    """
    Drains _write_queue and writes each row to a CSV file.

    Runs in a non-daemon thread so Python waits for it to finish on exit.
    Stops when it receives the None sentinel (put by _shutdown).

    CSV columns:
        timestamp_ns : device reference time [nanoseconds]
        type         : "accel" or "quat"
        f0..f3       : accel→[x,y,z,0]  |  quat→[w,x,y,z]
        flags        : valid_flags (quat) or 0 (accel)
    """
    log.info("Escritor de datos iniciado → %s", filepath)
    written = 0
    try:
        with open(filepath, "w", buffering=1, encoding="utf-8") as f:
            f.write("timestamp_ns,type,f0,f1,f2,f3,flags\n")
            while True:
                row = _write_queue.get()
                if row is None:          # sentinel — flush and stop
                    break
                f.write(row)
                written += 1
    except OSError as exc:
        log.error("Error de escritura en %s: %s", filepath, exc)
    finally:
        log.info("Escritor detenido — %d filas guardadas en %s", written, filepath)


# ---------------------------------------------------------------------------
# Socket server — receives MIP data from the C program
# ---------------------------------------------------------------------------
def _handle_client(conn: socket.socket, addr: tuple) -> None:
    """Receives and processes binary MIP data messages from the C program."""
    log.info("Cliente C conectado desde %s:%d", *addr)

    global _latest_accel, _latest_quat
    prev_quat_ns: int = 0
    buf = b""

    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk

            while len(buf) >= _MSG_SIZE:
                raw, buf = buf[:_MSG_SIZE], buf[_MSG_SIZE:]
                msg_type, ns, v0, v1, v2, v3, flags = struct.unpack(_MSG_FMT, raw)

                with _data_lock:
                    if msg_type == MSG_TYPE_ACCEL:
                        _latest_accel = (ns, v0, v1, v2)
                        log.debug(
                            "Accel  t=%d ns  [%7.4f, %7.4f, %7.4f] g",
                            ns, v0, v1, v2,
                        )
                        try:
                            _write_queue.put_nowait(
                                f"{ns},accel,{v0:.8f},{v1:.8f},{v2:.8f},0.00000000,0\n"
                            )
                        except queue.Full:
                            log.warning("Cola de escritura llena — muestra accel descartada")

                    elif msg_type == MSG_TYPE_QUAT:
                        _latest_quat = (ns, v0, v1, v2, v3, flags)
                        log.debug(
                            "Quat   t=%d ns  [w=%7.4f x=%7.4f y=%7.4f z=%7.4f] flags=0x%04X",
                            ns, v0, v1, v2, v3, flags,
                        )
                        try:
                            _write_queue.put_nowait(
                                f"{ns},quat,{v0:.8f},{v1:.8f},{v2:.8f},{v3:.8f},{flags}\n"
                            )
                        except queue.Full:
                            log.warning("Cola de escritura llena — muestra quat descartada")

                        # Integrate OBD velocity with quaternion orientation
                        if prev_quat_ns > 0 and flags != 0:
                            dt_s  = (ns - prev_quat_ns) * 1e-9
                            speed = read_obd_velocity()
                            dx, dy, dz = integrate_orientation_velocity(
                                (v0, v1, v2, v3), speed, dt_s
                            )
                            log.info(
                                "Δpose  dt=%.4fs  v=%.2f m/s  Δ=(%.5f, %.5f, %.5f) m",
                                dt_s, speed, dx, dy, dz,
                            )

                        prev_quat_ns = ns

    except (ConnectionResetError, BrokenPipeError, OSError):
        pass
    finally:
        conn.close()
        log.info("Cliente C desconectado")


def start_socket_server(host: str = SOCKET_HOST, port: int = SOCKET_PORT) -> None:
    """Starts the TCP server that accepts the C program's data connection."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    log.info("Socket servidor escuchando en %s:%d", host, port)

    while True:
        try:
            conn, addr = srv.accept()
            threading.Thread(
                target=_handle_client,
                args=(conn, addr),
                daemon=True,
            ).start()
        except OSError:
            break


# ---------------------------------------------------------------------------
# GPIO callbacks (called from a background thread by RPi.GPIO)
# ---------------------------------------------------------------------------
def _on_start_pressed(channel: int) -> None:
    log.info("Botón START presionado (GPIO %d)", channel)
    start_device(_executable_path)


def _on_stop_pressed(channel: int) -> None:
    log.info("Botón STOP presionado (GPIO %d)", channel)
    stop_device()


def setup_gpio(pin_start: int, pin_stop: int) -> None:
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for pin in (pin_start, pin_stop):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.add_event_detect(pin_start, GPIO.FALLING, callback=_on_start_pressed, bouncetime=DEBOUNCE_MS)
    GPIO.add_event_detect(pin_stop,  GPIO.FALLING, callback=_on_stop_pressed,  bouncetime=DEBOUNCE_MS)

    log.info("GPIO listo  —  START=GPIO%d  STOP=GPIO%d  (activo en bajo)", pin_start, pin_stop)


# ---------------------------------------------------------------------------
# Orchestrator shutdown (SIGINT / SIGTERM to this script)
# ---------------------------------------------------------------------------
def _shutdown(signum, frame) -> None:
    log.info("Señal %d recibida — cerrando orquestador...", signum)
    stop_device()
    GPIO.cleanup()
    _write_queue.put(None)  # sentinel: indica al escritor que cierre el archivo
    sys.exit(0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
_executable_path: str = DEFAULT_EXECUTABLE

def keyboard_simulator(pin_start: int, pin_stop: int) -> None:
    """
    Simula botones desde teclado:
      s -> START
      x -> STOP
      q -> salir
    """
    log.info("Simulador activo: presiona 's' START, 'x' STOP, 'q' salir")

    while True:
        key = input().strip().lower()

        if key == "s":
            start_device(_executable_path)

        elif key == "x":
            stop_device()

        elif key == "q":
            _shutdown(signal.SIGINT, None)

        else:
            log.info("Comando no reconocido. Usa: s=start, x=stop, q=salir")

def main() -> None:
    global _executable_path

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--executable", default=DEFAULT_EXECUTABLE, help="Ruta al binario compilado")
    parser.add_argument("--start-pin",  type=int, default=DEFAULT_BTN_START, help="Pin BCM del botón START")
    parser.add_argument("--stop-pin",   type=int, default=DEFAULT_BTN_STOP,  help="Pin BCM del botón STOP")
    parser.add_argument("--data-dir",   default=".",                         help="Directorio donde guardar los datos CSV")
    args = parser.parse_args()

    _executable_path = args.executable

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    os.makedirs(args.data_dir, exist_ok=True)
    data_file = os.path.join(
        args.data_dir,
        f"mip_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    log.info("Guardando datos en: %s", data_file)
    threading.Thread(
        target=_file_writer_thread,
        args=(data_file,),
        daemon=False,  # no-daemon: Python espera a que termine antes de salir
    ).start()

    threading.Thread(
        target=start_socket_server,
        daemon=True,
    ).start()

    setup_gpio(args.start_pin, args.stop_pin)

    threading.Thread(
        target=keyboard_simulator,
        args=(args.start_pin, args.stop_pin),
        daemon=True,
    ).start()

    log.info("Orquestador activo. Esperando botones... (Ctrl+C para salir)")

    try:
        while True:
            # Detectar si el proceso murió inesperadamente
            if _process is not None and _process.poll() is not None:
                log.warning(
                    "El proceso terminó inesperadamente (código %d)",
                    _process.returncode,
                )
                globals()["_process"] = None

            time.sleep(0.5)

    except KeyboardInterrupt:
        _shutdown(signal.SIGINT, None)


if __name__ == "__main__":
    main()
