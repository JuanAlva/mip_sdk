import obd
import time

DEFAULT_PORT     = "/dev/ttyUSB1"
DEFAULT_BAUDRATE = 115200


def escanear_red():
    """Auto-detecta el primer adaptador OBD disponible en los puertos serie."""
    for p in obd.scan_serial():
        con = obd.OBD(portstr=p, timeout=5, check_voltage=False)
        if con.is_connected():
            print({"port": p, "protocolo": con.protocol_name()})
            return con
        print(f"{p} | sin respuesta")
    return None


def conectar(port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
    """Conexión directa a un puerto conocido."""
    conexion = obd.OBD(portstr=port, baudrate=baudrate, timeout=5, check_voltage=False)
    if not conexion.is_connected():
        raise ConnectionError(f"No se pudo conectar a {port} @ {baudrate} baud")
    return conexion


def leer_pid(conexion, pid: str):
    """Lee un PID por nombre (ej. 'SPEED', 'RPM'). Devuelve el valor coNmo string."""
    try:
        comando = getattr(obd.commands, pid)
    except AttributeError:
        return f"PID '{pid}' no existe"
    respuesta = conexion.query(comando, force=True)
    if respuesta.is_null():
        return "sin datos"
    return str(respuesta.value)


def leer_velocidad_mps(conexion):
    """Devuelve la velocidad del vehículo en m/s, o None si no hay datos."""
    respuesta = conexion.query(obd.commands.SPEED, force=True)
    if respuesta.is_null():
        return None
    return float(respuesta.value.to("m/s").magnitude)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Lector OBD-II — Linux")
    parser.add_argument("--port",     default=None,                help="Puerto serie (ej. /dev/ttyUSB0) — omitir para auto-scan")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="Baudrate del adaptador (default: 115200)")
    parser.add_argument("--pids",     nargs="+", default=["SPEED", "RPM"], help="PIDs adicionales a mostrar")
    args = parser.parse_args()

    print("Conectando al adaptador OBD...")
    try:
        con = conectar(args.port, args.baudrate) if args.port else escanear_red()
    except ConnectionError as e:
        print(f"Error: {e}")
        exit(1)

    if con is None:
        print("No se encontró ningún adaptador OBD. Verifica el cable y los permisos del puerto.")
        print("  sudo usermod -aG dialout $USER  # añadir usuario al grupo dialout")
        exit(1)

    print(f"Conectado | Protocolo: {con.protocol_name()}")
    print("Presiona Ctrl+C para salir.\n")

    PIDS = ["RPM", "SPEED", "COOLANT_TEMP", "INTAKE_TEMP", "THROTTLE_POS",
            "ENGINE_LOAD", "FUEL_LEVEL", "MAF", "INTAKE_PRESSURE",
            "BAROMETRIC_PRESSURE", "OIL_TEMP", "FUEL_RATE"]

    try:
        while True:
            t_ms  = int(time.time() * 1000)
            v_mps = leer_velocidad_mps(con)
            extra = {pid: leer_pid(con, pid) for pid in args.pids if pid != "SPEED"}

            if v_mps is not None:
                print(f"{t_ms} ms | velocidad: {v_mps:.3f} m/s ({v_mps * 3.6:.1f} km/h) | {extra}")
            else:
                print(f"{t_ms} ms | velocidad: sin datos | {extra}")

    except KeyboardInterrupt:
        print("\nLectura detenida")