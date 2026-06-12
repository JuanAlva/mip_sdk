import obd
import time

def escanear_red():
    for p in obd.scan_serial():
        con = obd.OBD(portstr=p, timeout=5, check_voltage=False)
        if con.is_connected():
            print({"port": p, "baudrate": con.interface._ELM327__port.baudrate, "protocolo": con.protocol_name()})
            return con
        print(f"{p} | sin respuesta")
    return None

def conectar(port, baudrate):
    conexion = obd.OBD(portstr=port, baudrate=baudrate, timeout=5, check_voltage=False)
    if not conexion.is_connected():
        raise ConnectionError("no conectado")
    return conexion

def leer_pid(conexion, pid: str):
    try:
        comando = getattr(obd.commands, pid)
    except AttributeError:
        return f"PID '{pid}' no existe"
    respuesta = conexion.query(comando, force=True)
    if respuesta.is_null():
        return "sin datos"
    return str(respuesta.value)


# con = escanear_red()
con = conectar("COM6", 115200)
PIDS = ["RPM", "SPEED", "COOLANT_TEMP", "INTAKE_TEMP", "THROTTLE_POS",
        "ENGINE_LOAD", "FUEL_LEVEL", "MAF", "INTAKE_PRESSURE",
        "BAROMETRIC_PRESSURE", "OIL_TEMP", "FUEL_RATE"]

def leer_velocidad_mps(conexion):
    respuesta = conexion.query(obd.commands.SPEED, force=True)

    if respuesta.is_null():
        return None

    velocidad_mps = respuesta.value.to("m/s").magnitude
    return velocidad_mps

try:
    while True:
        t_ms = int(time.time() * 1000)
        v_mps = leer_velocidad_mps(con)

        if v_mps is None:
            print(t_ms, "sin datos")
        else:
            print(t_ms, v_mps)

except KeyboardInterrupt:
    print("Lectura detenida")