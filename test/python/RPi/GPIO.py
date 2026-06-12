import time
import threading

BCM = "BCM"
IN = "IN"
PUD_UP = "PUD_UP"
FALLING = "FALLING"

_callbacks = {}
_bouncetime = {}
_last_event_time = {}
_mode = None
_warnings = True


def setmode(mode):
    global _mode
    _mode = mode
    print(f"[FAKE GPIO] setmode({mode})")


def setwarnings(flag):
    global _warnings
    _warnings = flag
    print(f"[FAKE GPIO] setwarnings({flag})")


def setup(pin, mode, pull_up_down=None):
    print(f"[FAKE GPIO] setup(pin={pin}, mode={mode}, pull_up_down={pull_up_down})")


def add_event_detect(pin, edge, callback=None, bouncetime=0):
    _callbacks[pin] = callback
    _bouncetime[pin] = bouncetime / 1000.0
    _last_event_time[pin] = 0

    print(
        f"[FAKE GPIO] add_event_detect(pin={pin}, edge={edge}, "
        f"callback={callback.__name__ if callback else None}, "
        f"bouncetime={bouncetime} ms)"
    )


def simulate_falling(pin):
    """
    Simula un flanco de bajada en el pin.
    Es equivalente a presionar un botón activo en bajo.
    """
    now = time.time()
    last = _last_event_time.get(pin, 0)
    debounce = _bouncetime.get(pin, 0)

    if now - last < debounce:
        print(f"[FAKE GPIO] Rebote ignorado en GPIO {pin}")
        return

    _last_event_time[pin] = now

    callback = _callbacks.get(pin)

    if callback is None:
        print(f"[FAKE GPIO] No hay callback registrado para GPIO {pin}")
        return

    print(f"[FAKE GPIO] Simulando FALLING en GPIO {pin}")
    callback(pin)


def cleanup():
    print("[FAKE GPIO] cleanup()")
    _callbacks.clear()