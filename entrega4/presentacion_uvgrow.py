#Codigo final usado en la presentacion

#!/usr/bin/env python3
"""Demostracion local de UVGrow: MQTT real, RFID, actuadores y camara HTTP."""
import argparse
import json
import os
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from urllib.request import Request, urlopen

import paho.mqtt.client as mqtt

ROOT = Path(__file__).resolve().parent
TELEMETRY = 'uvgrow/nodo01/telemetria'
EVENTS = 'uvgrow/nodo01/eventos'
COMMANDS = 'uvgrow/nodo01/comandos'


def normalize_uid(value):
    return ''.join(c for c in str(value).upper() if c in '0123456789ABCDEF')


def photo(cfg):
    folder = ROOT / cfg['photo_directory']
    folder.mkdir(parents=True, exist_ok=True)
    request = Request(cfg['camera_url'], headers={'User-Agent': 'UVGrow-Presentacion/1.0', 'Cache-Control': 'no-cache'})
    with urlopen(request, timeout=15) as response:
        content_type = response.headers.get('Content-Type', '')
        data = response.read(3_000_001)
    if ('image/jpeg' not in content_type.lower() or len(data) < 100 or
            len(data) > 3_000_000 or not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9')):
        raise ValueError('La camara no devolvio un JPEG valido')
    path = folder / ('uvgrow_presentacion_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jpg')
    path.write_bytes(data)
    return path, len(data)


class Demo:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.print_lock = threading.Lock()
        self.stop = threading.Event()
        self.connected = False
        self.last_telemetry = None
        self.last_telemetry_time = 0.0
        self.last_uid = {}
        self.pending_profile = None
        self.last_pump_command = None
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='uvgrow_demo_' + uuid.uuid4().hex[:8])
        if cfg.get('mqtt_username'):
            self.client.username_pw_set(cfg['mqtt_username'], os.environ.get(cfg.get('mqtt_password_env', ''), ''))
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def show(self, text):
        with self.print_lock:
            print(text, flush=True)

    def on_connect(self, client, userdata, flags, reason_code, properties):
        with self.lock:
            self.connected = not reason_code.is_failure
        if reason_code.is_failure:
            self.show('[MQTT] Conexion rechazada: ' + str(reason_code))
            return
        client.subscribe([(TELEMETRY, 1), (EVENTS, 1)])
        self.show('[MQTT] Conectado a Mosquitto. Escuchando telemetria y eventos.')

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        with self.lock:
            self.connected = False
        self.show('[MQTT] Desconectado: ' + str(reason_code))

    def command(self, name, **fields):
        with self.lock:
            if not self.connected:
                self.show('[ERROR] MQTT no conectado; comando cancelado')
                return False
        payload = {'command': name, 'request_id': uuid.uuid4().hex[:12], **fields}
        info = self.client.publish(COMMANDS, json.dumps(payload, separators=(',', ':')), qos=1, retain=False)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.show('[ERROR] No se pudo publicar el comando')
            return False
        self.show('\n[COMANDO -> ESP32] ' + json.dumps(payload, ensure_ascii=False))
        self.show('[INFO] Solicitud enviada; esperar evento de confirmacion del ESP32.')
        return True

    def on_message(self, client, userdata, message):
        try:
            obj = json.loads(message.payload.decode('utf-8'))
            if not isinstance(obj, dict) or obj.get('device_id') != 'uvgrow_nodo01':
                return
            now = time.monotonic()
            if message.topic == TELEMETRY:
                with self.lock:
                    self.last_telemetry = obj
                    self.last_telemetry_time = now
                    if self.pending_profile == obj.get('profile_id'):
                        self.pending_profile = None
                self.show('\n' + '=' * 58 + '\n[' + datetime.now().strftime('%H:%M:%S') + '] ESP32 -> RASPBERRY | JSON TELEMETRIA\nTopic: ' + message.topic + '\n' + json.dumps(obj, indent=2, ensure_ascii=False) + '\n' + '=' * 58)
            elif message.topic == EVENTS:
                self.show('\n[' + datetime.now().strftime('%H:%M:%S') + '] ESP32 -> RASPBERRY | EVENTO\n' + json.dumps(obj, indent=2, ensure_ascii=False))
                if obj.get('event') == 'rfid_detectado':
                    self.handle_rfid(obj, now)
                elif obj.get('event') == 'comando_rechazado':
                    self.show('[AVISO] El ESP32 rechazo el comando: ' + str(obj.get('reason')))
                elif obj.get('event') == 'perfil_actualizado':
                    self.show('[RFID] El ESP32 acepto el perfil: ' + str(obj.get('reason')))
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            self.show('[AVISO] Mensaje descartado: ' + str(exc))
        except Exception as exc:
            self.show('[ERROR] Procesando MQTT: ' + repr(exc))

    def handle_rfid(self, obj, now):
        uid = normalize_uid(obj.get('uid', ''))
        profiles = {normalize_uid(key): value for key, value in self.cfg['rfid'].items()}
        profile = profiles.get(uid)
        if not profile:
            self.show('[RFID] UID no registrado: ' + uid)
            return
        with self.lock:
            if now - self.last_uid.get(uid, -1e9) < self.cfg.get('rfid_debounce_seconds', 3):
                return
            self.last_uid[uid] = now
            self.pending_profile = profile
        self.show('[RFID] ' + uid + ' -> ' + profile)
        if not self.command('set_profile', profile_id=profile):
            with self.lock:
                self.pending_profile = None

    def pulse(self, which):
        with self.lock:
            obj = self.last_telemetry
            age = time.monotonic() - self.last_telemetry_time
        if not obj or age > min(self.cfg.get('telemetry_timeout_seconds', 100), 15):
            self.show('[SEGURIDAD] No hay telemetria reciente; prueba cancelada')
            return
        if obj.get('pump_on') is not False or obj.get('fan_on') is not False:
            self.show('[SEGURIDAD] Hay un actuador encendido o su estado no es valido')
            return
        if which == 'pump':
            profile = obj.get('profile_id')
            p = self.cfg['profiles'].get(profile)
            water = obj.get('water_adc')
            soil = obj.get('soil_pct')
            if not p or not isinstance(water, (int, float)) or isinstance(water, bool) or water < 800 or obj.get('water_pump_lock') is not False:
                self.show('[SEGURIDAD] Bomba bloqueada: perfil o agua insuficiente')
                return
            if not isinstance(soil, (int, float)) or isinstance(soil, bool) or soil > p['soil_max']:
                self.show('[SEGURIDAD] Bomba bloqueada: lectura de suelo invalida o demasiado humeda')
                return
            if self.last_pump_command is not None and time.monotonic() - self.last_pump_command < 300:
                self.show('[SEGURIDAD] Espera 300 segundos desde la ultima prueba de bomba')
                return
            duration = min(int(self.cfg['pump_pulse_ms']), 2000)
            if self.command('pump_pulse', duration_ms=duration):
                self.last_pump_command = time.monotonic()
        else:
            duration = min(int(self.cfg['fan_pulse_ms']), 2000)
            self.command('fan_pulse', duration_ms=duration)

    def run(self):
        self.show('UVGROW | PRESENTACION LOCAL\nTelemetria real, RFID, eventos y pruebas manuales.\nGreengrass envia a AWS de forma independiente.')
        self.client.connect_async(self.cfg['mqtt_host'], int(self.cfg['mqtt_port']), keepalive=60)
        self.client.loop_start()
        try:
            while not self.stop.is_set():
                self.show('\nOPCIONES: [f] ventilador  [b] bomba  [c] fotografia  [a] apagar  [s] estado  [q] salir')
                try:
                    option = input('Opcion: ').strip().lower()
                except EOFError:
                    break
                if option == 'f':
                    self.pulse('fan')
                elif option == 'b':
                    self.pulse('pump')
                elif option == 'c':
                    self.show('[CAMARA] Solicitando fotografia HTTP...')
                    try:
                        path, size = photo(self.cfg)
                        self.show('[CAMARA] JPEG guardado: ' + str(path) + ' (' + str(size) + ' bytes)')
                    except Exception as exc:
                        self.show('[CAMARA] Error: ' + str(exc))
                elif option == 'a':
                    self.command('all_off')
                elif option == 's':
                    with self.lock:
                        obj = self.last_telemetry
                        age = time.monotonic() - self.last_telemetry_time
                    if obj:
                        self.show('[ESTADO] Ultima telemetria hace %.1f s\n%s' % (age, json.dumps(obj, indent=2, ensure_ascii=False)))
                    else:
                        self.show('[ESTADO] Esperando primera telemetria')
                elif option == 'q':
                    break
                else:
                    self.show('Opcion no reconocida')
        finally:
            self.stop.set()
            self.client.disconnect()
            self.client.loop_stop()
            self.show('[UVGROW] Presentacion detenida')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(ROOT / 'config_uvgrow.json'))
    args = parser.parse_args()
    with open(args.config, encoding='utf-8') as stream:
        cfg = json.load(stream)
    for key in ('mqtt_host', 'mqtt_port', 'rfid', 'profiles', 'pump_pulse_ms', 'fan_pulse_ms', 'camera_url', 'photo_directory'):
        if key not in cfg:
            raise ValueError('Falta configuracion: ' + key)
    if not 1 <= int(cfg['pump_pulse_ms']) <= 2000 or not 1 <= int(cfg['fan_pulse_ms']) <= 2000:
        raise ValueError('Los pulsos deben estar entre 1 y 2000 ms')
    demo = Demo(cfg)
    try:
        demo.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
