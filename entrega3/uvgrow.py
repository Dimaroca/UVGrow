#Archivo principal utilizado por la raspberry Pi

#!/usr/bin/env python3
"""UVGrow Raspberry coordinator: MQTT, RFID, alerts, safe control and daily HTTP photos."""
import argparse
import json
import logging
import os
import signal
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from urllib.request import Request, urlopen

import paho.mqtt.client as mqtt

ROOT = Path(__file__).resolve().parent
TOPIC_TELEMETRY = 'uvgrow/nodo01/telemetria'
TOPIC_EVENTS = 'uvgrow/nodo01/eventos'
TOPIC_COMMANDS = 'uvgrow/nodo01/comandos'
LOG = logging.getLogger('UVGrow')


def load_config(path):
    with open(path, encoding='utf-8') as f:
        cfg = json.load(f)
    if cfg['photo_interval_seconds'] < 60:
        raise ValueError('photo_interval_seconds debe ser >= 60')
    if not (1 <= cfg['pump_pulse_ms'] <= 2000 and 1 <= cfg['fan_pulse_ms'] <= 2000):
        raise ValueError('Los pulsos no deben superar 2000 ms')
    return cfg


def uid_normalize(value):
    return ''.join(ch for ch in value.upper() if ch in '0123456789ABCDEF')


def take_photo(cfg):
    folder = ROOT / cfg['photo_directory']
    folder.mkdir(parents=True, exist_ok=True)
    request = Request(cfg['camera_url'], headers={'User-Agent': 'UVGrow/1.0', 'Cache-Control': 'no-cache'})
    with urlopen(request, timeout=15) as response:
        content_type = response.headers.get('Content-Type', '')
        data = response.read(3_000_001)
    if len(data) > 3_000_000 or len(data) < 100 or not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        raise ValueError('Respuesta de camara no es JPEG valido')
    if 'image/jpeg' not in content_type.lower():
        raise ValueError(f'Tipo HTTP inesperado: {content_type}')
    name = 'uvgrow_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jpg'
    destination = folder / name
    destination.write_bytes(data)
    LOG.info('FOTOGRAFIA guardada: %s (%d bytes)', destination, len(data))
    return destination


class Controller:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.connected = False
        self.last_telemetry = 0.0
        self.telemetry = {}
        self.profile = None
        self.pending_profile = None
        self.last_rfid = {}
        self.last_request = {'pump': 0.0, 'fan': 0.0}
        self.confirmations = {'pump': 0, 'fan': 0}
        self.awaiting = {'pump': False, 'fan': False}
        self.actuator_on = {'pump': False, 'fan': False}
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='uvgrow_raspberry_' + uuid.uuid4().hex[:8])
        if cfg.get('mqtt_username'):
            password = os.environ.get(cfg.get('mqtt_password_env', ''), '')
            self.client.username_pw_set(cfg['mqtt_username'], password)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, reason_code, properties):
        with self.lock:
            self.connected = not reason_code.is_failure
            self.pending_profile = None
            self.awaiting = {'pump': False, 'fan': False}
            self.confirmations = {'pump': 0, 'fan': 0}
            self.last_telemetry = 0.0
        if reason_code.is_failure:
            LOG.error('MQTT rechazado: %s', reason_code)
            return
        client.subscribe([(TOPIC_TELEMETRY, 1), (TOPIC_EVENTS, 1)])
        LOG.info('MQTT conectado; suscrito a telemetria y eventos')
        LOG.info('Esperando confirmacion de perfil desde telemetria; no se activaran actuadores hasta entonces')

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        with self.lock:
            self.connected = False
            self.pending_profile = None
            self.awaiting = {'pump': False, 'fan': False}
            self.confirmations = {'pump': 0, 'fan': 0}
        LOG.warning('MQTT desconectado: %s', reason_code)

    def publish_command(self, command, **fields):
        with self.lock:
            if not self.connected:
                LOG.warning('No se envia %s: MQTT desconectado', command)
                return False
        payload = {'command': command, 'request_id': uuid.uuid4().hex[:12], **fields}
        result = self.client.publish(TOPIC_COMMANDS, json.dumps(payload, separators=(',', ':')), qos=1, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            LOG.error('No se pudo encolar comando %s: %s', command, result.rc)
            return False
        LOG.info('COMANDO ENVIADO %s %s', command, fields)
        return True

    def on_message(self, client, userdata, message):
        try:
            obj = json.loads(message.payload.decode('utf-8'))
            if not isinstance(obj, dict):
                raise ValueError('JSON no es objeto')
            if obj.get('device_id') != 'uvgrow_nodo01':
                LOG.warning('Mensaje ignorado: device_id no coincide')
                return
            if message.topic == TOPIC_TELEMETRY:
                self.handle_telemetry(obj)
            elif message.topic == TOPIC_EVENTS:
                self.handle_event(obj)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            LOG.warning('Mensaje descartado en %s: %s', message.topic, exc)
        except Exception:
            LOG.exception('Error procesando mensaje MQTT')

    def handle_telemetry(self, obj):
        now = time.monotonic()
        with self.lock:
            self.telemetry = obj
            self.last_telemetry = now
            reported = obj.get('profile_id')
            if reported in self.cfg['profiles']:
                if self.profile != reported:
                    LOG.info('Perfil confirmado por ESP32: %s', reported)
                self.profile = reported
                if self.pending_profile == reported:
                    self.pending_profile = None
            else:
                self.profile = None
            self.actuator_on['pump'] = obj.get('pump_on') is True
            self.actuator_on['fan'] = obj.get('fan_on') is True
        LOG.info('TELEMETRIA perfil=%s suelo=%s%% temp=%sC agua_adc=%s bloqueo=%s luz=%s%% alertas=%s',
                 obj.get('profile_id'), obj.get('soil_pct'), obj.get('temperatura_ambiente_c'),
                 obj.get('water_adc'), obj.get('water_pump_lock'), obj.get('light_pct'), obj.get('active_alerts'))
        self.evaluate(obj, now)

    def handle_event(self, obj):
        event = obj.get('event', '')
        LOG.info('EVENTO %s: %s', event, json.dumps(obj, ensure_ascii=False))
        if event == 'rfid_detectado':
            uid = uid_normalize(str(obj.get('uid', '')))
            profile = self.cfg['rfid'].get(uid)
            if not profile:
                LOG.warning('RFID no registrado: %s', uid)
                return
            now = time.monotonic()
            with self.lock:
                if now - self.last_rfid.get(uid, -1e9) < self.cfg['rfid_debounce_seconds']:
                    return
                self.last_rfid[uid] = now
                self.pending_profile = profile
            LOG.info('RFID %s -> perfil %s', uid, profile)
            self.publish_command('set_profile', profile_id=profile)
        elif event == 'perfil_actualizado':
            LOG.info('ESP32 confirmo solicitud de perfil: %s', obj.get('reason'))
        elif event == 'comando_rechazado':
            LOG.warning('ESP32 rechazo comando: %s', obj.get('reason'))
            with self.lock:
                self.awaiting = {'pump': False, 'fan': False}
        elif event in ('bomba_encendida', 'bomba_apagada', 'ventilador_encendido', 'ventilador_apagado'):
            which = 'pump' if event.startswith('bomba') else 'fan'
            with self.lock:
                self.awaiting[which] = False
                self.actuator_on[which] = event.endswith('encendida') or event.endswith('encendido')
        elif event in ('alerta_activada', 'alerta_resuelta'):
            LOG.warning('ALERTA %s: %s valor=%s perfil=%s', event, obj.get('alert'), obj.get('value'), obj.get('profile_id'))

    def evaluate(self, obj, now):
        with self.lock:
            profile = self.profile
            connected = self.connected
            pending = self.pending_profile
            active = dict(self.actuator_on)
        if not connected or pending or profile not in self.cfg['profiles']:
            LOG.info('CONTROL: sin perfil confirmado o MQTT no disponible')
            return
        p = self.cfg['profiles'][profile]
        soil = obj.get('soil_pct')
        temp = obj.get('temperatura_ambiente_c')
        water = obj.get('water_adc')
        water_lock = obj.get('water_pump_lock')
        pump_possible = (isinstance(soil, (int, float)) and not isinstance(soil, bool)
                         and isinstance(water, (int, float)) and not isinstance(water, bool)
                         and water >= 800 and water_lock is False and soil <= p['soil_max']
                         and obj.get('pump_on') is False and obj.get('fan_on') is False)
        pump_needed = pump_possible and soil < p['soil_min']
        fan_needed = (isinstance(temp, (int, float)) and not isinstance(temp, bool)
                      and temp > p['temp_max'] and obj.get('pump_on') is False and obj.get('fan_on') is False)
        for name, needed in [('pump', pump_needed), ('fan', fan_needed)]:
            with self.lock:
                self.confirmations[name] = self.confirmations[name] + 1 if needed else 0
                count = self.confirmations[name]
                elapsed = now - self.last_request[name]
                busy = self.awaiting[name] or active['pump'] or active['fan']
            if count < self.cfg['decision_confirmations'] or busy:
                continue
            cooldown = self.cfg['pump_cooldown_seconds'] if name == 'pump' else self.cfg['fan_cooldown_seconds']
            if elapsed < cooldown:
                continue
            if not self.cfg['automatic_control']:
                LOG.info('SIMULACION: se solicitara %s por %d ms si se habilita control automatico',
                         name, self.cfg[name + '_pulse_ms'])
                with self.lock:
                    self.last_request[name] = now
                    self.confirmations[name] = 0
                continue
            command = 'pump_pulse' if name == 'pump' else 'fan_pulse'
            if self.publish_command(command, duration_ms=self.cfg[name + '_pulse_ms']):
                with self.lock:
                    self.last_request[name] = now
                    self.awaiting[name] = True
                    self.confirmations[name] = 0

    def camera_loop(self):
        interval = self.cfg['photo_interval_seconds']
        # First automatic photo after one complete interval unless explicitly configured otherwise.
        next_photo = time.monotonic() if self.cfg.get('photo_on_start') else time.monotonic() + interval
        while not self.stop.is_set():
            wait = max(0, next_photo - time.monotonic())
            if self.stop.wait(wait):
                break
            try:
                take_photo(self.cfg)
            except Exception as exc:
                LOG.error('Fallo fotografia: %s', exc)
            next_photo = time.monotonic() + interval

    def watchdog_loop(self):
        while not self.stop.wait(10):
            with self.lock:
                age = time.monotonic() - self.last_telemetry if self.last_telemetry else None
            if age is not None and age > self.cfg['telemetry_timeout_seconds']:
                LOG.warning('Sin telemetria nueva desde hace %.0f segundos', age)

    def run(self):
        LOG.info('Control automatico: %s', 'ACTIVADO' if self.cfg['automatic_control'] else 'SIMULACION')
        LOG.info('Fotografias cada %s segundos (%s)', self.cfg['photo_interval_seconds'], self.cfg['camera_url'])
        self.client.connect_async(self.cfg['mqtt_host'], self.cfg['mqtt_port'], keepalive=60)
        self.client.loop_start()
        threading.Thread(target=self.camera_loop, daemon=True).start()
        threading.Thread(target=self.watchdog_loop, daemon=True).start()
        self.stop.wait()
        self.client.disconnect()
        self.client.loop_stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(ROOT / 'config_uvgrow.json'))
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s', datefmt='%H:%M:%S')
    cfg = load_config(args.config)
    app = Controller(cfg)
    signal.signal(signal.SIGINT, lambda *_: app.stop.set())
    signal.signal(signal.SIGTERM, lambda *_: app.stop.set())
    try:
        app.run()
    except KeyboardInterrupt:
        app.stop.set()


if __name__ == '__main__':
    main()
