# Guía de `main.py`

## Archivos del proyecto

> Resumen funcional de cada fichero/módulo


## `main.py`

- **Qué es:** Punto de entrada (CLI con Typer).
- **Hace:** Expone dos subcomandos:
  - `wifi`: monitoriza Wi-Fi (RSSI, latencia/jitter/pérdida hacia un `target`).
  - `camera`: monitoriza Wi-Fi asociado a la cámara + consume métricas ROS2 (`DiagnosticArray`).
- **Opciones comunes:** `interface`, `-i/--interval`, `-w/--capture-interface`, `-l/--log`, `-n/--name`.
- **Opciones específicas:**
  - `wifi`: `-t/--target` (host/IP de ping).
  - `camera`: `--diag-topic` (tópico ROS2, por defecto `/diagnostics`).
- **Utilidades destacadas:** selección/alineación de interfaces, logging opcional, arranque/parada de captura pcap (TShark), apertura de gráfica en tiempo real.


## `utils.py`

- **Qué es:** Utilidades generales.
- **Hace:** Construcción de rutas y nombres de ficheros (logs/pcaps/exports), helpers varios de cadena/tiempo.


## `theme.py`

- **Qué es:** Estilos y consola enriquecida.
- **Hace:** Centraliza la instancia de consola (colores/formatos) para mensajes de estado, warning, error, success.


## `system_utils.py`
- **Qué es:** Utilidades de sistema y dependencias.
- **Hace:**
  - Verificación de dependencias (p. ej., `iw`, `tshark`, `ping`, `matplotlib`).
  - Gestión de capturas con TShark (arranque/parada) y rutas de salida.


## `ui.py`

- **Qué es:** Interacción con el usuario.
- **Hace:**
  - Diálogos/selección de interfaces (`Managed`/`Monitor`).
  - Petición de `target` (para ping).
  - Confirmaciones y guardado de artefactos (imagen de la gráfica, pcap).


## `plotting.py`

- **Qué es:** Lógica de la **gráfica en tiempo real**.
- **Hace:** Capa de visualización (Matplotlib) para datos de Wi-Fi/ROS, refresco periódico, exportación a imagen.


## `ros_collector.py`

- **Qué es:** Conector a ROS 2.
- **Hace:** Proporciona clases/utilidades para suscribirse a `/diagnostics` y extraer métricas de cámara.
- **Clases mencionadas:** `CameraDiagCollector`.
- **Nota:** Solo requerido cuando se usa `main.py camera`.

## `config.py`

- **Qué es:** Configuración estática y metadatos para la app.
- **Hace:**
  - Define `APEventType` (eventos Wi-Fi con **mensaje** y **color** para UI).
  - Mapea BSSID → `{name, color}` en `APS` para mostrar APs con nombres/colores amigables.
  - Mapea nombres de **interfaces** → alias legibles en `INTERFACES`.
  - Define `PlotConfig` y `PLOT_CONFIG` (títulos, ejes y colores de `rssi`, `latency`, `jitter`, `loss`).


## `collectors.py` *(módulo de monitorización refactorizado)*

- **Qué es:** Conjunto de colectores concurrentes + orquestador.
- **Hace:**
  - `APEventCollector`: escucha `iw event -t` y emite eventos de **scan / (des)conexión / reconexión**.
  - `BaseCollector`: base común para hilos (cola, parada limpia, `get_latest()`, `flush_queue()`).
  - `RSSICollector`: obtiene **RSSI** y **BSSID** de la interfaz asociada.
  - `LatencyCollector`: mide **latencia** con `ping` y lanza evento si supera umbral.
  - `Iperf3Collector`: mide **jitter** y **pérdida** (UDP) con `iperf3`.
  - `DataCollector`: orquesta colectores y **agrega** a `Sample`; imprime filas, registra log y muestra resumen.
    - Modos:
      - **`wifi`**: RSSI + (latencia/jitter/pérdida si hay `target`).
      - **`camera`**: integra métricas ROS 2 vía `CameraDiagCollector` (si está disponible).


## `models.py`

- **Qué es:** Modelos de datos tipados para el flujo de métricas/eventos.
- **Hace:**
  - `Sample`: una muestra unificada (**timestamp**, **elapsed**, **rssi**, **ap_mac/ap_name**, **latency**, **jitter**, **loss**).
  - `StatusUpdate`: cambio de estado con `APEventType` (+ `info` opcional) y propiedad `name` lista para UI.
  - `Event`: evento con timestamp formateado a milisegundos para el stream de eventos.


## Estructura de salida / artefactos

- **Logs:** se guardan en `./logs/…` (nombre autogenerado por `build_filepath`).
- **PCAPs:** si se usa `--capture-interface`, se guarda la captura y se ofrece confirmación para conservarla.
- **Imágenes:** al cerrar la sesión de plotting, se ofrece guardar la gráfica.


## Uso

El script tiene dos subcomandos principales: **wifi** y **camera**.  
Puedes ejecutarlo de distintas maneras:

### Formas de ejecución

```bash
# Mostrar ayuda general
python3 main.py --help

# Ayuda de cada subcomando
python3 main.py wifi --help
python3 main.py camera --help
```

### Modo Wi-Fi

```bash
# Modo básico (pide interfaz por UI y objetivo si hace falta)
python3 main.py wifi

# Especificando interfaz y objetivo de ping
python3 main.py wifi wlan0 -t 8.8.8.8

# Intervalo de actualización de 0.5s
python3 main.py wifi wlan0 -t 1.1.1.1 -i 0.5

# Guardar log con etiqueta
python3 main.py wifi wlan0 -t 8.8.8.8 -l -n prueba_pasillo

# Captura de tráfico en paralelo (interfaz en modo monitor)
python3 main.py wifi wlan0 -t 8.8.8.8 -w wlan0mon -n test_pcap
```

### Modo Cámara

```bash
# Modo básico (pide interfaz por UI)
python3 main.py camera

# Especificando interfaz y tópico de diagnósticos
python3 main.py camera wlan0 --diag-topic /diagnostics

# Intervalo de 0.5s y guardar log
python3 main.py camera wlan0 -i 0.5 -l -n test_cam

# Captura pcap en paralelo
python3 main.py camera wlan0 -w wlan0mon -n cam_capture
```

### Opciones comunes
1. `interface`: interfaz Wi-Fi (ej. wlan0)
2. `-i, --interval`: intervalo en segundos
3. `-w, --capture-interface`: interfaz en modo monitor (tshark)
4. `-l, --log`: guardar log en ./logs/
5. `-n, --name`: etiqueta para ficheros
6. `-t, --target`: **solo wifi**: destino de ping
7. `--diag-topic`: **solo camera**: tópico ROS 2 (def. /diagnostics)

### Ejemplo rápido

```bash
# Wi-Fi con todo: interfaz fija, ping a 127.0.0.1, 0.1s, y captura de nombre "audit_wifi"
python3 main.py wifi wlan0 -t 127.0.0.1 -i 0.1 -w wlan0mon -n audit_wifi
```

## Scripts adicionales

Conjunto de utilidades que **no** forman parte del programa principal, pero aportan funciones de soporte.

### `change_bgscan.sh` (Bash)

- **Qué hace:** Lista redes Wi-Fi configuradas con `wpa_cli list_networks` y **(intenta)** aplicar `set_network <ID> bgscan "simple:30:-120:30000"` a cada una.
- **Modo interactivo:** Si no pasas interfaz, te muestra un menú para elegirla (detecta interfaces con `iwconfig`).
- **Uso:**
  ```bash
  sudo ./change_bgscan.sh <interfaz_wifi>
  # o, para selección interactiva
  sudo ./change_bgscan.sh
  ```


### `summarize_csvs.py` (Python)

* **Qué hace:** Recorre CSVs de monitorización bajo `./YYYY-MM-DD/datos_graficas/<interfaz>/*.csv`, **agrega** métricas por fichero y muestra tablas **resumen por grupo** (Media/Mín/Máx). Incluye enlaces clicables al fichero de origen (si el visor lo permite).
* **Entrada esperada:** CSVs con cabecera de metadatos `#METADATA_START ... #METADATA_END` y campos numéricos (latencia, jitter, pérdida, etc.).
* **Agrupaciones disponibles:**

  * Sin flags → **Resumen global** (todas las filas, agrupado por *Grupo*).
  * `-d/--day` → agrupa por **Día → Grupo**.
  * `-i/--interface` → agrupa por **Interfaz → Grupo**.
  * `-d -i` juntos → **Día → Interfaz → Grupo** (modo más detallado).
* **Alias de interfaces:** `--iface-alias path.json` para mapear nombres de carpeta a alias legibles (p. ej. `{"wlp0s20f3": "Intel AX211"}`).

* **Uso:**

  ```bash
  # En el directorio raíz que contiene subcarpetas YYYY-MM-DD/...
  python3 results_parser.py

  # Con alias de interfaces y agrupación por Día -> Interfaz
  python3 summarize_csvs.py --iface-alias iface_alias.json -d -i

  # Solo por Interfaz -> Grupo
  python3 summarize_csvs.py -i

  # Solo por Día -> Grupo
  python3 summarize_csvs.py -d
  ```
