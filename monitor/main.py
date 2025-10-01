#!/usr/bin/env python3
# main.py

import os
import subprocess
import time
from datetime import datetime
from typing import Optional
import typer

from utils import build_filepath, get_interface_display_name
from theme import console
import system_utils
import ui
from plotting import RealTimePlot, MATPLOTLIB_AVAILABLE

app = typer.Typer(add_completion=False, help="Monitor Wi-Fi/cámara con modos separados.")

# ------------------------------
# Helpers existentes (sin cambios)
# ------------------------------
def setup_logging(interface_name: str, name: Optional[str]) -> Optional[object]:
    log_filename = build_filepath(
        category='logs',
        interface=interface_name,
        name=name,
        timestamp=datetime.now(),
        prefix='mon',
        extension='log'
    )
    try:
        log_file = open(log_filename, 'w', encoding='utf-8')
        console.print(f"Guardando log en: {log_filename}")
        return log_file
    except IOError as e:
        console.print(f"[error]Error al abrir el archivo de log: {e}[/error]")
        return None

def get_interface_channel(interface: str) -> Optional[int]:
    try:
        out = subprocess.check_output(
            ["iw", "dev", interface, "info"],
            stderr=subprocess.DEVNULL,
            text=True
        )
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("channel"):
                parts = line.split()
                if len(parts) >= 2 and parts[1].isdigit():
                    return int(parts[1])
    except Exception:
        pass
    return None

def set_interface_channel(interface: str, channel: int) -> bool:
    try:
        subprocess.run(
            ["sudo", "iw", "dev", interface, "set", "channel", str(channel)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return True
    except subprocess.CalledProcessError as e:
        console.print(f"[error]No se pudo cambiar el canal de {interface} a {channel}: {e}[/error]")
        return False

# ------------------------------
# Lógica común factorizada
# ------------------------------
def _resolve_interfaces_and_align(
    interface: Optional[str],
    capture_interface: Optional[str],
) -> tuple[str, Optional[str]]:
    """Selecciona interfaces y alinea canal si corresponde."""
    # Interfaz gestionada
    mon_iface = interface or ui.select_interface_dialog(mode='Managed')
    if not mon_iface:
        console.print("[error]No se seleccionó una interfaz de monitorización. Saliendo.[/error]")
        raise typer.Exit(code=1)

    # Interfaz monitor para captura (opcional)
    capture_iface = capture_interface if capture_interface is not None else ui.select_interface_dialog(mode='Monitor')

    # Alineación de canal si ambas existen
    if capture_iface:
        managed_chan = get_interface_channel(mon_iface)
        monitor_chan = get_interface_channel(capture_iface)

        console.print("")
        if managed_chan is None:
            console.print(f"[warning]No se pudo obtener el canal de {mon_iface}[/warning]")
        if monitor_chan is None:
            console.print(f"[warning]No se pudo obtener el canal de {capture_iface}[/warning]")

        if managed_chan and monitor_chan and managed_chan != monitor_chan:
            console.print(f"[warn]Cambiando canal de {capture_iface} de {monitor_chan} -> {managed_chan}[/]")
            if set_interface_channel(capture_iface, managed_chan):
                time.sleep(0.5)
                console.print(f"[success]{capture_iface} ahora en canal {managed_chan}[/success]")
            else:
                console.print(f"[error]No se pudo alinear el canal de {capture_iface}[/error]")
            console.print("")
    return mon_iface, capture_iface

def _run_common(
    *,
    source: str,                  # "wifi" | "camera"
    interface: Optional[str],
    interval: float,
    capture_interface: Optional[str],
    log: bool,
    name: Optional[str],
    target: Optional[str],        # usado solo en wifi
    diagnostics_topic: str,       # usado solo en camera
):
    """Orquesta ejecución compartida según el source."""
    if source == "camera":
        try:
            from ros_collector import CameraDiagCollector  # no rompe si falta; sería None
        except Exception:
            CameraDiagCollector = None
        if CameraDiagCollector is None:
            console.print("[error]ROS 2 no está disponible en este sistema. "
                          "El modo 'camera' requiere rclpy y diagnostic_msgs.[/error]")
            raise typer.Exit(code=2)
    
    system_utils.check_dependencies()

    # Selección/alineación
    mon_iface, cap_iface = _resolve_interfaces_and_align(interface, capture_interface)

    # Logging
    log_file = setup_logging(mon_iface, name) if log else None

    # target solo en wifi + cuando hay Matplotlib (como ya hacías)
    if MATPLOTLIB_AVAILABLE and source == 'wifi':
        target = target or ui.get_target_dialog()
        if not target:
            console.print("[error]El objetivo de ping es obligatorio para la gráfica. Saliendo.[/error]")
            raise typer.Exit(code=1)
    else:
        target = None

    # TShark opcional
    tshark_proc = pcap_path = None
    if cap_iface:
        tshark_proc, pcap_path = system_utils.start_tshark_capture(cap_iface, mon_iface, name)

    plotter = None
    try:
        if not MATPLOTLIB_AVAILABLE:
            console.print("[error]Matplotlib no disponible: no se puede mostrar la gráfica.[/error]")
            raise typer.Exit(code=1)

        plotter = RealTimePlot(
            mon_iface,
            interval,
            log_file,
            target if source == 'wifi' else None,
            name,
            source=source,
            diagnostics_topic=diagnostics_topic
        )
        plotter.start()

    except (KeyboardInterrupt, SystemExit):
        console.print("\n\n[error]Programa detenido por el usuario.[/error]")

    finally:
        if tshark_proc:
            system_utils.stop_tshark_capture(tshark_proc)

        if log_file:
            log_file.close()
            console.print("[success]Log cerrado.[/success]")

        ui.save_plot_dialog(plotter)

        if pcap_path:
            ui.confirm_save("Captura de tshark", pcap_path)

        console.print("\n[success]Script finalizado.[/success]")

# ------------------------------
# Subcomandos Typer
# ------------------------------

@app.command("wifi")
def cmd_wifi(
    interface: str = typer.Argument(None, help="Interfaz Wi-Fi a monitorizar (ej: wlan0)."),
    interval: float = typer.Option(1.0, "-i", "--interval", help="Intervalo en segundos (def: 1.0)."),
    capture_interface: str = typer.Option(None, "-w", "--capture-interface", help="Interfaz para captura con TShark."),
    log: bool = typer.Option(False, "-l", "--log", help="Guardar la salida en un archivo de log."),
    target: Optional[str] = typer.Option(None, "-t", "--target", help="IP/host para latencia, jitter y pérdida."),
    name: str = typer.Option(None, "-n", "--name", help="Etiqueta para ficheros."),
):
    """Modo Wi-Fi: RSSI + ping + iperf3 (si procede)."""
    _run_common(
        source="wifi",
        interface=interface,
        interval=interval,
        capture_interface=capture_interface,
        log=log,
        name=name,
        target=target,                 # requerido (o se pide por UI) si hay Matplotlib
        diagnostics_topic="/diagnostics"  # ignorado en wifi
    )

@app.command("camera")
def cmd_camera(
    interface: str = typer.Argument(None, help="Interfaz Wi-Fi asociada a la cámara (para RSSI/AP)."),
    interval: float = typer.Option(1.0, "-i", "--interval", help="Intervalo en segundos (def: 1.0)."),
    capture_interface: str = typer.Option(None, "-w", "--capture-interface", help="Interfaz para captura con TShark."),
    log: bool = typer.Option(False, "-l", "--log", help="Guardar la salida en un archivo de log."),
    name: str = typer.Option(None, "-n", "--name", help="Etiqueta para ficheros."),
    diagnostics_topic: str = typer.Option("/diagnostics", "--diag-topic", help="Tópico ROS 2 para DiagnosticArray."),
):
    """Modo Cámara: RSSI/AP + métricas desde ROS2 (/diagnostics)."""
    _run_common(
        source="camera",
        interface=interface,
        interval=interval,
        capture_interface=capture_interface,
        log=log,
        name=name,
        target=None,                    # no aplica en cámara
        diagnostics_topic=diagnostics_topic
    )

@app.callback(invoke_without_command=True)
def main(ctx: typer.Context):
    """
    Monitor Wi-Fi/Cámara
    """
    if ctx.invoked_subcommand is None:
        # Si no se invocó ningún subcomando, mostrar ayuda
        console = typer.rich_utils.Console()
        console.print(ctx.get_help())
        raise typer.Exit()

if __name__ == "__main__":
    app()
