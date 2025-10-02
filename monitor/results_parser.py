#!/usr/bin/env python3
"""
Procesa CSV de monitorización bajo rutas:
  ./YYYY-MM-DD/datos_graficas/<interfaz>/*.csv

Agrupa por DÍA -> INTERFAZ -> GRUPO (nombre base de fichero),
mostrando medias por fichero y filas resumen (Media, Mínimo, Máximo).
- Sin flags: muestra UNA tabla global (todas las filas juntas), agrupada solo por Grupo.
- Con --day/-d: agrupa por DÍA -> GRUPO.
- Con --interface/-i: agrupa por INTERFAZ -> GRUPO.
- Con ambos --day y --interface: comportamiento original DÍA -> INTERFAZ -> GRUPO.

Las filas de Mín/Máx del grupo se calculan sobre datos crudos (no sobre medias)
y cada cifra enlaza al fichero CSV que aporta el extremo.
Permite alias legibles de interfaces vía --iface-alias JSON.
"""

import os
import re
import glob
import json
import argparse
import pandas as pd
from typing import Dict, List, Tuple, Optional

from rich.console import Console, Group
from rich.table import Table
from rich.rule import Rule
from rich.panel import Panel
from rich.align import Align
from rich.text import Text

console = Console()


# ---------------------------- Utilidades de ruta ---------------------------- #

DAY_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")

def find_day_iface_csvs(root: str) -> List[str]:
    """
    Busca todos los CSV con patrón:
      <root>/<YYYY-MM-DD>/datos_graficas/<interfaz>/*.csv
    """
    pattern = os.path.join(root, "*", "datos_graficas", "*", "*.csv")
    return sorted(glob.glob(pattern))

def split_path_parts(path: str) -> List[str]:
    return os.path.normpath(path).split(os.sep)

def extract_day_and_iface(path: str) -> Tuple[Optional[str], Optional[str]]:
    """
    Extrae el 'día' (YYYY-MM-DD) y la 'interfaz' (carpeta tras datos_graficas)
    a partir de la ruta del CSV.
    """
    parts = split_path_parts(path)
    day = None
    iface = None
    # Día: primer componente que cumpla YYYY-MM-DD
    for p in parts:
        if DAY_RE.match(p):
            day = p
            break
    # Interfaz: componente inmediatamente después de 'datos_graficas'
    if "datos_graficas" in parts:
        try:
            i = parts.index("datos_graficas")
            iface = parts[i + 1]
        except Exception:
            iface = None
    return day, iface


# -------------------------- Lectura y métricas CSV -------------------------- #

def parse_csv_file(filepath: str):
    """
    Lee el CSV con metadatos y devuelve:
      - df: DataFrame con datos numéricos
      - events: lista de eventos (objetos con 'msg', 'ts', etc.)
    """
    metadata = {}
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        line = f.readline().strip()
        if line != "#METADATA_START":
            raise ValueError(f"{filepath} no tiene bloque METADATA_START")
        while True:
            line = f.readline().strip()
            if line == "#METADATA_END":
                break
            if line.startswith("#"):
                key, val = line[1:].split(',', 1)
                metadata[key] = val
        df = pd.read_csv(f)

    events = json.loads(metadata.get('Event_list_JSON', '[]'))
    return df, events

def compute_durations(events: list, keyword: str) -> List[float]:
    """
    Extrae duraciones (ms) de eventos cuyo mensaje contenga `keyword`,
    con formato 'Keyword: 123.456 ms'
    """
    durations = []
    pattern = re.compile(rf"{re.escape(keyword)}: ([\d\.]+) ms")
    for ev in events:
        msg = ev.get('msg', '')
        m = pattern.search(msg)
        if m:
            durations.append(float(m.group(1)))
    return durations


# --------------------------- Aliases de interfaces -------------------------- #

def load_iface_aliases(json_path: Optional[str]) -> Dict[str, str]:
    """
    Carga un diccionario {carpeta_iface: alias legible} desde JSON, si existe.
    """
    if not json_path:
        return {}
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            if isinstance(data, dict):
                # normaliza claves a str simples
                return {str(k): str(v) for k, v in data.items()}
    except Exception as e:
        console.print(f"[yellow]Aviso:[/] no se pudo leer --iface-alias: {e}")
    return {}

def infer_friendly(name: str) -> str:
    """
    Alias por defecto a partir del nombre de carpeta: guiones bajos→espacios, title-case.
    """
    if not name:
        return "Desconocida"
    return name.replace("_", " ").strip().title()


# ------------------------------- Renderizado -------------------------------- #

DROP_COLS_BASE = ["Archivo", "ArchivoPath", "FilenameNoExt", "Base", "Grupo", "Día", "Interfaz", "InterfazAlias"]

def build_group_tables(subset_df: pd.DataFrame) -> List[Align]:
    """
    Crea tablas por 'Grupo' para un subconjunto de df_summary,
    con filas por fichero (medias) y filas resumen (Media, Min, Max con enlaces).
    """
    group_tables: List[Align] = []

    for grupo, subdf in subset_df.groupby("Grupo"):
        # Columnas visibles: 'Archivo' + métricas (excluye auxiliares __min/__max y *_min/_max(ms))
        def is_aux(c: str) -> bool:
            return c.endswith("__min") or c.endswith("__max") or c.endswith("_min(ms)") or c.endswith("_max(ms)")

        cols = ["Archivo"] + [c for c in subdf.columns if c not in DROP_COLS_BASE and not is_aux(c)]

        table = Table(show_header=True, header_style="bold cyan", title=f"Grupo: {grupo}")
        for col in cols:
            table.add_column(col, justify=("left" if col == "Archivo" else "right"))

        # Filas por fichero (medias visibles)
        for _, row in subdf.iterrows():
            table.add_row(*[str(row[col]) if pd.notna(row[col]) else "-" for col in cols])

        # --- Filas resumen del GRUPO ---
        table.add_section()

        numeric_visible_cols = [c for c in cols if c != "Archivo"]
        numeric_visible = subdf[numeric_visible_cols].select_dtypes(include=["number"])
        media_series = numeric_visible.mean(numeric_only=True).round(3) if not numeric_visible.empty else pd.Series(dtype=float)

        # Para Mín/Máx: usar auxiliares __min/__max (o *_min/_max(ms) en eventos)
        def aux_min_col(c: str) -> Optional[str]:
            if f"{c}__min" in subdf.columns:
                return f"{c}__min"
            cand = c.replace("(ms)", "_min(ms)")
            return cand if cand in subdf.columns else None

        def aux_max_col(c: str) -> Optional[str]:
            if f"{c}__max" in subdf.columns:
                return f"{c}__max"
            cand = c.replace("(ms)", "_max(ms)")
            return cand if cand in subdf.columns else None

        min_vals, max_vals, min_src, max_src = {}, {}, {}, {}
        for c in numeric_visible_cols:
            cmin = aux_min_col(c)
            cmax = aux_max_col(c)
            if cmin and cmin in subdf.columns:
                serie_min = subdf[cmin].dropna()
                if not serie_min.empty:
                    vmin = serie_min.min()
                    min_vals[c] = round(float(vmin), 3)
                    im = serie_min.idxmin()
                    min_src[c] = subdf.loc[im, "Archivo"]
                else:
                    min_vals[c] = ""
                    min_src[c] = ""
            else:
                min_vals[c] = ""
                min_src[c] = ""

            if cmax and cmax in subdf.columns:
                serie_max = subdf[cmax].dropna()
                if not serie_max.empty:
                    vmax = serie_max.max()
                    max_vals[c] = round(float(vmax), 3)
                    ia = serie_max.idxmax()
                    max_src[c] = subdf.loc[ia, "Archivo"]
                else:
                    max_vals[c] = ""
                    max_src[c] = ""
            else:
                max_vals[c] = ""
                max_src[c] = ""


        # Mapa fichero -> ruta
        fname2path = dict(zip(subdf["Archivo"], subdf["ArchivoPath"]))

        def link_cell(value, fname):
            if value in ("", None) or not fname:
                return "-" if value in ("", None) else str(value)
            path = fname2path.get(fname, "")
            if not path:
                return str(value)
            uri = f"file://{os.path.abspath(path)}"
            t = Text(str(value))
            t.stylize(f"link {uri}")
            return t

        def row_from_series(label: str, ser: pd.Series) -> List[str]:
            return [label] + [str(ser.get(c, "")) for c in numeric_visible_cols]

        def row_with_links(label: str, values_map: Dict[str, object], src_map: Dict[str, str]) -> List[object]:
            return [label] + [link_cell(values_map.get(c, ""), src_map.get(c, "")) for c in numeric_visible_cols]

        # Filas resumen
        table.add_row(*row_from_series("Media",  media_series), style="bold yellow")
        table.add_row(*row_with_links("Mínimo", min_vals, min_src), style="bold green")
        table.add_row(*row_with_links("Máximo", max_vals, max_src), style="bold red")

        group_tables.append(Align.center(table))

    return group_tables


# --------------------------------- Main ------------------------------------ #

def main():
    parser = argparse.ArgumentParser(
        description="Agrupa CSV por día → interfaz → grupo y muestra medias por fichero y resumen por grupo."
    )
    parser.add_argument(
        "directory",
        nargs="?",
        default=".",
        help="Directorio raíz (por defecto, el directorio actual)."
    )
    parser.add_argument(
        "--iface-alias",
        help="JSON con alias de interfaces (p.ej. {'Intel':'Intel AX211','alfa1':'Alfa 1 (AWUS036ACHM)'})."
    )
    # NUEVOS FLAGS
    parser.add_argument(
        "-d", "--day",
        action="store_true",
        help="Agrupa por DÍA (si se combina con -i, será DÍA → INTERFAZ)."
    )
    parser.add_argument(
        "-i", "--interface",
        action="store_true",
        help="Agrupa por INTERFAZ (si se combina con -d, será DÍA → INTERFAZ)."
    )

    args = parser.parse_args()

    alias_map = load_iface_aliases(args.iface_alias)

    files = find_day_iface_csvs(args.directory)
    if not files:
        console.print(f"[red]No se encontraron CSV bajo {args.directory}/*/datos_graficas/*/*.csv[/]")
        return

    console.print(Rule("Procesando CSV", style="green"))

    records = []
    for path in files:
        filename = os.path.basename(path)
        day, iface = extract_day_and_iface(path)
        if not day or not iface:
            console.print(f"[yellow]Saltando (ruta no estándar):[/] {path}")
            continue

        console.print(f"- Leyendo [bold cyan]{filename}[/]  (día: {day}, interfaz: {iface})")

        try:
            df, events = parse_csv_file(path)
        except Exception as e:
            console.print(f"[red]Error leyendo {filename}: {e}[/]")
            continue

        # --- Medias por fichero (visibles en la tabla) ---
        num_df = df.select_dtypes(include=["number"])
        means = num_df.mean().round(3).to_dict()

        # --- Extremos por fichero (auxiliares ocultos para resumen del grupo sobre datos crudos) ---
        mins = num_df.min().round(3).to_dict()
        maxs = num_df.max().round(3).to_dict()

        # Duraciones desde eventos
        roam_list = compute_durations(events, "Tiempo reconexión")
        scan_list = compute_durations(events, "Duración escaneo")

        # Medias de eventos (visibles)
        means["Roaming(ms)"] = round(sum(roam_list)/len(roam_list), 3) if roam_list else None
        means["Escaneo(ms)"] = round(sum(scan_list)/len(scan_list), 3) if scan_list else None

        # Extremos de eventos (auxiliares ocultos)
        means["Roaming_min(ms)"] = round(min(roam_list), 3) if roam_list else None
        means["Roaming_max(ms)"] = round(max(roam_list), 3) if roam_list else None
        means["Escaneo_min(ms)"] = round(min(scan_list), 3) if scan_list else None
        means["Escaneo_max(ms)"] = round(max(scan_list), 3) if scan_list else None

        filename_no_ext = filename[:-4] if filename.lower().endswith(".csv") else filename
        base = filename_no_ext.rsplit("_", 1)[0]
        grupo = base.rsplit("-", 1)[0]

        record = {
            "Día": day,
            "Interfaz": iface,
            "InterfazAlias": alias_map.get(iface, infer_friendly(iface)),
            "Archivo": filename,
            "ArchivoPath": path,   # ruta completa para enlaces
            "FilenameNoExt": filename_no_ext,
            "Base": base,
            "Grupo": grupo,
        }
        # añadir medias visibles
        record.update(means)
        # añadir auxiliares por cada columna numérica del CSV (para resumen min/max real)
        for col in num_df.columns:
            record[f"{col}__min"] = mins.get(col, None)
            record[f"{col}__max"] = maxs.get(col, None)

        records.append(record)

    if not records:
        console.print("[yellow]No hay datos válidos para procesar.[/]")
        return

    df_summary = pd.DataFrame(records)

    # --------------------------- Lógica de agrupación --------------------------- #
    group_by_day = args.day
    group_by_iface = args.interface

    # Caso 1: ambos -> DÍA -> INTERFAZ -> GRUPO (comportamiento original)
    if group_by_day and group_by_iface:
        for day, df_day in df_summary.groupby("Día"):
            day_renderables = []
            for iface, df_iface in df_day.groupby("Interfaz"):
                alias = df_iface["InterfazAlias"].iloc[0]
                group_tables = build_group_tables(df_iface)
                iface_panel = Panel(
                    Group(*group_tables) if group_tables else "Sin datos",
                    title=f"[bold]Interfaz:[/] {alias}  [dim](carpeta: {iface})[/dim]",
                    border_style="magenta"
                )
                day_renderables.append(iface_panel)

            day_panel = Panel(
                Group(*day_renderables) if day_renderables else "Sin datos",
                title=f"[bold green]Día: {day}[/bold green]",
                border_style="green"
            )
            console.print(day_panel)

    # Caso 2: solo día -> DÍA -> GRUPO
    elif group_by_day and not group_by_iface:
        for day, df_day in df_summary.groupby("Día"):
            group_tables = build_group_tables(df_day)
            day_panel = Panel(
                Group(*group_tables) if group_tables else "Sin datos",
                title=f"[bold green]Día: {day}[/bold green]",
                border_style="green"
            )
            console.print(day_panel)

    # Caso 3: solo interfaz -> INTERFAZ -> GRUPO
    elif group_by_iface and not group_by_day:
        for iface, df_iface in df_summary.groupby("Interfaz"):
            alias = df_iface["InterfazAlias"].iloc[0]
            group_tables = build_group_tables(df_iface)
            iface_panel = Panel(
                Group(*group_tables) if group_tables else "Sin datos",
                title=f"[bold]Interfaz:[/] {alias}  [dim](carpeta: {iface})[/dim]",
                border_style="magenta"
            )
            console.print(iface_panel)

    # Caso 4: sin flags -> UNA tabla global (todas las filas), agrupada solo por GRUPO
    else:
        group_tables = build_group_tables(df_summary)
        global_panel = Panel(
            Group(*group_tables) if group_tables else "Sin datos",
            title=f"[bold]Resumen global (todas las interfaces y días)[/bold]",
            border_style="cyan"
        )
        console.print(global_panel)


if __name__ == "__main__":
    main()
