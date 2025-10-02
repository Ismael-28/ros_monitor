> Este es un fork de [tb2_unizar](https://github.com/dvdmc/tb2_unizar) desarrollado por [David Morilla Cabello](https://github.com/dvdmc), y expandido por [Ismael Berdusán Muñoz](https://github.com/Ismael-28).

# Instalación y ejecución con Docker

Se proporciona un Dockerfile para ejecutar el sistema dentro de Docker. Esto permite instalar fácilmente todas las dependencias. Además, este Dockerfile sirve como referencia de todo lo que debería configurarse en una máquina host para tener el sistema listo. Puedes instalar y **ejecutar (punto de entrada para las demos)** con:

```bash
xhost + # Permitir que Docker use la interfaz gráfica del host
sudo docker compose up -d # Ejecuta el docker-compose.yml y mantiene el contenedor en segundo plano. La primera vez tardará en compilarse (~20 min en un Intel NUC).
sudo docker exec -it tb2_unizar /bin/bash # Abrir una terminal dentro del contenedor Docker.
````

Por defecto, habrá un workspace llamado `tb2_unizar_compiled_ws` con todo lo necesario para las demos.

Es necesario instalar Docker tanto en el ordenador del robot (para ejecutar toda la navegación) como en la estación de control (para usar RViz y enviar metas).


## Uso básico (demos)

Una vez dentro del contenedor Docker, carga el workspace en la terminal:

```bash
source tb2_unizar_compiled_ws/install/setup.bash
```

Muévete al paquete `tb2_unizar`:

```bash
cd tb2_unizar_ws/src/tb2_unizar
```

Finalmente, lanza el comando:

```bash
./start_turtlebot.bash -m teleop
```

Esto iniciará todo lo necesario para la navegación del robot con la cámara conectada.

Para la teleoperación y visualización, en otro ordenador puedes ejecutar Docker, abrir una terminal dentro del contenedor y lanzar `launch_turtlebot_keyboard.yaml`:

```bash
source tb2_unizar_compiled_ws/install/setup.bash
cd tb2_unizar_ws/src/tb2_unizar/
./start_turtlebot.bash -m keyboard
```


## Cómo detener todo

En ambos equipos ejecutar `./stop_turtlebot.bash` en cualquiera de las terminales abiertas, y luego detener el contenedor:

```bash
exit
sudo docker compose down
```


## Grabaciones

Las grabaciones de la cámara se deberían guardar en un fichero `records` dentro de `tb2_unizar`.

## Métricas en tiempo real

Para visualizar métricas en tiempo real, abre otro terminal dentro del contenedor y ve a `monitor`:

```bash
cd monitor

# Crear un entorno virtual e instalar las dependencias si es la primera vez
python3 -m venv venv
source venv/bin/activate 
pip install -r requirements.txt
```

Sigue la [guía](monitor/guía.md) del módulo `monitor`.