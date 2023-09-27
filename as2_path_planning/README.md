# NOTAS

- La celda (0, 0) del grid corresponde a la esquina inferior derecha.
- El occupancy grid sigue el frame indicado en el header. El origin es la posición de la casilla inferior derecha (0, 0) del grid en este frame.
- Un mapa de ocupación se define unicamente con **WIDTH**, **HEIGHT** y **RESOLUTION**.

- Las medidas del laser hay que convertirlas al frame en el que se quiera construir el grid.

- En una imagen el pixel (0, 0) corresponde a la esquina superior izquierda.
- En una imagen el frame utilizado es Right-Down. Eje X crece hacia la derecha y eje Y crece hacia abajo.

- La conversión entre celda del mapa de ocupación y mundo (en el frame indicado por el mapa) es:

```python
cell = [50, 150]
point = cell * grid.resolution + grid.origin
```

- La conversión entre punto del mundo (**OJO, esta coordenada tiene que estar en el mismo frame que el grid**) y celda del mapa de ocupación es:

```python
point = [4.5, 2.0]
cell = int(round((point - grid.origin) / grid.resolution))
```

<!-- - De esta forma, usando la posicion del drone como frame (*base_link*) para obtener un mapa dinamico: -->
- **REVISAR ESTO:** Calculo del origen del mapa de ocupacion:
```python
origin.x = WIDTH / 2 * RESOLUTION
origin.y = HEIGHT / 2 * RESOLUTION
```

- Mapa de ocupación:
    - O -> Libre
    - 100 -> Ocupado

- Imagen:
    - 0 -> Ocupado: Negro
    - 255 -> Libre: Blanco

# KNOWN ISSUES:
- TODO: When map size is smaller than scan max range it doesn't fill the grid. Not fixed, range sensors reduced and world with walls at the map limits