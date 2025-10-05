#!/usr/bin/env python3
import sys

def main():
    if len(sys.argv) < 4:
        print("Uso: fire_action.py <rpi_id> <x> <y>")
        return

    rpi_id = sys.argv[1]
    x = float(sys.argv[2])
    y = float(sys.argv[3])

    print(f"Acción ejecutada para {rpi_id} en coordenadas x={x}, y={y}")
    # Aquí puedes agregar la lógica que quieras, p.ej., activar sirena, enviar alerta, etc.

if __name__ == "__main__":
    main()
